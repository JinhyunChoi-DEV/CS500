#include "Shape.h"

#include <math.h>
#include "Auxiliary.h"
#include "Intersection.h"
#include "Interval.h"
#include "CalculationHelper.h"
#include "HDRReader.h"
#include "Ray.h"
#include "raytrace.h"

float Sign(float x)
{
	return (x >= epsilon) ? 1.0f : -1.0f;
}

Shape::Shape(Material* material)
{
	const float s = length(material->Kd) + length(material->Ks) + length(material->Kt);
	p_d = length(material->Kd) / s;
	p_r = length(material->Ks) / s;
	p_t = length(material->Kt) / s;
}

float Shape::GetSmallestPositiveValue(float t0, float t1)
{
	if (t0 < epsilon)
		return t1;

	if (t0 < t1)
		return t0;

	return t1;
}

vec3 Shape::EvalRadiance(const Intersection& A)
{
	IBL* ibl = dynamic_cast<IBL*>(this);

	if (ibl != nullptr)
	{
		vec3 P = normalize(A.point);

		double u = (ibl->angle - atan2(P[1], P[0])) / (PI * 2);
		u = u - floor(u);
		double v = acos(P[2]) / PI;
		int i0 = floor(u * ibl->width);
		int j0 = floor(v * ibl->height);
		double uw[2], vw[2];
		uw[1] = u * ibl->width - i0;
		uw[0] = 1.0 - uw[1];
		vw[1] = v * ibl->height - j0;
		vw[0] = 1.0 - vw[1];

		vec3 r(0.0f);
		for (int i = 0; i < 2; i++)
		{
			for (int j = 0; j < 2; j++)
			{
				int k = 3 * (((j0 + j) % ibl->height) * ibl->width + ((i0 + i) % ibl->width));
				for (int c = 0; c < 3; c++)
				{
					r[c] += uw[i] * vw[j] * ibl->image[k + c];
				}
			}
		}

		if (r.x > 1.0f)
			r.x = 1.0f;
		if (r.y > 1.0f)
			r.y = 1.0f;
		if (r.z > 1.0f)
			r.z = 1.0f;

		return r;
	}
	else
	{
		return material->Kd;
	}
}

float Shape::PdfLight(int lightSize, const Intersection& B)
{
	IBL* ibl = dynamic_cast<IBL*>(this);

	if (ibl != nullptr)
	{
		vec3 P = normalize(B.point);

		double fu = (ibl->angle - atan2(P[1], P[0])) / 2.0 * PI;
		fu = fu - floor(fu);

		int u = floor(ibl->width * fu);
		int v = floor(ibl->height * acos(P[2]) / PI);
		float angleFrac = PI / (float)ibl->height;
		float* pVDist = &ibl->pBuffer[ibl->height * u];

		float pdfU = (u == 0) ? (ibl->pUDist[0]) : (ibl->pUDist[u] - ibl->pUDist[u - 1]);
		pdfU /= ibl->pUDist[ibl->width - 1];
		pdfU *= ibl->width / (2.0f * PI);

		float pdfV = (v == 0) ? (pVDist[0]) : (pVDist[v] - pVDist[v - 1]);
		pdfV /= pVDist[ibl->height - 1];
		pdfV *= ibl->height / PI;

		float theta = angleFrac * 0.5f + angleFrac * v;
		float pdf = pdfU * pdfV * sinf(theta) / (4.0f * PI * ibl->radius * ibl->radius);
		return pdf;
	}
	else
	{
		Sphere* sphere = dynamic_cast<Sphere*>(this);
		const float areaOfLightSphere = 4 * PI * powf(sphere->radius, 2);
		return 1.0f / (areaOfLightSphere * (float)lightSize);
	}
}

vec3 Shape::SampleBRDF(vec3 omegaO, vec3 normal)
{
	const float chooseFactor = myrandomf(RNGen);
	const float e1 = myrandomf(RNGen);
	const float e2 = myrandomf(RNGen);

	if (chooseFactor < p_d)
	{
		float theta = sqrtf(e1);
		float phi = 2.0f * PI * e2;
		return SampleLobe(normal, theta, phi);
	}

	float theta = GetDistributionCos(material, e1);
	float phi = 2.0f * PI * e2;
	vec3 m = SampleLobe(normal, theta, phi);

	if (chooseFactor < p_d + p_r)
		return normalize(2.0f * abs(dot(omegaO, m)) * m - omegaO);

	float etaI;
	float etaO;
	float eta;
	if (dot(omegaO, normal) > epsilon)
	{
		etaI = 1.0f;
		etaO = material->IOR;
	}
	else if (dot(omegaO, normal) < epsilon)
	{
		etaI = material->IOR;
		etaO = 1.0f;
	}
	eta = etaI / etaO;
	float r = 1.0f - powf(eta, 2) * (1.0f - powf(dot(omegaO, m), 2));
	if (r < epsilon)
		return normalize(2.0f * abs(dot(omegaO, m)) * m - omegaO);

	return normalize((eta * dot(omegaO, m) - Sign(dot(omegaO, normal)) * sqrtf(r)) * m - eta * omegaO);
}

vec3 Shape::EvalScattering(vec3 omegaO, vec3 normal, vec3 omegaI, float t)
{
	const vec3 E_d = Diffuse_EvalScattering(material);
	const vec3 E_r = Reflection_EvalScattering(omegaO, normal, omegaI, material);

	float etaI;
	float etaO;
	float eta;
	if (dot(omegaO, normal) > epsilon)
	{
		etaI = 1.0f;
		etaO = material->IOR;
	}
	else if (dot(omegaO, normal) < epsilon)
	{
		etaI = material->IOR;
		etaO = 1.0f;
	}
	eta = etaI / etaO;
	const vec3 E_t = Transmission_EvalScattering(omegaO, normal, omegaI, material, etaI, etaO, etaI, t);

	return abs(dot(normal, omegaI)) * (E_d + E_r + E_t);
}

float Shape::PdfBRDF(vec3 omegaO, vec3 normal, vec3 omegaI)
{
	const float P_d = Diffuse_Probability(omegaI, normal);
	const float P_r = Reflection_Probability(omegaO, normal, omegaI, material);

	float etaI;
	float etaO;
	float eta;
	if (dot(omegaO, normal) > epsilon)
	{
		etaI = 1.0f;
		etaO = material->IOR;
	}
	else if (dot(omegaO, normal) < epsilon)
	{
		etaI = material->IOR;
		etaO = 1.0f;
	}
	eta = etaI / etaO;
	const float P_t = Transmission_Probability(omegaO, normal, omegaI, material, etaO, etaI, eta);

	return (p_d * P_d) + (p_r * P_r) + (p_t * P_t);
}

void Shape::AffectMotionBlur(vec3& center)
{
	float t = myrandomf(RNGen);
	t = 1.0f - powf((1.0f - t), 2);

	vec3 A = center;
	vec3 B = center1;
	vec3 C = center2;

	vec3 Pt = (powf(1.0f - t, 2) * A) + (2.0f * t * (1.0f - t) * B) + (powf(t, 2) * C);
	center = Pt;
}

Sphere::Sphere(const vec3 center_, const float r, Material* mat) : Shape(mat)
{
	radius = r;

	material = mat;
	base = center_;
}

void Sphere::CreateBV()
{
	if (activeMotionBlur)
	{
		min = glm::min(glm::min(base, center1), center2) - vec3(radius);
		max = glm::max(glm::max(base, center1), center2) + vec3(radius);
	}
	else
	{
		min = base - vec3(radius);
		max = base + vec3(radius);
	}
}

bool Sphere::intersect(Ray ray, Intersection& intersection)
{
	vec3 center = base;
	if (activeMotionBlur)
		AffectMotionBlur(center);

	const vec3 Q = (ray.Q - center);

	const float a = dot(ray.D, ray.D);
	const float b = 2.0f * dot(Q, ray.D);
	const float c = dot(Q, Q) - powf(radius, 2);

	float discriminant = powf(b, 2) - (4.0f * a * c);

	if (discriminant < epsilon)
		return false;

	const float t0 = (-b - sqrt(discriminant)) / (2.0f * a);
	const float t1 = (-b + sqrt(discriminant)) / (2.0f * a);

	if (t0 < epsilon && t1 < epsilon)
		return false;

	if (t1 < epsilon)
		return false;

	intersection.object = this;
	intersection.t = GetSmallestPositiveValue(t0, t1);
	intersection.point = ray.eval(intersection.t);
	intersection.normal = normalize(intersection.point - center);

	return true;
}

Intersection Sphere::SampleSphere()
{
	Intersection result;

	float e1 = myrandomf(RNGen);
	float e2 = myrandomf(RNGen);

	float z = 2.0f * e1 - 1;
	float r = sqrtf(1 - powf(z, 2));
	float a = 2 * PI * e2;

	result.normal = vec3(r * cos(a), r * sin(a), z);
	result.point = base + (radius * result.normal);
	result.object = this;

	return result;
}

Box::Box(const vec3 base_, const vec3 diagonal_, Material* mat) : Shape(mat)
{
	base = base_;
	diagonal = diagonal_;

	material = mat;
}

void Box::CreateBV()
{
	min = base;
	max = base + diagonal;
}

bool Box::intersect(Ray ray, Intersection& intersection)
{
	Slab slab1{ Xaxis(), -base.x, -base.x - diagonal.x };
	Slab slab2{ Yaxis(), -base.y, -base.y - diagonal.y };
	Slab slab3{ Zaxis(), -base.z, -base.z - diagonal.z };

	Interval startI;
	Interval i1, i2, i3;
	i1.intersect(ray, slab1);
	i2.intersect(ray, slab2);
	i3.intersect(ray, slab3);

	const float t0 = std::max(std::max(std::max(startI.t0, i1.t0), i2.t0), i3.t0);
	const float t1 = std::min(std::min(std::min(startI.t1, i1.t1), i2.t1), i3.t1);

	if (t0 < epsilon && t1 < epsilon)
		return false;

	if (t0 > t1)
		return false;

	if (t1 < epsilon)
		return false;

	intersection.object = this;
	intersection.t = GetSmallestPositiveValue(t0, t1);
	intersection.point = ray.eval(intersection.t);
	intersection.normal = GetNormal(intersection.t, t0, t1, i1, i2, i3);

	return true;
}

vec3 Box::GetNormal(float t, float t0, float t1, Interval i1, Interval i2, Interval i3)
{
	vec3 n0 = vec3(1, 0, 0);
	vec3 n1 = vec3(0, 1, 0);
	vec3 n2 = vec3(0, 0, 1);

	if (fabs(t - t0) < epsilon)
	{
		if (fabs(t0 - i1.t0) < epsilon)
			return n0;
		if (fabs(t0 - i2.t0) < epsilon)
			return n1;
		if (fabs(t0 - i3.t0) < epsilon)
			return n2;
	}
	else
	{
		if (fabs(t1 - i1.t1) < epsilon)
			return -n0;
		if (fabs(t1 - i2.t1) < epsilon)
			return -n1;
		if (fabs(t1 - i3.t1) < epsilon)
			return -n2;
	}

	return vec3(0);
}

Triangle::Triangle(const vec3 v0_, const vec3 v1_, const vec3 v2_, const vec3 n0_, const vec3 n1_, const vec3 n2_, Material* mat) : Shape(mat)
{
	v0 = v0_;
	v1 = v1_;
	v2 = v2_;

	n0 = n0_;
	n1 = n1_;
	n2 = n2_;

	base = (v0 + v1 + v2) / 3.0f;
	material = mat;
}

void Triangle::CreateBV()
{
	min = glm::min(glm::min(v0, v1), v2);
	max = glm::max(glm::max(v0, v1), v2);
}

bool Triangle::intersect(Ray ray, Intersection& intersection)
{
	vec3 e1 = v1 - v0;
	vec3 e2 = v2 - v0;
	vec3 p = cross(ray.D, e2);
	float d = dot(p, e1);

	if (d == 0.0f)
		return false;

	vec3 s = ray.Q - v0;
	float u = dot(p, s) / d;

	if (u < epsilon || u > 1.0f)
		return false;

	vec3 q = cross(s, e1);
	float v = dot(ray.D, q) / d;

	if (v < epsilon || (u + v) > 1.0f)
		return false;

	float t = dot(e2, q) / d;
	if (t < epsilon)
		return false;

	intersection.object = this;
	intersection.t = t;
	intersection.point = ray.eval(t);
	intersection.normal = (1 - u - v) * n0 + u * n1 + v * n2;
	return true;
}

Cylinder::Cylinder(const vec3 base_, const vec3 axis_, const float r, Material* mat) : Shape(mat)
{
	axis = axis_;
	radius = r;

	material = mat;
	base = base_;
}

void Cylinder::CreateBV()
{
	vec3 p1 = base + vec3(radius);
	vec3 p2 = base - vec3(radius);
	vec3 p3 = axis + base + vec3(radius);
	vec3 p4 = axis + base - vec3(radius);

	min = glm::min(glm::min(glm::min(p1, p2), p3), p4);
	max = glm::max(glm::max(glm::max(p1, p2), p3), p4);
}

bool Cylinder::intersect(Ray ray, Intersection& intersection)
{
	vec3 A = normalize(axis);
	vec3 v = vec3(myrandomf(RNGen), myrandomf(RNGen), myrandomf(RNGen));
	vec3 B = normalize(cross(v, A));
	vec3 C = normalize(cross(A, B));

	mat3 inverse_R(B, C, A);
	mat3 R = glm::transpose(inverse_R);

	vec3 start = R * (ray.Q - base);
	vec3 dir = normalize(R * ray.D);
	Ray newRay(start, dir);

	// first interval
	Slab slab{ Zaxis(), 0, -length(axis) };
	Interval i1;
	i1.intersect(newRay, slab);

	// second interval
	const float a = (newRay.D.x * newRay.D.x) + (newRay.D.y * newRay.D.y);
	const float b = 2.0f * ((newRay.D.x * newRay.Q.x) + (newRay.D.y * newRay.Q.y));
	const float c = (newRay.Q.x * newRay.Q.x) + (newRay.Q.y * newRay.Q.y) - powf(radius, 2);

	const float discriminant = powf(b, 2) - (4.0f * a * c);
	if (discriminant < epsilon)
		return false;

	const float b0 = (-b - sqrt(discriminant)) / (2.0f * a);
	const float b1 = (-b + sqrt(discriminant)) / (2.0f * a);

	const float t0 = std::max(i1.t0, b0);
	const float t1 = std::min(i1.t1, b1);

	if (t0 < epsilon && t1 < epsilon)
		return false;

	if (t0 > t1)
		return false;

	if (t1 < epsilon)
		return false;

	const float t = GetSmallestPositiveValue(t0, t1);
	intersection.object = this;
	intersection.t = t;
	intersection.point = ray.eval(t);
	intersection.normal = normalize(GetNormal(t, t0, t1, i1, newRay, inverse_R));

	return true;
}

vec3 Cylinder::GetNormal(float t, float t0, float t1, Interval interval, Ray ray, mat3 inverse_R)
{
	if (fabs(t - t0) < epsilon)
	{
		if (fabs(t0 - interval.t0) < epsilon)
			return vec3(0, 0, 1);

		vec3 p = ray.eval(t0);
		return inverse_R * vec3(p.x, p.y, 0.f);
	}
	else if (fabs(t - t1) < epsilon)
	{
		if (fabs(t1 - interval.t1) < epsilon)
			return vec3(0, 0, -1);

		vec3 p = ray.eval(t1);
		return -inverse_R * vec3(p.x, p.y, 0.f);
	}

	return vec3(0);
}

IBL::IBL(const vec3 center_, const float radius_, Material* mat) : Shape(mat)
{
	material = mat;
	base = center_;
	radius = radius_;

	HDRResult result;
	HDRReader::load("background.hdr", result);

	//angle = 0.0f;
	image = result.cols;
	width = result.width;
	height = result.height;
	pBuffer = new float[width * (height + 1)];
	pUDist = &pBuffer[width * height];
	float* pSinTheta = new float[height];
	float angleFrac = PI / (float)height;
	angle = angleFrac;
	float theta = angleFrac * 0.5f;
	for (int i = 0; i < height; i++, theta += angleFrac)
		pSinTheta[i] = sin(theta);

	for (int i = 0, m = 0; i < width; i++, m += height)
	{
		float* pVDist = &pBuffer[m];
		int k = i * 3;
		pVDist[0] = 0.2126f * image[k + 0] + 0.7152f * image[k + 1] + 0.0722f * image[k + 2];
		pVDist[0] *= pSinTheta[0];

		for (int j = 1, k = (width + i) * 3; j < height; j++, k += width * 3)
		{
			float lum = 0.2126f * image[k + 0] + 0.7152f * image[k + 1] + 0.0722f * image[k + 2];
			pVDist[j] = pVDist[j - 1] + lum * pSinTheta[j];
		}

		if (i == 0)
			pUDist[i] = pVDist[height - 1];
		else
			pUDist[i] = pUDist[i - 1] + pVDist[height - 1];
	}
}

void IBL::CreateBV()
{
	min = base - vec3(radius);
	max = base + vec3(radius);
}

bool IBL::intersect(Ray ray, Intersection& intersection)
{
	const vec3 Q = (ray.Q - base);

	const float a = dot(ray.D, ray.D);
	const float b = 2.0f * dot(Q, ray.D);
	const float c = dot(Q, Q) - powf(radius, 2);

	float discriminant = powf(b, 2) - (4.0f * a * c);

	if (discriminant < epsilon)
		return false;

	const float t0 = (-b - sqrt(discriminant)) / (2.0f * a);
	const float t1 = (-b + sqrt(discriminant)) / (2.0f * a);

	if (t0 < epsilon && t1 < epsilon)
		return false;

	if (t1 < epsilon)
		return false;

	intersection.object = this;
	intersection.t = GetSmallestPositiveValue(t0, t1);
	intersection.point = ray.eval(intersection.t);
	intersection.normal = -ray.D;

	return true;
}

Intersection IBL::SampleAsLight()
{
	Intersection B;
	double u = myrandomd(RNGen);
	double v = myrandomd(RNGen);
	float maxUVal = pUDist[width - 1];
	float* pUPos = std::lower_bound(pUDist, pUDist + width, u * maxUVal);

	int iu = pUPos - pUDist;
	float* pVDist = &pBuffer[height * iu];
	float* pVPos = std::lower_bound(pVDist, pVDist + height, v * pVDist[height - 1]);

	int iv = pVPos - pVDist;

	double phi = angle - 2.0 * PI * iu / width;
	double theta = PI * iv / height;

	B.normal = glm::normalize(vec3(sin(theta) * cos(phi), sin(theta) * sin(phi), cos(theta)));
	B.point = B.normal * radius;
	B.object = this;

	return B;
}
