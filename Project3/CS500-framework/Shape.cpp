#include "Shape.h"

#include "Auxiliary.h"
#include "Intersection.h"
#include "Interval.h"
#include "Ray.h"
#include "raytrace.h"

float Shape::GetSmallestPositiveValue(float t0, float t1)
{
	if (t0 < epsilon)
		return t1;

	if (t0 < t1)
		return t0;

	return t1;
}

float Shape::PdfLight()
{
	Sphere* sphere = static_cast<Sphere*>(this);

	const float areaOfLightSphere = 4 * PI * powf(sphere->radius, 2);
	return 1.0f / areaOfLightSphere;
}

vec3 Shape::SampleBRDF(vec3 omegaO, vec3 normal)
{
	const float s = length(material->Kd) + length(material->Ks);
	p_d = length(material->Kd) / s;
	p_r = length(material->Ks) / s;
	const float chooseFactor = myrandom(RNGen);

	const float e1 = myrandom(RNGen);
	const float e2 = myrandom(RNGen);

	if (chooseFactor < p_d)
	{
		float theta = sqrtf(e1);
		float phi = 2.0f * PI * e2;
		return SampleLobe(normal, theta, phi);
	}

	float theta = GetDistributionCos(material, e1);
	float phi = 2.0f * PI * e2;
	vec3 m = SampleLobe(normal, theta, phi);
	return normalize(2.0f * abs(dot(omegaO, m)) * m - omegaO);
}

vec3 Shape::EvalScattering(vec3 omegaO, vec3 normal, vec3 omegaI)
{
	const vec3 m = normalize(omegaO + omegaI);
	const float D = D_Factor(m, normal, material);
	const float G = G_Factor(omegaI, omegaO, m, normal, material);
	const vec3 F = F_Factor(dot(omegaI, m), material);

	const vec3 numerator = D * G * F;
	const float denominator = 4.0f * abs(dot(omegaI, normal)) * abs(dot(omegaO, normal));

	const vec3 E_d = material->Kd / PI;
	const vec3 E_r = numerator / denominator;
	return abs(dot(normal, omegaI)) * (E_d + E_r);
}

float Shape::PdfBRDF(vec3 omegaO, vec3 normal, vec3 omegaI)
{
	if (std::isinf(p_d) || std::isinf(p_r))
	{
		std::cout << "ERROR: Before calling PdfBRDF, the p_d and p_r should be initialized" << std::endl;
		return 0.0f;
	}

	const vec3 m = normalize(omegaO + omegaI);
	const float D = D_Factor(m, normal, material);

	const float P_d = abs(dot(omegaI, normal)) / PI;
	const float P_r = D * abs(dot(m, normal)) * (1.0f / (4.0f * abs(dot(omegaI, m))));

	return p_d * P_d + p_r * P_r;
}

Sphere::Sphere(const vec3 center_, const float r, Material* mat)
{
	radius = r;

	material = mat;
	base = center_;

	min = base - vec3(radius);
	max = base + vec3(radius);
}

bool Sphere::intersect(Ray ray, Intersection& intersection)
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
	intersection.normal = normalize(intersection.point - base);

	return true;
}

Intersection Sphere::SampleSphere()
{
	Intersection result;

	float e1 = myrandom(RNGen);
	float e2 = myrandom(RNGen);

	float z = 2.0f * e1 - 1;
	float r = sqrtf(1 - powf(z, 2));
	float a = 2 * PI * e2;

	result.normal = vec3(r * cos(a), r * sin(a), z);
	result.point = base + (radius * result.normal);
	result.object = this;

	return result;
}

Box::Box(const vec3 base_, const vec3 diagonal_, Material* mat)
{
	base = base_;
	diagonal = diagonal_;

	material = mat;

	min = base_;
	max = base_ + diagonal_;
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

Triangle::Triangle(const vec3 v0_, const vec3 v1_, const vec3 v2_, const vec3 n0_, const vec3 n1_, const vec3 n2_, Material* mat)
{
	v0 = v0_;
	v1 = v1_;
	v2 = v2_;

	n0 = n0_;
	n1 = n1_;
	n2 = n2_;

	base = (v0 + v1 + v2) / 3.0f;
	material = mat;

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

Cylinder::Cylinder(const vec3 base_, const vec3 axis_, const float r, Material* mat)
{
	axis = axis_;
	radius = r;

	material = mat;
	base = base_;

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
	vec3 v = vec3(myrandom(RNGen), myrandom(RNGen), myrandom(RNGen));
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