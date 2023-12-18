#include <random>
#include "Shape.h"
#include "Intersection.h"
#include "Interval.h"
#include "Ray.h"
#include "raytrace.h"

// A good quality *thread-safe* Mersenne Twister random number generator.
#include <random>
std::random_device device;
std::mt19937_64 RNGen(device());
std::uniform_real_distribution<> myrandom(-1.0, 1.0);
// Call myrandom(RNGen) to get a uniformly distributed random number in [0,1].

const float epsilon = std::numeric_limits<float>::epsilon();

float Shape::GetSmallestPositiveValue(float t0, float t1)
{
	if ((t0 > 0 && t1 > 0) && fabs(t0 - t1) <= epsilon)
		return t0;

	if (t0 > 0 && t1 > 0)
		return std::min(t0, t1);

	if (t0 > 0 && t1 < 0)
		return t0;

	return t1;
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
	const vec3 Q = (ray.Origin() - base);

	const float a = dot(ray.Direction(), ray.Direction());
	const float b = 2.0f * dot(Q, ray.Direction());
	const float c = dot(Q, Q) - (radius * radius);

	const float b_square = b * b;
	float discriminant = b_square - (4.0f * a * c);

	if (discriminant < 0)
		return false;

	const float t0 = (-b - sqrt(discriminant)) / (2.0f * a);
	const float t1 = (-b + sqrt(discriminant)) / (2.0f * a);

	if (t0 < 0 && t1 < 0)
		return false;

	intersection.object = this;
	intersection.t = GetSmallestPositiveValue(t0, t1);
	intersection.point = ray.eval(intersection.t);
	intersection.normal = normalize(intersection.point - base);

	return true;
}

Box::Box(const vec3 corner_, const vec3 diagonal_, Material* mat)
{
	diagonal = diagonal_;

	material = mat;
	base = corner_;

	min = corner_;
	max = corner_ + diagonal_;
}

bool Box::intersect(Ray ray, Intersection& intersection)
{
	Slab slab1{ vec3(1,0,0), -base.x, -base.x - diagonal.x };
	Slab slab2{ vec3(0,1,0), -base.y, -base.y - diagonal.y };
	Slab slab3{ vec3(0,0,1), -base.z, -base.z - diagonal.z };

	Interval startI(0, std::numeric_limits<float>::infinity());
	Interval i1 = Interval::intersect(ray, slab1);
	Interval i2 = Interval::intersect(ray, slab2);
	Interval i3 = Interval::intersect(ray, slab3);

	const float t0 = std::max(std::max(std::max(startI.t0, i1.t0), i2.t0), i3.t0);
	const float t1 = std::min(std::min(std::min(startI.t1, i1.t1), i2.t1), i3.t1);

	if (t0 > t1)
		return false;

	if (t1 < 0)
		return false;

	if (std::_Is_inf(t1))
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

	if (((t0 > 0 && t1 > 0) && t0 < t1) || (t0 > 0 && t1 < 0))
	{
		if (fabs(t - i1.t0) <= epsilon)
			return n0;
		if (fabs(t - i2.t0) <= epsilon)
			return n1;
		if (fabs(t - i3.t0) <= epsilon)
			return n2;
	}

	if (((t0 > 0 && t1 > 0) && t1 < t0) || (t1 > 0 && t0 < 0))
	{
		if (fabs(t - i1.t1) <= epsilon)
			return -n0;
		if (fabs(t - i2.t1) <= epsilon)
			return -n1;
		if (fabs(t - i3.t1) <= epsilon)
			return -n2;
	}

	if ((t0 > 0 && t1 > 0) && (fabs(t0 - t1) <= epsilon))
	{
		if (fabs(t - i1.t0) <= epsilon)
			return n0;
		if (fabs(t - i2.t0) <= epsilon)
			return n1;
		if (fabs(t - i3.t0) <= epsilon)
			return n2;
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
	vec3 p = cross(ray.Direction(), e2);
	float d = dot(p, e1);
	if (fabs(d - 0.0f) < epsilon)
		return false;

	vec3 s = ray.Origin() - v0;
	float u = dot(p, s) / d;
	if (u < 0.0f || u > 1.0f)
		return false;

	vec3 q = cross(s, e1);
	float v = dot(ray.Direction(), q) / d;
	if (v < 0.0f || (u + v) > 1.0f)
		return false;

	float t = dot(e2, q) / d;
	if (t < 0.0f)
		return false;

	intersection.object = this;
	intersection.t = t;
	intersection.point = ray.eval(intersection.t);
	intersection.normal = (1.0f - u - v) * n0 + u * n1 + v * n2;

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
	//vec3 v(myrandom(RNGen), myrandom(RNGen), myrandom(RNGen));
	vec3 rotationV = vec3(0.f, 0.f, 1.f);
	if ((glm::abs(glm::dot(rotationV, normalize(axis))) == 1.f))
	{
		rotationV = vec3(1.f, 0.f, 0.f);
	}

	vec3 A = normalize(axis);
	vec3 B = normalize(cross(rotationV, A));
	vec3 C = normalize(cross(A, B));

	mat3 inverse_R(B, C, A);
	mat3 R = glm::transpose(inverse_R);

	vec3 start = R * (ray.Origin() - base);
	vec3 dir = normalize(R * ray.Direction());
	Ray newRay(start, dir);

	// first interval
	Slab slab{ vec3(0, 0, 1), 0, -length(axis) };
	Interval i1 = Interval::intersect(newRay, slab);

	// second interval
	const float a = (newRay.Direction().x * newRay.Direction().x) + (newRay.Direction().y * newRay.Direction().y);
	const float b = 2.0f * ((newRay.Direction().x * newRay.Origin().x) + (newRay.Direction().y * newRay.Origin().y));
	const float c = (newRay.Origin().x * newRay.Origin().x) + (newRay.Origin().y * newRay.Origin().y) - (radius * radius);

	const float discriminant = (b * b) - (4.0f * a * c);
	if (discriminant < 0)
		return false;

	const float b0 = (-b - sqrt(discriminant)) / (2.0f * a);
	const float b1 = (-b + sqrt(discriminant)) / (2.0f * a);
	Interval i2(b0, b1);

	const float t0 = std::max(i1.t0, i2.t0);
	const float t1 = std::min(i1.t1, i2.t1);

	if (t0 > t1)
		return false;

	if (t0 < 0 && t1 < 0)
		return false;

	const float t = GetSmallestPositiveValue(t0, t1);

	intersection.object = this;
	intersection.t = t;
	intersection.point = ray.eval(t);
	intersection.normal = GetNormal(t, t0, t1, i1, newRay, inverse_R);

	return true;
}

vec3 Cylinder::GetNormal(float t, float t0, float t1, Interval interval, Ray ray, mat3 inverse_R)
{
	vec3 p = ray.eval(t);
	vec3 zAxis = vec3(0, 0, 1);

	if (ray.Origin().z < 0)
		zAxis = -zAxis;

	if ((t0 > 0 && t1 > 0) && t0 < t1 || (t0 > 0 && t1 < 0))
	{
		if (fabs(t - interval.t0) <= epsilon)
			return inverse_R * zAxis;
		else
			return inverse_R * vec3(p.x, p.y, 0);
	}

	if ((t0 > 0 && t1 > 0) && t1 < t0 || (t1 > 0 && t0 < 0))
	{
		if (fabs(t - interval.t1) <= epsilon)
			return inverse_R * zAxis;
		else
			return inverse_R * vec3(p.x, p.y, 0);
	}

	if ((t0 > 0 && t1 > 0) && (fabs(t0 - t1) <= epsilon))
	{
		if (fabs(t - interval.t0) <= epsilon)
			return inverse_R * zAxis;
		else
			return inverse_R * vec3(p.x, p.y, 0);
	}

	return vec3(0);
}
