#pragma once
#include "geom.h"
#include "raytrace.h"
#include "Auxiliary.h"

class Ray;
class Intersection;
class MeshData;
class Material;
class VertexData;
class Interval;

class Shape
{
public:
	Shape(Material* material);
	virtual ~Shape() = default;
	virtual bool intersect(Ray, Intersection&) = 0;
	float GetSmallestPositiveValue(float t0, float t1);

	// object's light method
	bool IsLight() { return material->isLight(); }
	vec3 EvalRadiance() { return material->Kd; }
	float PdfLight();

	// object's brdf method
	vec3 SampleBRDF(vec3 omegaO, vec3 normal);
	vec3 EvalScattering(vec3 omegaO, vec3 normal, vec3 omegaI, float t);
	float PdfBRDF(vec3 omegaO, vec3 normal, vec3 omegaI);

	Material* material = nullptr;
	vec3 base;
	vec3 min;
	vec3 max;
	float p_d = std::numeric_limits<float>::infinity();
	float p_r = std::numeric_limits<float>::infinity();
	float p_t = std::numeric_limits<float>::infinity();
};

class Sphere : public Shape
{
public:
	Sphere(const vec3, const float, Material*);

	bool intersect(Ray, Intersection&) override;
	Intersection SampleSphere();

	float radius;
};

class Box : public Shape
{
public:
	Box(const vec3 base_, const vec3 diagonal_, Material* mat);

	bool intersect(Ray, Intersection&) override;

	vec3 diagonal;

private:
	vec3 GetNormal(float, float, float, Interval, Interval, Interval);
};

class Triangle : public Shape
{
public:
	Triangle(const vec3, const vec3, const vec3, const vec3, const vec3, const vec3, Material*);

	bool intersect(Ray, Intersection&) override;

	vec3 v0, v1, v2;
	vec3 n0, n1, n2;
};

class Cylinder : public Shape
{
public:
	Cylinder(const vec3, const vec3, const float, Material*);

	bool intersect(Ray, Intersection&) override;

	vec3 axis;
	float radius;

private:
	vec3 GetNormal(float t, float t0, float t1, Interval interval, Ray ray, mat3 R);
};