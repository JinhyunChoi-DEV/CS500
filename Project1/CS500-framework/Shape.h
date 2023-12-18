#pragma once
#include "geom.h"

class Ray;
class Intersection;
class MeshData;
class Material;
class VertexData;
class Interval;

class Shape
{
public:
	virtual ~Shape() = default;
	virtual bool intersect(Ray, Intersection&) = 0;
	float GetSmallestPositiveValue(float t0, float t1);

	Material* material = nullptr;
	vec3 base;
	vec3 min;
	vec3 max;
};

class Sphere : public Shape
{
public:
	Sphere(const vec3, const float, Material*);

	bool intersect(Ray, Intersection&) override;

	float radius;
};

class Box : public Shape
{
public:
	Box(const vec3, const vec3, Material*);

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