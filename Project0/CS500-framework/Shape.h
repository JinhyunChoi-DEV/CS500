#pragma once
#include "geom.h"

class Ray;
class Intersection;
class MeshData;
class Material;

class Shape
{
public:
	virtual ~Shape() = default;
	virtual void intersect(Ray, Intersection&) = 0;

	MeshData* mesh = nullptr;
	Material* material = nullptr;
	vec3 center;
};

class Sphere : public Shape
{
public:
	Sphere(const vec3, const float, Material*);

	void intersect(Ray, Intersection&) override;
};

class Box : public Shape
{
public:
	Box(const vec3, const vec3, Material*);

	void intersect(Ray, Intersection&) override;

private:
};

class Cylinder : public Shape
{
public:
	Cylinder(const vec3, const vec3, const float r, Material*);

	void intersect(Ray, Intersection&) override;
};

class Triangle : public Shape
{
public:
	Triangle(MeshData*);

	void intersect(Ray, Intersection&) override;
};