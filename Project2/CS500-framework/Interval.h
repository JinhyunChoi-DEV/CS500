#pragma once
#include "geom.h"

struct Slab
{
	vec3 normal;
	float d0;
	float d1;
};

class Ray;

class Interval
{
public:
	Interval();
	Interval(float, float);

	void intersect(Ray, Slab);

	float t0 = 0;
	float t1 = std::numeric_limits<float>::infinity();
	vec3 n0;
	vec3 n1;
};