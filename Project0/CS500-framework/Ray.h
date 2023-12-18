#pragma once
#include "geom.h"

class Ray
{
public:
	Ray(vec3 origin_, vec3 direction_)
	{
		origin = origin_;
		direction = direction_;
	}

	vec3 eval(float t)
	{
		return origin + (t * direction);
	}

	vec3 Origin() const { return origin; }
	vec3 Direction() const { return direction; }

private:
	vec3 origin;
	vec3 direction;
};