#pragma once
#include "geom.h"

class Ray
{
public:
	Ray(vec3 origin_, vec3 direction_)
	{
		Q = origin_;
		D = direction_;
	}

	vec3 eval(float t)
	{
		return Q + (t * D);
	}

	vec3 Q;
	vec3 D;
};