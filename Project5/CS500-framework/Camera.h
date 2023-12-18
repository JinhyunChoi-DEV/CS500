#pragma once
#include "geom.h"

class Camera
{
public:
	Camera();

	void Set(const vec3& eye_, const quat& orient_, const float& ratio_, const int& width, const int& height);

	vec3 eye;
	vec3 X, Y, Z;
	quat orient;
	float r_x;
	float r_y;
};