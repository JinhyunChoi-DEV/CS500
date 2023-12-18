#pragma once

class Shape;

class Intersection
{
public:
	Intersection() = default;

	Shape* object = nullptr;
	float t = 0.0f;
	vec3 point = vec3(0);
	vec3 normal = vec3(0);

};