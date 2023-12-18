#pragma once

class Shape;

class Intersection
{
public:
	Intersection() = default;
	float distance() const { return t; }

	Shape* object = nullptr;
	float t = std::numeric_limits<float>::infinity();
	vec3 point = vec3(0);
	vec3 normal = vec3(0);
};