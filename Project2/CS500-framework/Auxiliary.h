#pragma once
#include "geom.h"
#include "Helper.h"
#include "Intersection.h"

inline vec3 SampleLobe(vec3 A, float theta, float phi)
{
	float s = sqrtf(1.0f - powf(theta, 2));
	vec3 K = vec3(s * cos(phi), s * sin(phi), theta);	// vector centered around z-axis

	if (abs(A.z - 1.0f) < epsilon)	// A=z no rotation
		return K;
	if (abs(A.z + 1.0f) < epsilon)	// A=-z so rotate 180 around X axis
		return vec3(K.x, -K.y, -K.z);

	vec3 B = normalize(vec3(-A.y, A.x, 0));
	vec3 C = cross(A, B);
	return K.x * B + K.y * C + K.z * A;
}

inline float GeometryFactor(Intersection A, Intersection B)
{
	vec3 D = A.point - B.point;
	return abs(dot(A.normal, D) * dot(B.normal, D) / powf(dot(D, D), 2));
}