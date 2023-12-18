#pragma once
#include "geom.h"
#include "Helper.h"
#include "Intersection.h"
#include "StaticRayTrace.h"

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
	vec3 D = normalize(A.point - B.point);
	return abs(dot(A.normal, D) * dot(B.normal, D) / powf(dot(D, D), 2));
}

inline float CharacteristicFactor(float d)
{
	if (d > 0)
		return 1.0f;

	return 0.0f;
}

inline float GetDistributionCos(Material* mat, float theta)
{
	if (type == DistributionType::Phong)
	{
		return powf(theta, 1.0f / (mat->alpha_phong + 1));
	}

	if (type == DistributionType::GGX)
	{
		const float numerator = mat->alpha_other * sqrtf(theta);
		const float denominator = sqrtf(1 - theta);
		const float aTan = atanf(numerator / denominator);
		return cosf(aTan);
	}

	if (type == DistributionType::Beckman)
	{
		const float factor = sqrtf(-powf(mat->alpha_other, 2) * log(1 - theta));
		const float aTan = atanf(factor);
		return cosf(aTan);
	}

	return 0.0f;
}

inline vec3 F_Factor(float d, Material* mat)
{
	return mat->Ks + (1.0f - mat->Ks) * powf(1.0f - abs(d), 5);
}

inline float D_Factor(vec3 m, vec3 normal, Material* mat)
{
	float mDotN = dot(m, normal);
	float tanTheta = sqrtf(1.0f - powf(mDotN, 2)) / mDotN;
	float cFactor = CharacteristicFactor(mDotN);

	if (type == DistributionType::Phong)
	{
		return cFactor * (mat->alpha_phong + 2.0f) * pow(mDotN, mat->alpha_phong) / (2.0f * PI);
	}

	if (type == DistributionType::GGX)
	{
		float square_alpha = powf(mat->alpha_other, 2);
		float square_tan_m = powf(tanTheta, 2);

		float left_denom = powf(mDotN, 4);
		float right_denom = powf(square_alpha + square_tan_m, 2);
		float f1 = square_alpha / (PI * left_denom * right_denom);

		return cFactor * f1;
	}

	if (type == DistributionType::Beckman)
	{
		float f1 = 1.0f / (PI * powf(mat->alpha_other, 2) * powf(mDotN, 4));
		float f2 = std::expf(-powf(tanTheta, 2) / powf(mat->alpha_other, 2));

		return cFactor * f1 * f2;
	}

	return 0.0f;
}

inline float GetG(vec3 v, vec3 m, vec3 normal, Material* mat)
{
	float vDotN = dot(v, normal);
	if (vDotN > 1.0f)
		return 1.0f;

	float tanTheta = sqrtf(1.0f - powf(vDotN, 2)) / vDotN;
	if (fabs(tanTheta - 0.0f) < epsilon)
		return 1.0f;

	float vDotm = dot(v, m);
	float cFactor = CharacteristicFactor(vDotm / vDotN);

	if (type == DistributionType::Phong)
	{
		float a = sqrtf(mat->alpha_phong / 2 + 1) / tanTheta;

		if (a >= 1.6f)
			return cFactor;

		const float numerator = 3.535f * a + 2.181f * powf(a, 2);
		const float denominator = 1.0f + 2.276f * a + 2.577f * powf(a, 2);
		return cFactor * (numerator / denominator);
	}

	if (type == DistributionType::GGX)
	{
		float square_alpha = powf(mat->alpha_other, 2);
		float square_theta = powf(tanTheta, 2);
		float f1 = 2.0f / (1 + sqrtf(1 + square_alpha * square_theta));

		return cFactor * f1;
	}

	if (type == DistributionType::Beckman)
	{
		float a = 1 / (mat->alpha_other * tanTheta);

		if (a >= 1.6f)
			return cFactor;

		const float numerator = 3.535f * a + 2.181f * powf(a, 2);
		const float denominator = 1.0f + 2.276f * a + 2.577f * powf(a, 2);
		return cFactor * (numerator / denominator);
	}

	return 0.0f;
}


inline float G_Factor(vec3 omegaI, vec3 omegaO, vec3 m, vec3 normal, Material* mat)
{
	return GetG(omegaI, m, normal, mat) * GetG(omegaO, m, normal, mat);
}
