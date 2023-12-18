#pragma once
#include <glm/glm.hpp>
#include "Auxiliary.h"
#include "raytrace.h"

vec3 AttenuationColor(vec3 omegaO, vec3 normal, vec3 Kt, float t);

inline float Diffuse_Probability(vec3 omegaI, vec3 normal)
{
	return abs(dot(omegaI, normal)) / PI;
}

inline float Reflection_Probability(vec3 omegaO, vec3 normal, vec3 omegaI, Material* material)
{
	const vec3 m = normalize(omegaO + omegaI);
	float D = D_Factor(m, normal, material);

	return D * abs(dot(m, normal)) * (1.0f / (4.0f * abs(dot(omegaI, m))));
}

inline float Transmission_Probability(vec3 omegaO, vec3 normal, vec3 omegaI, Material* material, float etaO, float etaI, float eta)
{
	vec3 m = -normalize(etaO * omegaI + etaI * omegaO);

	float omegaDotm = dot(omegaO, m);
	const float r = 1.0f - powf(eta, 2) * (1.0f - powf(omegaDotm, 2));

	if (r < epsilon)
		return Reflection_Probability(omegaO, normal, omegaI, material);

	float D = D_Factor(m, normal, material);
	float numerator = powf(etaO, 2) * abs(dot(omegaI, m));
	float denominator = powf((etaO * dot(omegaI, m) + etaI * dot(omegaO, m)), 2);

	return D * abs(dot(m, normal)) * (numerator / denominator);
}

inline vec3 Diffuse_EvalScattering(Material* material)
{
	return material->Kd / PI;
}

inline vec3 Reflection_EvalScattering(vec3 omegaO, vec3 normal, vec3 omegaI, Material* material)
{
	const vec3 m = normalize(omegaO + omegaI);
	const float D = D_Factor(m, normal, material);
	const float G = G_Factor(omegaI, omegaO, m, normal, material);
	const vec3 F = F_Factor(dot(omegaI, m), material);

	const vec3 numerator = D * G * F;
	const float denominator = 4.0f * abs(dot(omegaI, normal)) * abs(dot(omegaO, normal));

	return numerator / denominator;
}

inline vec3 Transmission_EvalScattering(vec3 omegaO, vec3 normal, vec3 omegaI, Material* material, float etaI, float etaO, float eta, float t)
{
	vec3 m = -normalize(etaO * omegaI + etaI * omegaO);
	float omegaDotm = dot(omegaO, m);
	const float r = 1.0f - powf(eta, 2) * (1.0f - powf(omegaDotm, 2));

	vec3 attenuation = AttenuationColor(omegaO, normal, material->Kt, t);
	if (r < epsilon)
		return attenuation * Reflection_EvalScattering(omegaO, normal, omegaI, material);

	const float D = D_Factor(m, normal, material);
	const float G = G_Factor(omegaI, omegaO, m, normal, material);
	const vec3 F = F_Factor(dot(omegaI, m), material);

	const vec3 numerator_left = D * G * (1.0f - F);
	const float denominator_left = abs(dot(omegaI, normal)) * abs(dot(omegaO, normal));

	float numerator_right = abs(dot(omegaI, m)) * abs(dot(omegaO, m)) * powf(etaO, 2);
	float denominator_right = powf(etaO * dot(omegaI, m) + etaI * dot(omegaO, m), 2);

	return attenuation * (numerator_left / denominator_left) * (numerator_right / denominator_right);
}

inline vec3 AttenuationColor(vec3 omegaO, vec3 normal, vec3 Kt, float t)
{
	float omegaDotN = dot(omegaO, normal);

	vec3 result(1.0f);
	if (omegaDotN < 0)
	{
		result.x = exp(t * log(Kt.x));
		result.y = exp(t * log(Kt.y));
		result.z = exp(t * log(Kt.z));
	}

	return result;
}