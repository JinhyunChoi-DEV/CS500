#pragma once
#include <glm/glm.hpp>

inline float epsilon = 0.0001f;

// A good quality *thread-safe* Mersenne Twister random number generator.
#include <random>
inline std::random_device device;
inline std::mt19937_64 RNGen(device());
inline std::uniform_real_distribution<float> myrandom(0.0f, 1.0f);

inline int GetRandomIndex(int size)
{
	std::uniform_int_distribution<int> distribution(0, size - 1);
	return distribution(RNGen);
}