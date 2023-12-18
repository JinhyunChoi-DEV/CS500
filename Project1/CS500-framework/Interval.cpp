#include "Interval.h"
#include "Ray.h"

constexpr float epsilon = std::numeric_limits<float>::epsilon();

Interval::Interval()
{
	t0 = 0;
	t1 = std::numeric_limits<float>::infinity();

	n0 = vec3(0);
	n1 = vec3(0);
}

Interval::Interval(float t0_, float t1_)
{
	t0 = t0_;
	t1 = t1_;

	n0 = vec3(0);
	n1 = vec3(0);
}

Interval::Interval(float t0_, float t1_, vec3 n0_, vec3 n1_)
{
	t0 = t0_;
	t1 = t1_;

	n0 = n0_;
	n1 = n1_;
}

void Interval::Empty()
{
	t0 = 0;
	t1 = -1;
}

Interval Interval::intersect()
{
	return Interval();
}

Interval Interval::intersect(Ray ray, Slab slab)
{
	Interval result;

	const float NdotQ = dot(slab.normal, ray.Origin());
	const float NdotD = dot(slab.normal, ray.Direction());

	if (!(fabs(NdotD - 0.0f) <= epsilon))
	{
		const float t0 = -(slab.d0 + NdotQ) / NdotD;
		const float t1 = -(slab.d1 + NdotQ) / NdotD;

		result.t0 = t0;
		result.t1 = t1;
		if (result.t1 < result.t0)
			std::swap(result.t0, result.t1);
	}
	else
	{
		const float s0 = NdotQ + slab.d0;
		const float s1 = NdotQ + slab.d1;

		if (std::signbit(s0) == std::signbit(s1))
		{
			result.t0 = std::numeric_limits<float>::infinity();
			result.t1 = 0;
		}
		else
		{
			result.t0 = 0.0f;
			result.t1 = std::numeric_limits<float>::infinity();
		}
	}

	return result;
}
