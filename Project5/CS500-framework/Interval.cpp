#include "Interval.h"
#include "Ray.h"
#include "Helper.h"

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

void Interval::intersect(Ray ray, Slab slab)
{
	const float NdotQ = dot(slab.normal, ray.Q);
	const float NdotD = dot(slab.normal, ray.D);

	if (!(fabs(NdotD - 0.0f) <= epsilon))
	{
		const float t_t0 = -(slab.d0 + NdotQ) / NdotD;
		const float t_t1 = -(slab.d1 + NdotQ) / NdotD;

		t0 = t_t0;
		t1 = t_t1;
		if (t0 > t1)
			std::swap(t0, t1);
	}
	else
	{
		const float s0 = NdotQ + slab.d0;
		const float s1 = NdotQ + slab.d1;

		if (s0 * s1 < 0)
		{
			t0 = 0.0f;
			t1 = std::numeric_limits<float>::infinity();
		}
		else
		{
			t0 = 1;
			t1 = 0;
		}
	}
}
