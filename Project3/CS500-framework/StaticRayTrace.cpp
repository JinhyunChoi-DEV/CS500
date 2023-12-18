#include <iostream>
#include "Camera.h"
#include "Shape.h"
#include "raytrace.h"
#include "Ray.h"
#include "StaticRayTrace.h"
#include "Helper.h"
#include "acceleration.h"
#include "Auxiliary.h"

StaticRayTrace::StaticRayTrace()
{
	camera = new Camera();
}

void StaticRayTrace::AddShape(Shape* shape)
{
	if (shape == nullptr)
	{
		std::cout << "ERROR::AddShape - Try to add null data of shape" << std::endl;
		return;
	}

	if (shape->material->isLight())
		lights.push_back(shape);

	shapes.push_back(shape);
	materials.push_back(shape->material);
}

void StaticRayTrace::AddModel(MeshData* mesh, Material* mat)
{
	int size = static_cast<int>(mesh->triangles.size());

	for (int i = 0; i < size; ++i)
	{
		auto triangle = mesh->triangles[i];
		VertexData v1 = mesh->vertices[triangle.x];
		VertexData v2 = mesh->vertices[triangle.y];
		VertexData v3 = mesh->vertices[triangle.z];

		auto shape = new Triangle(v1.pnt, v2.pnt, v3.pnt, v1.nrm, v2.nrm, v3.nrm, mat);

		if (mat->isLight())
			lights.push_back(shape);

		shapes.push_back(shape);
		materials.push_back(mat);
	}
}

vec3 StaticRayTrace::TraceRay(Ray ray)
{
	vec3 C = vec3(0);
	vec3 W = vec3(1);

	Intersection P = bvh->intersect(ray);
	vec3 N = P.normal;

	if (P.object == nullptr)
		return C;

	if (P.object->IsLight())
		return P.object->EvalRadiance();

	vec3 omegaO = -ray.D;
	while (myrandom(RNGen) <= RussianRoulette)
	{
		//Explicit light connect
		Intersection L = SampleLight();
		float p = L.object->PdfLight() / GeometryFactor(P, L);
		vec3 omegaI = normalize(L.point - P.point);
		Ray I = Ray(P.point, omegaI);
		Intersection lightIntersection = bvh->intersect(I);
		if (p > epsilon && lightIntersection.object != nullptr && (lightIntersection.point == L.point))
		{
			vec3 f = P.object->EvalScattering(omegaO, N, omegaI);
			C += 0.5f * W * f / p * L.object->EvalRadiance();
		}

		// Extend Path
		omegaI = P.object->SampleBRDF(omegaO, N);

		Intersection Q = bvh->intersect(Ray(P.point, omegaI));
		if (Q.object == nullptr)
			break;

		vec3 f = P.object->EvalScattering(omegaO, N, omegaI);
		p = P.object->PdfBRDF(omegaO, N, omegaI) * RussianRoulette;

		if (p < epsilon)
			break;
		W *= f / p;

		if (Q.object->IsLight())
		{
			C += 0.5f * W * Q.object->EvalRadiance();
			break;
		}

		P = Q;
		N = P.normal;
		omegaO = -omegaI;
	}

	return C;
}

Intersection StaticRayTrace::SampleLight()
{
	/*const int size = static_cast<int>(lights.size());
	int randomIndex = GetRandomIndex(size);
	Sphere* light = static_cast<Sphere*>(lights[randomIndex]);*/
	Sphere* light = static_cast<Sphere*>(lights[0]);

	return light->SampleSphere();
}
