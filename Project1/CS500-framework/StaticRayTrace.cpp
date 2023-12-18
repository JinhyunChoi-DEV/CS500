#include <iostream>
#include "Camera.h"
#include "Shape.h"
#include "raytrace.h"
#include "Ray.h"
#include "StaticRayTrace.h"

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

void StaticRayTrace::TraceRay(Ray ray) {}
