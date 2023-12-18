#include <iostream>
#include "Camera.h"
#include "Shape.h"
#include "raytrace.h"
#include "Ray.h"
#include "StaticRayTrace.h"

StaticRayTrace::StaticRayTrace()
{
	width = 0;
	height = 0;
	camera = new Camera();
}

void StaticRayTrace::Test(Shape* shape)
{
	if (shape == nullptr || shape->mesh == nullptr)
	{
		std::cout << "ERROR::Test - Try to add null data of shape" << std::endl;
		return;
	}

	if (shape->material->isLight())
		lights.push_back(shape->material);

	shapes.push_back(shape);
	materials.push_back(shape->material);
}

void StaticRayTrace::TraceRay(Ray ray) {}
