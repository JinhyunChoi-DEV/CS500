#pragma once
#include "geom.h"
#include <vector>

class Shape;
class Camera;
class Ray;
struct MeshData;
class Material;

class StaticRayTrace
{
public:
	StaticRayTrace();

	int width, height;
	Camera* camera;

	vec3 lightAmbientColor;

	std::vector<Shape*> shapes;
	std::vector<Material*> materials;
	std::vector<Material*> lights;

	void Test(Shape* shape);
	void TraceRay(Ray ray);
};

