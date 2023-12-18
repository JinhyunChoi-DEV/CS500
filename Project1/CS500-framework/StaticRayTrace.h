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
	Camera* camera;

	vec3 lightAmbientColor;

	std::vector<Shape*> shapes;
	std::vector<Shape*> modelShapes;
	std::vector<Material*> materials;
	std::vector<Shape*> lights;

	void AddShape(Shape* shape);
	void AddModel(MeshData* shape, Material* mat);
	void TraceRay(Ray ray);
};

