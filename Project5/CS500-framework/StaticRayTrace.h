#pragma once
#include "geom.h"
#include <vector>
#include "Intersection.h"

class Shape;
class Camera;
class Ray;
struct MeshData;
class Material;
class IBL;

enum class DistributionType
{
	Phong, GGX, Beckman
};

inline static DistributionType type = DistributionType::Beckman;

class StaticRayTrace
{
public:
	StaticRayTrace();
	Camera* camera;
	AccelerationBvh* bvh;
	vec3 lightAmbientColor;

	std::vector<Shape*> shapes;
	std::vector<Shape*> modelShapes;
	std::vector<Material*> materials;
	std::vector<Shape*> lights;

	void AddShape(Shape* shape);
	void AddModel(MeshData* shape, Material* mat);
	vec3 TraceRay(Ray ray);
	Intersection SampleLight(std::vector<Shape*> lights);
	IBL* ibl;

private:
	const float RussianRoulette = 0.8f;
};

