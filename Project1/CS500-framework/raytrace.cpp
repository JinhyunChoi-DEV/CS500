//////////////////////////////////////////////////////////////////////
// Provides the framework for a raytracer.
////////////////////////////////////////////////////////////////////////

#include <vector>
#include <fstream>

#ifdef _WIN32
	// Includes for Windows
#include <windows.h>
#include <cstdlib>
#include <limits>
#include <crtdbg.h>
#else
	// Includes for Linux
#endif

#include "geom.h"
#include "raytrace.h"
#include "StaticRayTrace.h"
#include "Shape.h"
#include "acceleration.h"

#define STB_IMAGE_IMPLEMENTATION
#define STBI_FAILURE_USERMSG
#include "stb_image.h"

#include "Camera.h"

Scene::Scene()
{
	staticRayTrace = new StaticRayTrace();
}

void Scene::Finit()
{
	bvh = new AccelerationBvh(staticRayTrace->shapes);
}

void Scene::triangleMesh(MeshData* mesh)
{
	staticRayTrace->AddModel(mesh, currentMat);
}

Texture::Texture(const std::string& bpath) : id(0)
{
	// Replace backslashes with forward slashes -- Good for Linux, and maybe Windows?
	std::string path = bpath;
	std::string bs = "\\";
	std::string fs = "/";
	while (path.find(bs) != std::string::npos) {
		path.replace(path.find(bs), 1, fs);
	}

	// Does the file exist?
	std::ifstream find_it(path.c_str());
	if (find_it.fail()) {
		std::cerr << "Texture file not found: " << path << std::endl;
		exit(-1);
	}
	else {
		// Read image, and check for success
		stbi_set_flip_vertically_on_load(true);
		image = stbi_load(path.c_str(), &width, &height, &depth, 4);
		printf("%d %d %d %s\n", depth, width, height, path.c_str());
		if (!image) {
			printf("\nRead error on file %s:\n  %s\n\n", path.c_str(), stbi_failure_reason());
			exit(-1);
		}
	}
}

quat Orientation(int i,
	const std::vector<std::string>& strings,
	const std::vector<float>& f)
{
	quat q(1, 0, 0, 0); // Unit quaternion
	while (i < strings.size()) {
		std::string c = strings[i++];
		if (c == "x")
			q *= angleAxis(f[i++] * Radians, Xaxis());
		else if (c == "y")
			q *= angleAxis(f[i++] * Radians, Yaxis());
		else if (c == "z")
			q *= angleAxis(f[i++] * Radians, Zaxis());
		else if (c == "q") {
			q *= quat(f[i + 0], f[i + 1], f[i + 2], f[i + 3]);
			i += 4;
		}
		else if (c == "a") {
			q *= angleAxis(f[i + 0] * Radians, normalize(vec3(f[i + 1], f[i + 2], f[i + 3])));
			i += 4;
		}
	}
	return q;
}

void Scene::Command(const std::vector<std::string>& strings,
	const std::vector<float>& f)
{
	if (strings.size() == 0) return;
	std::string c = strings[0];

	if (c == "screen") {
		// syntax: screen width height
		width = int(f[1]);
		height = int(f[2]);
	}

	else if (c == "camera") {
		// syntax: camera x y z   ry   <orientation spec>
		// Eye position (x,y,z),  view orientation (qw qx qy qz),  frustum height ratio ry
		staticRayTrace->camera->Set(vec3(f[1], f[2], f[3]), Orientation(5, strings, f), f[4], width, height);
	}

	else if (c == "ambient") {
		// syntax: ambient r g b
		// Sets the ambient color.  Note: This parameter is temporary.
		// It will be ignored once your raytracer becomes capable of
		// accurately *calculating* the true ambient light.
		staticRayTrace->lightAmbientColor = vec3(f[1], f[2], f[3]);
	}

	else if (c == "brdf") {
		// syntax: brdf  r g b   r g b  alpha
		// later:  brdf  r g b   r g b  alpha  r g b ior
		// First rgb is Diffuse reflection, second is specular reflection.
		// third is beer's law transmission followed by index of refraction.
		// Creates a Material instance to be picked up by successive shapes
		currentMat = new Material(vec3(f[1], f[2], f[3]), vec3(f[4], f[5], f[6]), f[7]);
	}

	else if (c == "light") {
		// syntax: light  r g b   
		// The rgb is the emission of the light
		// Creates a Material instance to be picked up by successive shapes
		currentMat = new Light(vec3(f[1], f[2], f[3]));
	}

	else if (c == "sphere") {
		// syntax: sphere x y z   r
		// Creates a Shape instance for a sphere defined by a center and radius
		auto shape = new Sphere(vec3(f[1], f[2], f[3]), f[4], currentMat);
		staticRayTrace->AddShape(shape);
	}

	else if (c == "box") {
		// syntax: box bx by bz   dx dy dz
		// Creates a Shape instance for a box defined by a corner point and diagonal vector
		auto shape = new Box(vec3(f[1], f[2], f[3]), vec3(f[4], f[5], f[6]), currentMat);
		staticRayTrace->AddShape(shape);
	}

	else if (c == "cylinder") {
		// syntax: cylinder bx by bz   ax ay az  r
		// Creates a Shape instance for a cylinder defined by a base point, axis vector, and radius
		auto shape = new Cylinder(vec3(f[1], f[2], f[3]), vec3(f[4], f[5], f[6]), f[7], currentMat);
		staticRayTrace->AddShape(shape);
	}


	else if (c == "mesh") {
		// syntax: mesh   filename   tx ty tz   s   <orientation>
		// Creates many Shape instances (one per triangle) by reading
		// model(s) from filename. All triangles are rotated by a
		// quaternion (qw qx qy qz), uniformly scaled by s, and
		// translated by (tx ty tz) .
		mat4 modelTr = translate(vec3(f[2], f[3], f[4]))
			* scale(vec3(f[5], f[5], f[5]))
			* toMat4(Orientation(6, strings, f));
		ReadAssimpFile(strings[1], modelTr);
	}


	else {
		fprintf(stderr, "\n*********************************************\n");
		fprintf(stderr, "* Unknown command: %s\n", c.c_str());
		fprintf(stderr, "*********************************************\n\n");
	}
}

void Scene::TraceImage(Color* image, const int pass)
{
	vec3 X = staticRayTrace->camera->X;
	vec3 Y = staticRayTrace->camera->Y;
	vec3 Z = staticRayTrace->camera->Z;

	vec3 eye = staticRayTrace->camera->eye;
	int lightSize = static_cast<int>(staticRayTrace->lights.size());

#pragma omp parallel for schedule(dynamic, 1) // Magic: Multi-thread y loop
	for (int y = 0; y < height; y++)
	{
		fprintf(stderr, "Rendering %4d\r", y);
		for (int x = 0; x < width; x++)
		{
			float dx = (2 * (static_cast<float>(x) + 0.5f) / static_cast<float>(width)) - 1;
			float dy = (2 * (static_cast<float>(y) + 0.5f) / static_cast<float>(height)) - 1;

			vec3 dir = normalize(dx * X + dy * Y - Z);
			Ray ray(eye, dir);

			// not using bvh algorithm
			/*Intersection result, current;
			for (auto obj : staticRayTrace->shapes)
			{
				obj->intersect(ray, current);
				if (current.object != nullptr)
				{
					if (result.t > current.t)
						result = current;
				}
			}*/


			// using bvh algorithm
			Intersection result = bvh->intersect(ray);

			if (result.object != nullptr)
			{
				float diffuse = 0.0f;
				for (int i = 0; i < lightSize; ++i)
				{
					auto shape = staticRayTrace->lights[i];
					vec3 lightPos = shape->base;
					vec3 lightDir = lightPos - result.point;
					diffuse += dot(normalize(result.normal), normalize(lightDir));
				}

				// diffuse color
				image[y * width + x] = diffuse * result.object->material->Kd;

				// base color
				//image[y * width + x] = result.object->material->Kd;

				// t value color
				//image[y * width + x] = vec3((result.t - 4.0f) / 5.0f);

				// normal color
				//image[y * width + x] = vec3(glm::abs(normalize(result.normal)));
			}
		}
	}

	fprintf(stderr, "\n");
}
