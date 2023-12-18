#include "Shape.h"
#include "Intersection.h"
#include "Ray.h"
#include "raytrace.h"

namespace MeshCreator
{
	MeshData* CreateSphere();
	MeshData* CreateBox();
	MeshData* CreateCylinder();
}

MeshData* sphere = MeshCreator::CreateSphere();
MeshData* box = MeshCreator::CreateBox();
MeshData* cylinder = MeshCreator::CreateCylinder();

Sphere::Sphere(const vec3 center_, const float r, Material* mat)
{
	mesh = sphere;
	material = mat;
	center = center_;
}

void Sphere::intersect(Ray, Intersection&) {}

Box::Box(const vec3 center_, const vec3 diagonal, Material* mat)
{
	mesh = box;
	material = mat;
	center = center_;
}

void Box::intersect(Ray, Intersection&) {}

Cylinder::Cylinder(const vec3 base, const vec3 axis, const float r, Material* mat)
{
	mesh = cylinder;
	material = mat;
	center = base;
}

void Cylinder::intersect(Ray, Intersection&) {}

Triangle::Triangle(MeshData* mesh_)
{
	mesh = mesh_;

}

void Triangle::intersect(Ray, Intersection&) {}


namespace MeshCreator
{
	MeshData* CreateSphere()
	{
		MeshData* meshdata = new MeshData();
		unsigned int n = 20;
		float d = 2.0f * PI / float(n * 2);
		for (unsigned int i = 0; i <= n * 2; i++) {
			float s = i * 2.0f * PI / float(n * 2);
			for (unsigned int j = 0; j <= n; j++) {
				float t = j * PI / float(n);
				float x = cos(s) * sin(t);
				float y = sin(s) * sin(t);
				float z = cos(t);
				meshdata->vertices.push_back(VertexData(vec3(x, y, z),
					vec3(x, y, z),
					vec2(s / (2 * PI), t / PI),
					vec3(sin(s), cos(s), 0.0)));
				if (i > 0 && j > 0) {
					meshdata->triangles.push_back(ivec3((i - 1) * (n + 1) + (j - 1),
						(i - 1) * (n + 1) + (j),
						(i) * (n + 1) + (j)));
					meshdata->triangles.push_back(ivec3((i - 1) * (n + 1) + (j - 1),
						(i) * (n + 1) + (j),
						(i) * (n + 1) + (j - 1)));
				}
			}
		}
		return meshdata;
	}

	MeshData* CreateBox()
	{
		mat4 face[6] = {
			Identity(),
			rotate(180.0f * Radians, vec3(1.0f, 0.0f, 0.0f)),
			rotate(90.0f * Radians, vec3(1.0f, 0.0f, 0.0f)),
			rotate(-90.0f * Radians, vec3(1.0f, 0.0f, 0.0f)),
			rotate(90.0f * Radians, vec3(0.0f, 1.0f, 0.0f)),
			rotate(-90.0f * Radians, vec3(0.0f, 1.0f, 0.0f)) };

		mat4 half = translate(vec3(0.5f, 0.5f, 0.5f)) * scale(vec3(0.5f, 0.5f, 0.5f));
		MeshData* meshdata = new MeshData();
		for (unsigned int f = 0; f < 6; f++) {
			mat4 m4 = half * face[f];
			mat3 m3 = mat3(m4); // Extracts 3x3 from a 4x4
			for (unsigned int i = 0; i < 2; i++) {
				for (unsigned int j = 0; j < 2; j++) {
					vec4 p = m4 * vec4(float(2 * i) - 1.0f, float(2 * j) - 1.0f, 1.0f, 1.0f);
					vec3 tnrm = m3 * vec3(0.0f, 0.0f, 1.0f);
					vec3 ttan = m3 * vec3(1.0, 0.0, 0.0);
					meshdata->vertices.push_back(VertexData(vec3(p[0], p[1], p[2]),
						vec3(tnrm[0], tnrm[1], tnrm[2]),
						vec2(float(i), float(j)),
						vec3(ttan[0], ttan[1], ttan[2])));
					meshdata->triangles.push_back(ivec3(4 * f + 0, 4 * f + 1, 4 * f + 3));
					meshdata->triangles.push_back(ivec3(4 * f + 0, 4 * f + 3, 4 * f + 2));
				}
			}
		}
		return meshdata;
	}

	MeshData* CreateCylinder()
	{
		MeshData* meshdata = new MeshData();
		unsigned int n = 20;
		float d = 2.0f * PI / float(n * 2);
		for (unsigned int i = 0; i <= n; i++) {
			float s = i * 2.0f * PI / float(n);
			float x = cos(s);
			float y = sin(s);

			meshdata->vertices.push_back(VertexData(vec3(x, y, 0.0f),
				vec3(x, y, 0.0f),
				vec2(s / (2 * PI), 0.0f),
				vec3(-sin(s), cos(s), 0.0f)));

			meshdata->vertices.push_back(VertexData(vec3(x, y, 1.0f),
				vec3(x, y, 0.0f),
				vec2(s / (2 * PI), 0.0f),
				vec3(-sin(s), cos(s), 0.0f)));

			if (i > 0) {
				meshdata->triangles.push_back(ivec3((i - 1) * 2 + 1, (i - 1) * 2, (i) * 2));
				meshdata->triangles.push_back(ivec3((i - 1) * 2 + 1, (i) * 2, (i) * 2 + 1));
			}
		}
		return meshdata;
	}
}
