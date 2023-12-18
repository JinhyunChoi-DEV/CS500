#include "Camera.h"
#include "geom.h"

Camera::Camera()
{
	eye = vec3(0);
	X = vec3(0);
	Y = vec3(0);
	Z = vec3(0);
	orient = quat();
	r_x = 0.0f;
	r_y = 0.0f;
}

void Camera::Set(const vec3& eye_, const quat& orient_, const float& r_y_, const int& width, const int& height)
{
	eye = eye_;
	orient = orient_;
	r_y = r_y_;
	r_x = (r_y * static_cast<float>(width)) / static_cast<float>(height);
	X = r_x * transformVector(orient, Xaxis());
	Y = r_y * transformVector(orient, Yaxis());
	Z = transformVector(orient, Zaxis());
}
