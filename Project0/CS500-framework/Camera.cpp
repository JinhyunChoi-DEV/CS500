#include "Camera.h"

Camera::Camera()
{
	eye = glm::vec3(0);
	orient = glm::quat();
	ratio = 0.0f;
}

void Camera::Set(const glm::vec3& eye_, const glm::quat& orient_, const float& ratio_)
{
	eye = eye_;
	orient = orient_;
	ratio = ratio_;
}
