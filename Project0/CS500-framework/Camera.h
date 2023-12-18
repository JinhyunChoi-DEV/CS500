#pragma once
#include <glm/glm.hpp>
#include <glm/gtx/quaternion.hpp>

class Camera
{
public:
	Camera();

	void Set(const glm::vec3& eye_, const glm::quat& orient_, const float& ratio_);

	glm::vec3 eye;
	glm::quat orient;
	float ratio;
};