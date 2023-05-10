#pragma once

//#include <glm/glm.hpp>
#include <glm/gtx/quaternion.hpp>

class Trans
{
public:
	Trans(const glm::vec3 position = glm::vec3(0, 0, 0)
		, const glm::vec3 scale = glm::vec3(1, 1, 1)
		, const glm::vec3 rotateAxis = glm::vec3(0,0,0)
		, const float angle = 0)
		: m_position(position), m_scale(scale), m_rotation(glm::angleAxis(angle, rotateAxis))
	{
	}

	void ResetTrans(const Trans& r)
	{
		m_position = r.m_position;
		m_scale = r.m_scale;
		m_rotation = r.m_rotation;
		m_objTr = r.m_objTr;
	}

	glm::vec3 m_position;
	glm::vec3 m_scale;
	glm::quat m_rotation;

	glm::mat4 m_objTr;


	glm::vec3 m_prevPos;
	glm::quat m_prevRot;
};