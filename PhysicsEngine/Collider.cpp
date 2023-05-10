#include <glbinding/gl/gl.h>
#include <glbinding/Binding.h>
using namespace gl;

#include "Collider.h"

void AABB::Draw(int programId)
{
	glm::vec3 pos = (m_upper + m_lower) * 0.5f;
	glm::vec3 scale = (m_upper - m_lower) * 0.5f;
	m_trans = Translate(pos.x, pos.y, pos.z) * Scale(scale.x, scale.y, scale.z);

	glm::vec3 color = { 0,1,0 };
	if (isColliding)
		color = { 1,0,0 };

	int loc = glGetUniformLocation(programId, "lineColor");
	glUniform3fv(loc, 1, &color[0]);

	loc = glGetUniformLocation(programId, "ModelTr");
	glUniformMatrix4fv(loc, 1, GL_FALSE, Pntr(m_trans));

	glm::mat4 inv = glm::inverse(m_trans);
	loc = glGetUniformLocation(programId, "NormalTr");
	glUniformMatrix4fv(loc, 1, GL_FALSE, Pntr(inv));

	loc = glGetUniformLocation(programId, "mode");
	glUniform1i(loc, 1);

	m_shape->DrawVAO(true);
}

void AABB::Update(const glm::vec3& p, const glm::vec3& s, const glm::quat& r)
{
	glm::vec3 trans = p;
	glm::vec3 scale = s;
	glm::mat4 rot = glm::toMat4(r);


	glm::vec3 newUpper = trans;
	glm::vec3 newLower = trans;

	float aMin[3], aMax[3];
	float bMin[3], bMax[3];

	aMin[0] = scale.x;
	aMin[1] = scale.y;
	aMin[2] = scale.z;
	aMax[0] = -scale.x;
	aMax[1] = -scale.y;
	aMax[2] = -scale.z;

	bMin[0] = bMax[0] = trans.x;
	bMin[1] = bMax[1] = trans.y;
	bMin[2] = bMax[2] = trans.z;

	float a, b;
	for (int i = 0; i < 3; ++i)
	{
		for (int j = 0; j < 3; ++j)
		{
			a = rot[i][j] * aMin[j];
			b = rot[i][j] * aMax[j];

			if (a < b)
			{
				bMin[i] += a;
				bMax[i] += b;
			}
			else
			{
				bMin[i] += b;
				bMax[i] += a;
			}
		}
	}

	m_lower = { bMin[0], bMin[1], bMin[2] };
	m_upper = { bMax[0], bMax[1], bMax[2] };
}

void ConvexCollider::ConvexFindFurthestPoint(const glm::vec3& direction, glm::vec3& result)
{
	float maxDist = -FLT_MAX;
	for (int i = 0; i < m_vertices.size(); ++i)
	{
		float currDist = glm::dot(m_vertices[i], direction);
		if (currDist > maxDist)
		{
			maxDist = currDist;
			result = m_vertices[i];
		}
	}
}

void ConvexCollider::ConvexFindFurthestPointLocal(const glm::vec3& direction, glm::vec3& result)
{
	float maxDist = -FLT_MAX;
	for (int i = 0; i < box_verts.size(); ++i)
	{
		float currDist = glm::dot(box_verts[i], direction);
		if (currDist > maxDist)
		{
			maxDist = currDist;
			result = box_verts[i];
		}
	}

	result.x *= m_scale.x;
	result.y *= m_scale.y;
	result.z *= m_scale.z;
}

void OBBCollider::OBBFindFurthestPoint(const glm::vec3& direction, glm::vec3& result)
{
	float maxDist = -FLT_MAX;
	for (int i = 0; i < m_vertices.size(); ++i)
	{
		float currDist = glm::dot(m_vertices[i], direction);
		if (currDist > maxDist)
		{
			maxDist = currDist;
			result = m_vertices[i];
		}
	}
}

