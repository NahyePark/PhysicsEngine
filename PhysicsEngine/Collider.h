#pragma once

#include "Trans.h"
#include "shapes.h"
#include <iostream>
#include <tuple>

static std::vector<glm::vec3> box_verts = { glm::vec3(1,1,1),
										glm::vec3(1,1,-1),
										glm::vec3(-1,1,-1),
										glm::vec3(-1,1,1),
										glm::vec3(1,-1,1),
										glm::vec3(1,-1,-1),
										glm::vec3(-1,-1,-1),
										glm::vec3(-1,-1,1)
										 };

static std::vector<std::vector<size_t>> box_faces = {
	{0,1,2,3},
	{3,2,6,7},
	{7,6,5,4},
	{0,4,5,1},
	{0,3,7,4},
	{6,2,1,5}
};

static std::vector<glm::vec3> box_normals = {
	glm::vec3(0,1,0),
	glm::vec3(-1,0,0),
	glm::vec3(0,-1,0),
	glm::vec3(1,0,0),
	glm::vec3(0,0,1),
	glm::vec3(0,0,-1)
};

struct AABB
{
	glm::vec3 m_lower;
	glm::vec3 m_upper;

	glm::mat4 m_trans;
	bool isColliding = false;

	Shape* m_shape;
	void Draw(int programId);

	bool Contains(const glm::vec3& p)
	{
		return (
			p.x >= m_lower.x &&
			p.x <= m_upper.x &&
			p.y >= m_lower.y &&
			p.y <= m_upper.y &&
			p.z >= m_lower.z &&
			p.z <= m_upper.z
			);
	}

	bool Contains(const AABB& r)
	{
		return (Contains(r.m_lower) && Contains(r.m_upper));
	}

	void Update(const glm::vec3& p, const glm::vec3& s, const glm::quat& r);
};

enum class BoundingType
{
	SPHERE,
	BOX,
	CONVEX,
	NUMSHAPES
};

class Collider : public Trans
{
public:
	Collider(Shape* shape, BoundingType type, const glm::vec3& col = {0,0,1})
		: m_shape(shape), m_type(type), m_color(col)
	{}

	void Draw() { m_shape->DrawVAO(true); }
	void UpdateAABB()
	{
		m_aabb.Update(m_position, m_scale, m_rotation);

	}
	void UpdateMatrix()
	{
		glm::mat4 curr_rot = glm::toMat4(m_rotation);
		m_objTr = Translate(m_position.x, m_position.y, m_position.z) *
			curr_rot * Scale(m_scale.x, m_scale.y, m_scale.z);
	}

	BoundingType m_type;
	Shape* m_shape;
	glm::vec3 m_color;
	AABB m_aabb; //for dynamic AABB Tree node
};

class OBBCollider : public Collider
{
public:
	OBBCollider(Shape* shape)
		: Collider(shape, BoundingType::BOX)
	{
		m_vertices = box_verts;

		for (size_t i = 0; i < shape->Nrm.size(); ++i)
		{
			auto it = std::find_if(m_normals.begin(), m_normals.end(), [&](const glm::vec3& a)
				{
					return (a.x == shape->Nrm[i].x) && (a.y == shape->Nrm[i].y) && (a.z == shape->Nrm[i].z);
				});
			if (it == m_normals.end())
				m_normals.emplace_back(glm::vec3(shape->Nrm[i]));
		}

		for (size_t i = 0; i < shape->Tri.size(); ++i)
		{
			glm::vec3 a = glm::vec3(shape->Pnt[shape->Tri[i][0]]);
			glm::vec3 b = glm::vec3(shape->Pnt[shape->Tri[i][1]]);
			glm::vec3 c = glm::vec3(shape->Pnt[shape->Tri[i][2]]);

			glm::vec3 edge0 = glm::normalize(b - a);
			glm::vec3 edge1 = glm::normalize(c - b);
			glm::vec3 edge2 = glm::normalize(a - c);

			auto it = std::find_if(m_edges.begin(), m_edges.end(), [&](const glm::vec3& a)
				{
					return (a.x == edge0.x) && (a.y == edge0.y) && (a.z == edge0.z);
				});
			if (it == m_edges.end())
				m_edges.emplace_back(edge0);

			it = std::find_if(m_edges.begin(), m_edges.end(), [&](const glm::vec3& a)
				{
					return (a.x == edge1.x) && (a.y == edge1.y) && (a.z == edge1.z);
				});
			if (it == m_edges.end())
				m_edges.emplace_back(edge1);

			it = std::find_if(m_edges.begin(), m_edges.end(), [&](const glm::vec3& a)
				{
					return (a.x == edge2.x) && (a.y == edge2.y) && (a.z == edge2.z);
				});
			if (it == m_edges.end())
				m_edges.emplace_back(edge2);
		}

		m_localAxis.push_back(glm::vec3(1, 0, 0));
		m_localAxis.push_back(glm::vec3(0, 1, 0));
		m_localAxis.push_back(glm::vec3(0, 0, 1));
	}

	void OBBResetCollider(glm::mat4 rot)
	{
		m_localAxis[0] = glm::vec3(rot * glm::vec4(1, 0, 0, 1.0));
		m_localAxis[1] = glm::vec3(rot * glm::vec4(0, 1, 0, 1.0));
		m_localAxis[2] = glm::vec3(rot * glm::vec4(0, 0, 1, 1.0));

		UpdateAABB();
	}

	void OBBUpdate()
	{
		glm::mat4 curr_rot = glm::toMat4(m_rotation);
		for (int i = 0; i <m_localAxis.size(); ++i)
			m_localAxis[i] = glm::vec3(curr_rot * glm::vec4(m_localAxis[i], 1.0));

		for (size_t i = 0; i < m_vertices.size(); ++i)
			m_vertices[i] = glm::vec3( m_objTr* m_shape->Pnt[i]);

	}
	void OBBFindFurthestPoint(const glm::vec3& direction, glm::vec3& result);

	std::vector<glm::vec3> m_localAxis;	
	std::vector<glm::vec3> m_vertices;
	std::vector<glm::vec3> m_normals;
	std::vector<glm::vec3> m_edges;

};

class SphereCollider : public Collider
{
public:
	SphereCollider(Shape* shape)
		: Collider(shape, BoundingType::SPHERE)
	{
	}
	void SphereResetCollider(glm::mat4 rot)
	{
		UpdateAABB();
	}

	void SphereUpdate()
	{
		m_radius = m_scale.x;
	}

	float m_radius;

};

class ConvexCollider : public Collider
{
public:
	ConvexCollider(Shape* shape)
		: Collider(shape, BoundingType::CONVEX)
	{
		m_vertices = box_verts;
		for (size_t i = 0; i < box_faces.size(); ++i)
			m_faces.push_back(std::make_pair(box_faces[i], box_normals[i]));

		for (size_t i = 0; i < m_faces.size(); ++i)
		{
			auto curr_face = m_faces[i].first;
			for (size_t j = 0; j < curr_face.size(); ++j)
			{
				size_t next = (j + 1)% curr_face.size();
				m_edges.push_back(std::make_pair(std::make_pair(j, next), i));
			}
		}
	}

	void ConvexResetCollider(glm::mat4 rot)
	{
		UpdateAABB();
	}

	void ConvexUpdate()
	{
		for (size_t i = 0; i < m_vertices.size(); ++i)
			m_vertices[i] = glm::vec3(m_objTr * glm::vec4(box_verts[i],1.f));

		for (size_t i = 0; i < m_faces.size(); ++i)
			m_faces[i].second = glm::vec3(glm::toMat4(m_rotation) * glm::vec4(box_normals[i], 1.f));
	}

	void ConvexFindFurthestPoint(const glm::vec3& direction, glm::vec3& result);
	void ConvexFindFurthestPointLocal(const glm::vec3& direction, glm::vec3& result);

	std::vector<glm::vec3> m_vertices;
							//face indices, normal
	std::vector<std::pair<std::vector<size_t>, glm::vec3>> m_faces;
	std::vector<std::pair<std::pair<size_t,size_t>, size_t>> m_edges;
	

};

