#pragma once

#include "Collider.h"
#include "Contact.h"
#include "Helper.h"
#include <vector>


static glm::vec2 SAT(const glm::vec3& axis, RigidBody* col)
{
	glm::vec2 minmax{ FLT_MAX,-FLT_MAX };
	for (int i = 0; i < box_verts.size(); ++i)
	{
		glm::vec3 currPt = glm::vec3(col->m_collider->m_objTr * glm::vec4(box_verts[i],1.f));
		float dot = glm::dot(currPt, axis);
		if (dot < minmax.x)
			minmax.x = dot;
		if (dot > minmax.y)
			minmax.y = dot;
	}

	return minmax;
}

static bool overlaps(const glm::vec2& minmax1, const glm::vec2& minmax2)
{
	if (minmax1.x >= minmax2.x && minmax1.x <= minmax2.y)
		return true;
	if (minmax2.x >= minmax1.x && minmax2.x <= minmax1.y)
		return true;

	return false;
}

static bool SATSphereConvex(RigidBody* a, RigidBody* b, std::vector<ContactPoint>& cp)
{
	auto convex = std::static_pointer_cast<ConvexCollider>(a->m_collider);
	auto sphere = std::static_pointer_cast<SphereCollider>(b->m_collider);

	glm::vec3 sepAxis;
	float minDepth = FLT_MAX;
	std::pair<std::vector<size_t>,glm::vec3> face;
	for (size_t i = 0; i < convex->m_faces.size(); ++i)
	{
		glm::vec3 faceNormal = convex->m_faces[i].second;

		glm::vec3 sphereToFace = convex->m_vertices[convex->m_faces[i].first[0]] - sphere->m_position;
		float depth = glm::dot(sphereToFace, faceNormal) + sphere->m_radius;

		if (depth <= 0.f)
			return false;

		if (depth < minDepth)
		{
			face = { convex->m_faces[i].first, faceNormal };
			minDepth = depth;
			sepAxis = faceNormal;
		}
	}

	ContactPoint c;
	c.contactNormal = face.second;
	c.contactPointA = sphere->m_position - face.second * sphere->m_radius;
	c.contactPointB = sphere->m_position + face.second * (minDepth - sphere->m_radius);
	c.penetrationDepth = minDepth;
	cp.push_back(c);

	return true;
}

static bool SATFacePolygonLocal(std::shared_ptr<ConvexCollider> a, std::shared_ptr<ConvexCollider> b,
							const std::pair<std::vector<size_t>, glm::vec3>& face, float& depth)
{
	glm::vec3 faceNormalInB = glm::vec3(glm::inverse(glm::toMat4(b->m_rotation)) * glm::vec4(face.second, 0.0f));
	glm::vec3 support;
	b->ConvexFindFurthestPointLocal(-faceNormalInB, support);

	glm::vec3 faceVert = glm::vec3(glm::inverse(glm::toMat4(b->m_rotation)) * glm::vec4(a->m_vertices[face.first[0]], 1.0f));
	depth = glm::dot((faceVert - support), face.second);

	if (depth <= 0.f)
		return false;

	return true;
}

static bool SATFacePolygonGlobal(std::shared_ptr<ConvexCollider> a, std::shared_ptr<ConvexCollider> b,
	const std::pair<std::vector<size_t>, glm::vec3>& face, float& depth)
{
	glm::vec3 support;
	b->ConvexFindFurthestPoint(-face.second, support);

	glm::vec3 faceVert = a->m_vertices[face.first[0]];
	depth = glm::dot((faceVert - support), face.second);

	if (depth <= 0.f)
		return false;

	return true;
}

static float DistanceEdgeEdge(std::shared_ptr<ConvexCollider> a, std::shared_ptr<ConvexCollider> b,
									const glm::vec3& edgeAStart, const glm::vec3& edgeAEnd,
									const glm::vec3& edgeBStart, const glm::vec3& edgeBEnd, 
									glm::vec3& sepAxis)
{
	glm::vec3 edgeA = glm::normalize(edgeAEnd - edgeAStart);
	glm::vec3 edgeB = glm::normalize(edgeBEnd - edgeBStart);
	sepAxis = glm::normalize(glm::cross(edgeA, edgeB));
	float dotResult = glm::dot(sepAxis, a->m_position - edgeAStart);
	if (dotResult > 0.f)
		sepAxis = -sepAxis;

	return -glm::dot(sepAxis, edgeBStart - edgeAStart);
}

static bool EdgeBuildMinkowski(std::shared_ptr<ConvexCollider> a, std::shared_ptr<ConvexCollider> b,
								size_t edge1, size_t edge2)
{
	glm::vec3 n1 = a->m_faces[a->m_edges[edge1].second].second;
	//glm::vec3 n2
}

static bool FindSparatingAxis(RigidBody* a, RigidBody* b, bool& flip, std::vector<ContactPoint>& cp)
{
	std::shared_ptr<ConvexCollider> colA = std::static_pointer_cast<ConvexCollider>(a->m_collider);
	std::shared_ptr<ConvexCollider> colB = std::static_pointer_cast<ConvexCollider>(b->m_collider);

	glm::vec3 diff = colA->m_position - colB->m_position;
	std::vector<size_t> faceA;
	std::vector<size_t> faceB;
	flip = false;

	float minDepthA = FLT_MAX;
	float minDepthB = FLT_MAX;

	glm::vec3 sepAxisA;
	glm::vec3 sepAxisB;

	//For each face normal of collider A
	for (size_t i = 0; i < colA->m_faces.size(); ++i)
	{
		float depth;
		if (!SATFacePolygonGlobal(colA, colB, colA->m_faces[i], depth))
			return false;

		if (depth < minDepthA)
		{
			minDepthA = depth;
			faceA = colA->m_faces[i].first;
			sepAxisA = colA->m_faces[i].second;
		}
	}

	//For each face normal of collider B
	for (size_t i = 0; i < colB->m_faces.size(); ++i)
	{
		float depth;
		if (!SATFacePolygonGlobal(colB, colA, colB->m_faces[i], depth))
			return false;

		if (depth < minDepthB)
		{
			minDepthB = depth;
			faceB = colB->m_faces[i].first;
			sepAxisB = colB->m_faces[i].second;
		}
	}

	float minDepth;
	std::vector<size_t> face;
	glm::vec3 sepAxis;

	if (minDepthA < minDepthB * 1.002f + 0.0005f)
	{
		//use axis in col A
		flip = false;
		minDepth = std::min(minDepthA,minDepthB);
		face = faceA;
		sepAxis = sepAxisA;
	}
	else
	{
		flip = true;
		minDepth = std::min(minDepthA, minDepthB);
		face = faceB;
		sepAxis = sepAxisB;
	}

	return CreateFaceContact(sepAxis, flip, face, colA, colB, cp);
}
