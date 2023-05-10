#pragma once
#include "Collider.h"
#include "Contact.h"
#include "Helper.h"

#define GJK_EPA_MAX_ITER 32


static bool CheckLine(std::vector<SupportVector>& simplex, glm::vec3& direction)
{
	SupportVector a = simplex[1];
	SupportVector b = simplex[0];

	glm::vec3 ab = b.support - a.support;
	glm::vec3 ao = -a.support;

	if (glm::dot(ab, ao) > 0)
		direction = glm::cross(glm::cross(ab, ao), ab);
	else
	{
		simplex = { a };
		direction = ao;
	}

	return false;
}

static bool CheckTriangle(std::vector<SupportVector>& simplex, glm::vec3& direction)
{
	SupportVector a = simplex[2];
	SupportVector b = simplex[1];
	SupportVector c = simplex[0];

	glm::vec3 ab = b.support - a.support;
	glm::vec3 ac = c.support - a.support;
	glm::vec3 ao = -a.support;

	glm::vec3 abc = glm::cross(ab, ac);

	if (glm::dot(glm::cross(abc, ac), ao) > 0)
	{
		if (glm::dot(ac, ao) > 0)
		{
			simplex = { c, a };
			direction = glm::cross(glm::cross(ac, ao), ac);
		}
		else
			return CheckLine(simplex = { b, a }, direction);
	}
	else
	{
		if (glm::dot(glm::cross(ab, abc), ao) > 0)
			return CheckLine(simplex = { b, a }, direction);
		else
		{
			if (glm::dot(abc, ao) > 0)
				direction = abc;
			else
			{
				simplex = { b, c, a };
				direction = -abc;
			}
		}
	}

	return false;
}

static bool CheckTetrahedron(std::vector<SupportVector>& simplex, glm::vec3& direction)
{
	SupportVector a = simplex[3];
	SupportVector b = simplex[2];
	SupportVector c = simplex[1];
	SupportVector d = simplex[0];

	glm::vec3 ab = b.support - a.support;
	glm::vec3 ac = c.support - a.support;
	glm::vec3 ad = d.support - a.support;
	glm::vec3 ao = -a.support;

	glm::vec3 abc = glm::cross(ab, ac);
	glm::vec3 acd = glm::cross(ac, ad);
	glm::vec3 adb = glm::cross(ad, ab);

	if (glm::dot(abc, ao) > 0)
		return CheckTriangle(simplex = { c, b, a }, direction);
	if (glm::dot(acd, ao) > 0)
		return CheckTriangle(simplex = { d, c, a }, direction);
	if (glm::dot(adb, ao) > 0)
		return CheckTriangle(simplex = { b, d, a }, direction);

	return true;
}

static bool GJK(RigidBody* a, RigidBody* b, std::vector<SupportVector>& simplex)
{
	auto colA = a->m_collider;
	auto colB = b->m_collider;

	glm::vec3 direction = { 1,0,0 };
	SupportVector supportVec = GetSupportVector(direction, colA, colB);

	simplex.push_back(supportVec);

	direction = -supportVec.support;

	bool result = false;
	while (!result)
	{
		supportVec = GetSupportVector(direction, colA, colB);

		//no collision
		if (glm::dot(supportVec.support, direction) <= 0)
			return false;

		simplex.push_back(supportVec);
	
		if (simplex.size() > 4)
			simplex.pop_back();

		if (simplex.size() == 2) //line
			result = CheckLine(simplex, direction);

		else if (simplex.size() == 3) // triangle
			result = CheckTriangle(simplex, direction);

		else if (simplex.size() == 4) // tetrahedron
			result = CheckTetrahedron(simplex, direction);

	}

	return result;
}

static std::pair<std::vector<glm::vec4>, size_t> GetFaceNormals(const std::vector<SupportVector>& polytope, const std::vector<size_t>& faces)
{
	std::vector<glm::vec4> normals;
	size_t minTriangle = 0;
	float minDist = FLT_MAX;

	for (size_t i = 0; i < faces.size(); i += 3)
	{
		glm::vec3 a = polytope[faces[i]].support;
		glm::vec3 b = polytope[faces[i + 1]].support;
		glm::vec3 c = polytope[faces[i + 2]].support;

		glm::vec3 normal = glm::normalize(glm::cross(b - a, c - a));
		float dist = glm::dot(normal, a);

		if (dist < 0)
		{
			normal *= -1;
			dist *= -1;
		}

		normals.emplace_back(normal, dist);

		if (dist < minDist)
		{
			minTriangle = i / 3;
			minDist = dist;
		}
	}

	return std::make_pair(normals, minTriangle);
}

static void AddEdge(std::vector<std::pair<size_t, size_t>>& edges, const std::vector<size_t>& faces, size_t a, size_t b)
{
	auto it = std::find(edges.begin(), edges.end(), std::make_pair(faces[b], faces[a]));
	if (it != edges.end())
		edges.erase(it);
	else
		edges.emplace_back(faces[a], faces[b]);

}

static void EPA(const std::vector<SupportVector>& simplex, RigidBody* a, RigidBody* b, std::vector<ContactPoint>& colData)
{
	auto colA = a->m_collider;
	auto colB = b->m_collider;

	std::vector<SupportVector> polytope(simplex.rbegin(), simplex.rend());
	std::vector<size_t> faces = { 0 ,1, 2,
								  0, 3, 1,
								  0, 2, 3,
								  1, 3, 2 };

	std::pair<std::vector<glm::vec4>, size_t> normals = GetFaceNormals(polytope, faces);

	glm::vec3 minNormal;
	float minDist = FLT_MAX;
	glm::vec3 contactPointA, contactPointB;

	size_t iterations = 0;
	while (minDist == FLT_MAX)
	{
		if (iterations++ > GJK_EPA_MAX_ITER)
			break;

		minNormal = glm::vec3(normals.first[normals.second]);
		minDist = normals.first[normals.second].w;

		SupportVector supportVec = GetSupportVector(minNormal, colA, colB);

		float supportDist = glm::dot(minNormal, supportVec.support);

		if (abs(supportDist - minDist) <= 0.001f)
		{
			size_t index = normals.second * 3;
			glm::vec3 contactA, contactB;
			std::vector<glm::vec3> curr_faces = { polytope[faces[index]].support, polytope[faces[index + 1]].support, polytope[faces[index + 2]].support };
			FindClosestPoint(supportVec.support, polytope[faces[index]], polytope[faces[index + 1]], polytope[faces[index + 2]], supportDist, contactA, contactB);

			return;
		}
		else//if (abs(supportDist - minDist) > 0.001f)
		{
			minDist = FLT_MAX;

			std::vector<std::pair<size_t, size_t>> uniqueEdges;

			for (size_t i = 0; i < normals.first.size(); ++i)
			{
				if (glm::dot(glm::vec3(normals.first[i]), supportVec.support) > 0)
				{
					size_t f = i * 3;

					AddEdge(uniqueEdges, faces, f, f + 1);
					AddEdge(uniqueEdges, faces, f + 1, f + 2);
					AddEdge(uniqueEdges, faces, f + 2, f);

					faces[f + 2] = faces.back();
					faces.pop_back();
					faces[f + 1] = faces.back();
					faces.pop_back();
					faces[f] = faces.back();
					faces.pop_back();

					normals.first[i] = normals.first.back();
					normals.first.pop_back();

					--i;
				}
			}

			if (uniqueEdges.empty())
				break;

			std::vector<size_t> newFaces;
			for (size_t i = 0; i < uniqueEdges.size(); ++i)
			{
				newFaces.push_back(uniqueEdges[i].first);
				newFaces.push_back(uniqueEdges[i].second);
				newFaces.push_back(polytope.size());
			}

			polytope.push_back(supportVec);

			std::pair<std::vector<glm::vec4>, size_t> newNormals = GetFaceNormals(polytope, newFaces);

			float oldMinDist = FLT_MAX;
			for (size_t i = 0; i < newNormals.first.size(); ++i)
			{
				if (newNormals.first[i].w < oldMinDist)
				{
					oldMinDist = newNormals.first[i].w;
					normals.second = i;
				}
			}

			if (newNormals.first[newNormals.second].w < oldMinDist)
				normals.second = newNormals.second + normals.first.size();

			faces.insert(faces.end(), newFaces.begin(), newFaces.end());
			normals.first.insert(normals.first.end(), newNormals.first.begin(), newNormals.first.end());
		}

	}

	if (minDist == FLT_MAX)
		return;

	ContactPoint c;
	c.contactPointA = contactPointA;
	c.contactPointB = contactPointB;
	c.contactNormal = minNormal;
	c.penetrationDepth = minDist;

	colData.push_back(c);
}

