#pragma once

#include "RigidBody.h"


struct SupportVector
{
	glm::vec3 support;
	glm::vec3 supportA;
	glm::vec3 supportB;

	glm::mat4 objTrA;
	glm::mat4 objTrB;

};


static void FindFurthestPoint(std::shared_ptr<Collider> col, const glm::vec3& direction, glm::vec3& result)
{
	if (col->m_type == BoundingType::BOX)
		std::static_pointer_cast<OBBCollider>(col)->OBBFindFurthestPoint(direction, result);
	else if (col->m_type == BoundingType::CONVEX)
		std::static_pointer_cast<ConvexCollider>(col)->ConvexFindFurthestPoint(direction, result);
}

static SupportVector GetSupportVector(const glm::vec3& direction, std::shared_ptr<Collider> colA, std::shared_ptr<Collider> colB)
{
	SupportVector result;
	FindFurthestPoint(colA, direction, result.supportA);
	FindFurthestPoint(colB, -direction, result.supportB);
	result.support = result.supportA - result.supportB;
	result.objTrA = colA->m_objTr;
	result.objTrB = colB->m_objTr;
	//result.support *= 0.01f; //margin
	return result;
}


static void SegmentsClosestPoints(const glm::vec3& pointAStart, const glm::vec3& pointAEnd,
								  const glm::vec3& pointBStart, const glm::vec3& pointBEnd, 
								  glm::vec3& closestPointA, glm::vec3& closestPointB)
{
	glm::vec3 dA = pointAEnd - pointAStart;
	glm::vec3 dB = pointBEnd - pointBStart;
	glm::vec3 r = pointAStart - pointBStart;
	float a = glm::length2(dA);
	float e = glm::length2(dB);
	float f = glm::dot(dB, r);
	float s, t;

	if (a <= DBL_EPSILON && e <= DBL_EPSILON)
	{
		closestPointA = pointAStart;
		closestPointB = pointBStart;
		return;
	}

	if (a <= DBL_EPSILON)
	{
		s = 0.f;
		t = glm::clamp(f / e, 0.f, 1.f);
	}
	else
	{
		float c = glm::dot(dA, r);
		if (e <= DBL_EPSILON)
		{
			t = 0.f;
			s = glm::clamp(-c / a, 0.f, 1.f);
		}
		else
		{
			float b = glm::dot(dA, dB);
			float denom = a * e - b * b;

			if (denom != 0.f)
				s = glm::clamp((b * f - c * e) / denom, 0.f, 1.f);
			else
				s = 0.f;

			t = (b * s + f) / e;

			if (t < 0.f)
			{
				t = 0.f;
				s = glm::clamp(-c / a, 0.f, 1.f);
			}
			else if (t > 1.f)
			{
				t = 1.f;
				s = glm::clamp((b - c), 0.f, 1.f);
			}
		}
	}

	closestPointA = pointAStart + dA * s;
	closestPointB = pointBStart + dB * t;
}


static bool CreateEdgeContact(std::shared_ptr<ConvexCollider> colA, std::shared_ptr<ConvexCollider> colB,
								size_t e1, size_t e2, float depth, const glm::vec3& sepNormal,
								std::vector<ContactPoint>& cp)
{
	glm::vec3 edgeAStart = colA->m_vertices[colA->m_edges[e1].first.first];
	glm::vec3 edgeAEnd = colA->m_vertices[colA->m_edges[e1].first.second];
	glm::vec3 edgeBStart = colA->m_vertices[colA->m_edges[e2].first.first];
	glm::vec3 edgeBEnd = colA->m_vertices[colA->m_edges[e2].first.second];
	glm::vec3 contactPointA, contactPointB;

	SegmentsClosestPoints(edgeAStart, edgeAEnd, edgeBStart, edgeBEnd, contactPointA, contactPointB);

	ContactPoint c;
	c.contactNormal = sepNormal;
	c.contactPointA = contactPointA;
	c.contactPointB = contactPointB;
	c.penetrationDepth = depth;
	cp.push_back(c);

	return true;
}

static size_t FindMostAntiParallelFace(std::shared_ptr<Collider> col, const glm::vec3& direction)
{
	std::vector<std::pair<std::vector<size_t>, glm::vec3>> faces;
	if (col->m_type == BoundingType::CONVEX)
		faces = std::static_pointer_cast<ConvexCollider>(col)->m_faces;

	float minDot = FLT_MAX;
	size_t result = 0;

	for (size_t i = 0; i < faces.size(); ++i)
	{
		float dot = glm::dot(faces[i].second, direction);
		if (dot < minDot)
		{
			minDot = dot;
			result = i;
		}
	}

	return result;
}

static float PlaneLineIntersection(const glm::vec3& v0, const glm::vec3& v1, float dot, const glm::vec3& n)
{
	float dot_n_ab = glm::dot(n, v1 - v0);
	float t = -1.f;

	if (std::abs(dot_n_ab) > 0.0001f)
		t = (dot - glm::dot(n, v0)) / dot_n_ab;

	return t;
}

static void ClipPolygonWithPlane(const std::vector<glm::vec3>& polygonVerts, const glm::vec3& planePoint,
	const glm::vec3& planeNormal, std::vector<glm::vec3>& out)
{
	size_t start = polygonVerts.size() - 1;
	float dot_normpoint = glm::dot(planeNormal, planePoint);

	float dot_start = glm::dot((polygonVerts[start] - planePoint), planeNormal);

	for (size_t end = 0; end < polygonVerts.size(); ++end)
	{
		glm::vec3 v0 = polygonVerts[start];
		glm::vec3 v1 = polygonVerts[end];

		float dot_end = glm::dot((v1 - planePoint), planeNormal);
		if (dot_end >= 0.f)
		{
			if (dot_start < 0.f)
			{
				float t = PlaneLineIntersection(v0, v1, dot_normpoint, planeNormal);

				if (t >= 0.f && t <= 1.f)
					out.push_back(v0 + t * (v1 - v0));
				else
					out.push_back(v1);
			}

			out.push_back(v1);
		}
		else
		{
			if (dot_start >= 0.f)
			{
				float t = PlaneLineIntersection(v0, v1, -dot_normpoint, -planeNormal);
				if (t >= 0.f && t <= 1.f)
					out.push_back(v0 + t * (v1 - v0));
				else
					out.push_back(v0);
			}
		}

		start = end;
		dot_start = dot_end;
	}
}

static glm::vec3 ProjectPointToPlane(const glm::vec3& point, const glm::vec3& n, const glm::vec3& planePoint)
{
	return point - glm::dot(n, point - planePoint) * n;
}

static void ReduceContactPoints(std::vector<glm::vec3>& points, const glm::vec3& normal)
{
	glm::vec3 center(0.f, 0.f, 0.f);
	float maxD = 0;
	size_t a;
	for (size_t i = 0; i < points.size(); ++i)
	{
		center += points[i];
		for (size_t j = i + 1; j < points.size(); ++j)
		{
			float d = glm::length2(points[i] - points[j]);
			if (d > maxD)
			{
				a = i;
				maxD = d;
			}
		}
	}
	center /= (float)(points.size());

	glm::vec3 u = glm::normalize(points[a] - center);
	glm::vec3 v = glm::normalize(glm::cross(u, normal));

	float minU = FLT_MAX;
	float maxU = -FLT_MAX;
	float minV = FLT_MAX;
	float maxV = -FLT_MAX;

	size_t maxUi, minUi, maxVi, minVi;
	for (int i = 0; i < points.size(); ++i)
	{
		float dotU = glm::dot(points[i], u);
		float dotV = glm::dot(points[i], v);

		if (dotU > maxU)
		{
			maxUi = i;
			maxU = dotU;
		}

		if (dotU < minU)
		{
			minUi = i;
			minU = dotU;
		}

		if (dotV > maxV)
		{
			maxVi = i;
			maxV = dotV;
		}

		if (dotV < minV)
		{
			minVi = i;
			minV = dotV;
		}
	}

	points = { points[maxUi], points[maxVi], points[minUi], points[minVi] };
}

static bool CreateFaceContact(const glm::vec3& sepNormal, bool flip, const std::vector<size_t>& face, std::shared_ptr<Collider> colA, std::shared_ptr<Collider> colB,
	std::vector<ContactPoint>& cp)
{
	auto reference = flip ? colB : colA;
	auto incident = flip ? colA : colB;

	//Find Incident Edge
	std::vector<std::pair<std::vector<size_t>, glm::vec3>> incidentFaces;
	if (incident->m_type == BoundingType::CONVEX)
		incidentFaces = std::static_pointer_cast<ConvexCollider>(incident)->m_faces;

	std::vector<glm::vec3> incidentVertices;
	if (incident->m_type == BoundingType::CONVEX)
		incidentVertices = std::static_pointer_cast<ConvexCollider>(incident)->m_vertices;

	std::vector<glm::vec3> referenceVertices;
	if (reference->m_type == BoundingType::CONVEX)
		referenceVertices = std::static_pointer_cast<ConvexCollider>(reference)->m_vertices;

	size_t incidentFaceIndex = FindMostAntiParallelFace(incident, sepNormal);
	auto incidentFace = incidentFaces[incidentFaceIndex];

	glm::vec3 normalWorld = sepNormal;
	std::vector<glm::vec3> verticesTemp1;
	std::vector<glm::vec3> verticesTemp2;
	for (size_t i = 0; i < incidentFace.first.size(); ++i)
	{
		glm::vec3 faceVertIncident = incidentVertices[incidentFace.first[i]];
		verticesTemp1.push_back(faceVertIncident);
	}

	size_t curr_index = 0;
	size_t number = 0;
	glm::vec3 edgeV1 = referenceVertices[face[curr_index]];
	bool vertice1Input = false;

	do {

		vertice1Input = !vertice1Input;

		++curr_index;
		if (curr_index == face.size())
			curr_index = 0;

		glm::vec3 edgeV2 = referenceVertices[face[curr_index]];
		glm::vec3 edgeDirection = glm::normalize(edgeV2 - edgeV1);

		glm::vec3 planeNormal = glm::cross(sepNormal, edgeDirection);

		if (vertice1Input)
			ClipPolygonWithPlane(verticesTemp1, edgeV1, planeNormal, verticesTemp2);
		else
			ClipPolygonWithPlane(verticesTemp2, edgeV1, planeNormal, verticesTemp1);

		edgeV1 = edgeV2;

		if (vertice1Input)
		{
			verticesTemp1.clear();
			number = verticesTemp2.size();
		}
		else
		{
			verticesTemp2.clear();
			number = verticesTemp1.size();
		}

	} while (curr_index != 0 && number > 0);

	std::vector<glm::vec3>& clippedPoints = vertice1Input ? verticesTemp2 : verticesTemp1;
	
	//Reduce contact points
	if (clippedPoints.size() > 4)
		ReduceContactPoints(clippedPoints, sepNormal);

	glm::vec3 referenceFaceVert = glm::vec3(glm::vec4(referenceVertices[face[0]], 1.f));
	bool found = false;
	for (size_t i = 0; i < clippedPoints.size(); ++i)
	{
		glm::vec3 clippedInWorld = clippedPoints[i];
		float penetration = glm::dot((referenceFaceVert - clippedInWorld), sepNormal);

		if (penetration > 0.f)
		{
			found = true;

			glm::vec3 contactPointIncident = clippedPoints[i];
			glm::vec3 contactPointReference = ProjectPointToPlane(clippedPoints[i], sepNormal, referenceFaceVert);
			ContactPoint c;
			c.contactNormal = normalWorld;
			c.contactPointA = contactPointReference;
			c.contactPointB = contactPointIncident; 
			c.penetrationDepth = penetration;
			cp.push_back(c);
		}
	}

	return found;
}

static void FindClosestPoint(const glm::vec3& s,
	const SupportVector& v0, const SupportVector& v1, const SupportVector& v2,
	float d,
	glm::vec3& contactPointA, glm::vec3& contactPointB)
{
	glm::vec3 support_d = s * d;
	glm::vec3 a = v1.support - v0.support;
	glm::vec3 b = v2.support - v0.support;
	glm::vec3 c = support_d - v0.support;
	float dot_aa = glm::dot(a, a);
	float dot_ab = glm::dot(a, b);
	float dot_bb = glm::dot(b, b);
	float dot_ca = glm::dot(c, a);
	float dot_cb = glm::dot(c, b);
	float denom = 1.0f / (dot_aa * dot_bb - dot_ab * dot_ab);
	float proj_b = (dot_bb * dot_ca - dot_ab * dot_cb) * denom;
	float proj_c = (dot_aa * dot_cb - dot_ab * dot_ca) * denom;
	float proj_a = 1.0f - proj_b - proj_c;

	contactPointA = (proj_a * v0.supportA) + (proj_b * v1.supportA) + (proj_c * v2.supportA);
	contactPointB = (proj_a * v0.supportB) + (proj_b * v1.supportB) + (proj_c * v2.supportB);
}
