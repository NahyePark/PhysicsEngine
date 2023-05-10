#pragma once

#include "object.h"
#include "SAT.h"
#include "GJKEPA.h"
#include "SphereCollision.h"
#include <list>

inline int CollisionInQueueAlready(RigidBody* a, RigidBody* b
	, std::vector<std::shared_ptr<CollisionData>>& colQueue)
{
	for (int i = 0; i < colQueue.size(); i++) {
		if (colQueue[i]->a == a && colQueue[i]->b == b) {
			return i;
		}
	}

	return -1;
}


static bool intersectOBBOBB(RigidBody* a, RigidBody* b, ContactPoint& colData, std::vector<std::shared_ptr<CollisionData>>& colQueue)
{
	glm::vec2 minmax1;
	glm::vec2 minmax2;

	std::shared_ptr<OBBCollider> colA = std::static_pointer_cast<OBBCollider>(a->m_collider);
	std::shared_ptr<OBBCollider> colB = std::static_pointer_cast<OBBCollider>(b->m_collider);
	bool colliding = true;
	// local axis of a (3)
	for (int i = 0; i < colA->m_localAxis.size(); ++i)
	{
		minmax1 = SAT(colA->m_localAxis[i], a);
		minmax2 = SAT(colA->m_localAxis[i], b);
		if (!overlaps(minmax1, minmax2))
		{
			colliding = false;
			break;
		}

		for (int j = 0; j < colB->m_localAxis.size(); ++j)
		{
			glm::vec3 axis = glm::cross(colA->m_localAxis[i], colB->m_localAxis[j]);
			if (axis == glm::vec3(0, 0, 0))
				continue;

			minmax1 = SAT(axis, a);
			minmax2 = SAT(axis, b);
			if (!overlaps(minmax1, minmax2))
			{
				colliding = false;
				break;
			}
		}
	}

	// local axis of b (3)
	for (int i = 0; i < colB->m_localAxis.size(); ++i)
	{
		minmax1 = SAT(colB->m_localAxis[i], a);
		minmax2 = SAT(colB->m_localAxis[i], b);
		if (!overlaps(minmax1, minmax2))
		{
			colliding = false;
			break;
		}
	}

	int inQueue = CollisionInQueueAlready(a, b, colQueue);
	if (inQueue == -1 && !colliding) return false;
	else if (inQueue != -1 && !colliding)
		colQueue[inQueue]->collided = false; return false;

	//simple AABB collision response
	glm::vec3 posA = (a->m_collider->m_aabb.m_upper + a->m_collider->m_aabb.m_lower) * 0.5f;
	glm::vec3 posB = (b->m_collider->m_aabb.m_upper + b->m_collider->m_aabb.m_lower) * 0.5f;
	glm::vec3 scaleA = a->m_collider->m_aabb.m_upper - posA;
	glm::vec3 scaleB = b->m_collider->m_aabb.m_upper - posB;

	glm::vec3 diffPos = posA - posB;
	glm::vec3 absdiffPos = { std::abs(diffPos.x), std::abs(diffPos.y), std::abs(diffPos.z) };

	glm::vec3 penetration = scaleA + scaleB - absdiffPos;
	glm::vec3 dir = diffPos - absdiffPos;
	if (penetration.x < 0.5f)
	{
		if (std::abs(dir.x) > DBL_EPSILON)
			colData.contactNormal = { -1, 0,0 };
		else
			colData.contactNormal = { 1, 0,0 };
	}
	if (penetration.y < 0.5f)
	{
		if (std::abs(dir.y) > DBL_EPSILON)
			colData.contactNormal = { 0, -1,0 };
		else
			colData.contactNormal = { 0, 1,0 };
	}
	if (penetration.z < 0.5f)
	{
		if (std::abs(dir.z) > DBL_EPSILON)
			colData.contactNormal = { 0 , 0,-1 };
		else
			colData.contactNormal = { 0, 0,1 };
	}

	colData.contactNormal = -colData.contactNormal;
	colData.penetrationDepth = std::min(penetration.x, std::min(penetration.y, penetration.z));

	if (inQueue != -1)
	{
		colQueue[inQueue]->InsertContactPoint(colData);
		colQueue[inQueue]->collided = true;
	}
	else
	{
		auto manifold = std::make_shared<CollisionData>();
		manifold->a = a;
		manifold->b = b;
		manifold->collided = true;
		manifold->InsertContactPoint(colData);
		colQueue.push_back(manifold);
	}

	return true;

}


static bool intersectConvexSphere(RigidBody* a, RigidBody* b, ContactPoint& colData, std::vector<std::shared_ptr<CollisionData>>& colQueue)
{

	int inQueue = CollisionInQueueAlready(a, b, colQueue);
	
	std::vector<ContactPoint> cp;
	bool colliding = SATSphereConvex(a, b, cp);
	if (inQueue == -1 && !colliding)
		return false;
	else if (inQueue != -1 && !colliding)
	{
		colQueue.erase(colQueue.begin() + inQueue);
		return false;
	}

	if (inQueue != -1)
	{
		colQueue[inQueue]->contactPoints = cp;
		colQueue[inQueue]->collided = true;
	}
	else
	{
		auto col = std::make_shared<CollisionData>();
		col->a = a;
		col->b = b;
		col->collided = true;
		col->contactPoints = cp;
		colQueue.push_back(col);
	}
	
	return true;

}

static bool intersectSphereSphere(RigidBody* a, RigidBody* b, ContactPoint& colData, std::vector<std::shared_ptr<CollisionData>>& colQueue)
{
	bool colliding = SphereSphereCollision(a, b);

	int inQueue = CollisionInQueueAlready(a, b, colQueue);

	if (inQueue == -1 && !colliding)
		return false;
	else if (inQueue != -1 && !colliding)
	{
		colQueue.erase(colQueue.begin() + inQueue);
		return false;
	}

	std::vector<ContactPoint> cp;
	SphereSphereContactPoint(a, b, cp);
	

	if (inQueue != -1)
	{
		for (size_t i = 0; i < cp.size(); ++i)
			colQueue[inQueue]->InsertContactPoint(cp[i]);
		colQueue[inQueue]->collided = true;
	}
	else
	{
		auto col = std::make_shared<CollisionData>();
		col->a = a;
		col->b = b;
		col->collided = true;
		col->contactPoints = cp;
		colQueue.push_back(col);
	}


	return true;
}

static bool intersectWithConvex(RigidBody* a, RigidBody* b, ContactPoint& colData, std::vector<std::shared_ptr<CollisionData>>& colQueue)
{
	
	std::vector<ContactPoint> cp;
	bool flip;
	bool colliding = FindSparatingAxis(a, b, flip, cp);

	int inQueue = CollisionInQueueAlready(a, b, colQueue);
	
	if (inQueue == -1 && !colliding)
		return false;
	else if (inQueue != -1 && !colliding)
		return false;

	if (inQueue != -1)
	{
		for (size_t i = 0; i < cp.size(); ++i)
			colQueue[inQueue]->InsertContactPoint(cp[i]);
		colQueue[inQueue]->collided = true;
	}
	else
	{
		auto col = std::make_shared<CollisionData>();
		if (flip)
		{
			col->a = b;
			col->b = a;
		}
		else
		{
			col->a = a;
			col->b = b;
		}
		col->collided = true;

		for (size_t i = 0; i < cp.size(); ++i)
			col->InsertContactPoint(cp[i]);

		colQueue.push_back(col);
	}
	
	return true;
}


static bool intersect(RigidBody* a, RigidBody* b, ContactPoint& colData, std::vector<std::shared_ptr<CollisionData>>& colQueue)
{
	if (a->m_collider->m_type == BoundingType::CONVEX && b->m_collider->m_type == BoundingType::CONVEX)
		return intersectWithConvex(a, b, colData, colQueue);

	else if (a->m_collider->m_type == BoundingType::CONVEX && b->m_collider->m_type == BoundingType::CONVEX)
		return intersectOBBOBB(a, b, colData, colQueue);

	else if (a->m_collider->m_type == BoundingType::CONVEX && b->m_collider->m_type == BoundingType::SPHERE)
		return intersectConvexSphere(a, b, colData, colQueue);

	else if (a->m_collider->m_type == BoundingType::SPHERE && b->m_collider->m_type == BoundingType::CONVEX)
		return intersectConvexSphere(b, a, colData, colQueue);

	else if (a->m_collider->m_type == BoundingType::SPHERE && b->m_collider->m_type == BoundingType::SPHERE)
		return intersectSphereSphere(a, b, colData, colQueue);
}

inline bool intersectAABB(const AABB& a, const AABB& b)
{
	return (
		a.m_lower.x <= b.m_upper.x &&
		a.m_upper.x >= b.m_lower.x &&
		a.m_lower.y <= b.m_upper.y &&
		a.m_upper.y >= b.m_lower.y &&
		a.m_lower.z <= b.m_upper.z &&
		a.m_upper.z >= b.m_lower.z
		);
}