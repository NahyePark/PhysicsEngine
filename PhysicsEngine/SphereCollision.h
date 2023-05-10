#pragma once

#include "Contact.h"
#include "RigidBody.h"

static bool SphereSphereCollision(RigidBody* a, RigidBody* b)
{
	auto colA = std::static_pointer_cast<SphereCollider>(a->m_collider);
	auto colB = std::static_pointer_cast<SphereCollider>(b->m_collider);
	if (glm::length(colA->m_position - colB->m_position) <= colA->m_radius + colB->m_radius)
		return true;

	return false;
}

static void SphereSphereContactPoint(RigidBody* a, RigidBody* b,
									 std::vector<ContactPoint>& c)
{
	auto colA = std::static_pointer_cast<SphereCollider>(a->m_collider);
	auto colB = std::static_pointer_cast<SphereCollider>(b->m_collider);

	ContactPoint cp;
	cp.contactNormal = -glm::normalize(colA->m_position - colB->m_position);
	glm::vec3 surfaceA = colA->m_position + cp.contactNormal * colA->m_radius;
	glm::vec3 surfaceB = colB->m_position - cp.contactNormal * colB->m_radius;
	cp.penetrationDepth = colA->m_radius + colB->m_radius - glm::length(colA->m_position - colB->m_position);
	cp.contactPointA = (surfaceA + surfaceB) * 0.5f;
	cp.contactPointB = cp.contactPointA;
	c.push_back(cp);

}