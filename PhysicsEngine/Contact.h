#pragma once
#include "glm/glm.hpp"
#include <vector>

class RigidBody;

struct ContactPoint {
	glm::vec3 contactPointA;
	glm::vec3 contactPointB;
	glm::vec3 contactNormal;
	float penetrationDepth;
	float restitution = 0.3;

	float normalImpulse = 0.f;
	float velocityBias = 0.f;
	float normalMass = 0.f;
	bool isResting = false;
};

struct CollisionData
{
	RigidBody* a;
	RigidBody* b;
	std::vector<ContactPoint> contactPoints;
	int pointCount = 0;
	bool collided;

	void InsertContactPoint(const ContactPoint& cp);
	void ChangeContactPoint(const ContactPoint& cp);

};



