#pragma once

#include "Contact.h"

void CollisionData::InsertContactPoint(const ContactPoint& cp)
{
	size_t shallow = -1;
	float minimumPenetration = FLT_MAX;
	for (size_t i = 0; i < contactPoints.size(); ++i)
	{
		if (contactPoints[i].penetrationDepth < minimumPenetration)
		{
			shallow = i;
			minimumPenetration = contactPoints[i].penetrationDepth;
		}

		//if already exist
		if (contactPoints[i].contactPointA == cp.contactPointA && contactPoints[i].contactPointB == cp.contactPointB)
		{
			contactPoints[i].isResting = true;
			return;
		}

		//If it's too close to old point, reject it
		if (glm::distance(contactPoints[i].contactPointA, cp.contactPointA) < 0.001f
			&& glm::distance(contactPoints[i].contactPointB, cp.contactPointB) < 0.001f)
			return;
	}

	// keep deepest one
	if (pointCount >= 4)
	{
		contactPoints[shallow] = cp;
		pointCount = 4;
	}
	else
	{
		contactPoints.emplace_back(cp);
		++pointCount;
	}
}

void CollisionData::ChangeContactPoint(const ContactPoint& cp)
{
	/////When the number of contact point is 0
	for (size_t i = 0; i < contactPoints.size(); ++i)
	{
		//if already exist
		if (contactPoints[i].contactPointA == cp.contactPointA && contactPoints[i].contactPointB == cp.contactPointB)
		{
			contactPoints[i].isResting = true;
			return;
		}
	}
	
	contactPoints.clear();
	contactPoints.push_back(cp);
	pointCount++;


}

