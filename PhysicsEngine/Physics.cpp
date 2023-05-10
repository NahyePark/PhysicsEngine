#include <thread>
#include <iostream>
#include "Physics.h"

Physics::Physics()
{
	m_DynamicPhysicsObjects.clear();
	m_StaticPhysicsObjects.clear();
	tree = new AABBDynamicTree(new Box());
}

void Physics::Init()
{
	m_CollisionQueue.clear();
	
}

void Physics::Update(float dt)
{
	// Do dynamic updates
	Integrate(dt);

	// Then solve for collisions
	DetectCollisions(dt);
	SolveCollisions(dt);
}

void Physics::AddPhysicsObject(Object* obj)
{
	if (obj == nullptr)
		return;

	if (obj->rigidbody.IsDynamic())
		m_DynamicPhysicsObjects.emplace_back(obj);
	else if (!obj->rigidbody.IsDynamic())
		m_StaticPhysicsObjects.emplace_back(obj);
	else
		return;

	tree->Insert(&obj->rigidbody);
}

void Physics::RemovePhysicsObject(Object* obj)
{
	if (obj == nullptr)
		return;

	// First search the vec of dynamic objects;
	auto it = std::find(m_DynamicPhysicsObjects.begin(), m_DynamicPhysicsObjects.end(), obj);
	if (it == m_DynamicPhysicsObjects.end())
	{
		// if not found, search in vec of static objects;
		it = std::find(m_StaticPhysicsObjects.begin(), m_StaticPhysicsObjects.end(), obj);
		if (it == m_StaticPhysicsObjects.end())
			return;
		else
			m_StaticPhysicsObjects.erase(it);
	}
	else
		m_DynamicPhysicsObjects.erase(it);

	int leafIndex = tree->FindIndex(&obj->rigidbody);
	tree->Remove(leafIndex);
}

void Physics::RemoveAllDynamicObjects()
{
	for (int i = m_DynamicPhysicsObjects.size() -1 ; i >= 0; --i)
		RemovePhysicsObject(m_DynamicPhysicsObjects[i]);
}

std::vector<Object*>& Physics::GetDynamicPhysicsObjects()
{
	return m_DynamicPhysicsObjects;
}

std::vector<Object*>& Physics::GetStaticPhysicsObjects()
{
	return m_StaticPhysicsObjects;
}

void Physics::DetectCollisionThread(int parent)
{
	std::vector<int> dynamicObjs;
	std::vector<int> staticObjs;

	std::queue<int> q;
	if (parent != -1)
		q.push(parent);

	while (!q.empty())
	{
		int curr = q.front();
		q.pop();

		if (tree->nodes[curr]->m_left != -1)
			q.push(tree->nodes[curr]->m_left);
		if (tree->nodes[curr]->m_right != -1)
			q.push(tree->nodes[curr]->m_right);

		if (tree->nodes[curr]->IsLeaf())
		{
			if (tree->nodes[curr]->m_clientData->IsDynamic())
				dynamicObjs.emplace_back(curr);
			else
				staticObjs.emplace_back(curr);
		}
	}

	for (unsigned int i = 0; i < dynamicObjs.size(); ++i)
	{
		for (unsigned int j = i+1; j < dynamicObjs.size(); ++j)
		{
			if (i == j) continue;

			if (!intersectAABB(tree->nodes[dynamicObjs[i]]->m_box, tree->nodes[dynamicObjs[j]]->m_box))
				continue;

			RigidBody* rbA = tree->nodes[dynamicObjs[i]]->m_clientData;
			RigidBody* rbB = tree->nodes[dynamicObjs[j]]->m_clientData;
			if (!Colliding(rbA, rbB))
				continue;

			tree->nodes[dynamicObjs[i]]->m_box.isColliding = true;
			tree->nodes[dynamicObjs[j]]->m_box.isColliding = true;
			rbA->m_collider->m_color = glm::vec3(1, 0, 0);
			rbB->m_collider->m_color = glm::vec3(1, 0, 0);
		}

		for (unsigned int j = 0; j < staticObjs.size(); ++j)
		{
			if (!intersectAABB(tree->nodes[dynamicObjs[i]]->m_box, tree->nodes[staticObjs[j]]->m_box))
				continue;

			RigidBody* rbA = tree->nodes[dynamicObjs[i]]->m_clientData;
			RigidBody* rbB = tree->nodes[staticObjs[j]]->m_clientData;
			if (!Colliding(rbA, rbB))
				continue;

			tree->nodes[dynamicObjs[i]]->m_box.isColliding = true;
			tree->nodes[staticObjs[j]]->m_box.isColliding = true;
			rbA->m_collider->m_color = glm::vec3(1, 0, 0);
			rbB->m_collider->m_color = glm::vec3(1, 0, 0);
		}
	}
}

bool Physics::Colliding(RigidBody* rbA, RigidBody* rbB)
{
	ContactPoint col;
	if (intersect(rbA, rbB, col, m_CollisionQueue))
		return true;

	return false;
}

void Physics::DetectCollisions(float dt)
{
	//tree traversal
	std::queue<int> q;
	if (tree->rootIndex != -1)
		q.push(tree->rootIndex);

	while (!q.empty())
	{
		int curr = q.front();
		q.pop();

		int left = tree->nodes[curr]->m_left;
		int right = tree->nodes[curr]->m_right;

		if (left == -1 || right == -1)
			continue;

		if (intersectAABB(tree->nodes[left]->m_box, tree->nodes[right]->m_box))
		{
			DetectCollisionThread(curr);
			tree->nodes[curr]->m_box.isColliding = true;
		}
		else
		{
			tree->nodes[curr]->m_box.isColliding = false;
			q.push(left);
			q.push(right);
		}
	}
}

void Physics::InitializeConstraints()
{
	for (size_t i = 0; i < m_CollisionQueue.size(); ++i)
	{
		auto curr = m_CollisionQueue[i];
		auto colA = curr->a->m_collider;
		auto colB = curr->b->m_collider;

		glm::vec3 vA = curr->a->Velocity();
		glm::vec3 vB = curr->b->Velocity();
		glm::vec3 wA = curr->a->AngularVelocity();
		glm::vec3 wB = curr->b->AngularVelocity();

		glm::mat4 iIA = curr->a->GetInverseIntertiaTensor();
		glm::mat4 iIB = curr->b->GetInverseIntertiaTensor();

		float iMA = curr->a->GetInverseMass();
		float iMB = curr->b->GetInverseMass();

		for (size_t j = 0; j < curr->contactPoints.size(); ++j)
		{
			auto& contact = curr->contactPoints[j];
			glm::vec3 rA = contact.contactPointA - colA->m_position;
			glm::vec3 rB = contact.contactPointB - colB->m_position;

			// Kn = 1/m1 + 1/m2 + [I1^-1 (r1 x n) x r + I2*-1 (r2 x n) x r2] * n
			glm::vec3 K = glm::vec3(iIA * glm::vec4(glm::cross(glm::cross(rA, contact.contactNormal), rA), 1.f)
				+ iIB * glm::vec4(glm::cross(glm::cross(rB, contact.contactNormal), rB), 1.f));
			float Kn = iMA + iMB + glm::dot(K, contact.contactNormal);
			contact.normalMass = (Kn > 0.f) ? (1.0f / Kn) : 0.f;

			contact.velocityBias = 0.f;
			//relative velocity = v2 + w2 x r2 - v1 - w1 x r1
			float rVel = glm::dot(contact.contactNormal, vB + glm::cross(wB, rB) - vA - glm::cross(wA, rA));
			if (rVel < -0.5f)
				contact.velocityBias = -contact.restitution * rVel;

			///Debug drawing
			debugDraw->contactIndex.push_back(debugDraw->contactPoints.size());
			debugDraw->contactPoints.push_back(glm::vec4(contact.contactPointA, 1.f));
			debugDraw->contactIndex.push_back(debugDraw->contactPoints.size());
			debugDraw->contactPoints.push_back(glm::vec4(contact.contactPointB, 1.f));
		}

	}
}

void Physics::WarmStart()
{
	for (size_t i = 0; i < m_CollisionQueue.size(); ++i)
	{
		auto curr = m_CollisionQueue[i];

		glm::vec3& vA = curr->a->Velocity();
		glm::vec3& vB = curr->b->Velocity();
		glm::vec3& wA = curr->a->AngularVelocity();
		glm::vec3& wB = curr->b->AngularVelocity();

		glm::mat4 iIA = curr->a->GetInverseIntertiaTensor();
		glm::mat4 iIB = curr->b->GetInverseIntertiaTensor();

		float iMA = curr->a->GetInverseMass();
		float iMB = curr->b->GetInverseMass();

		for (size_t j = 0; j < curr->contactPoints.size(); ++j)
		{
			auto& contact = curr->contactPoints[j];
			glm::vec3 rA = contact.contactPointA - curr->a->m_collider->m_position;
			glm::vec3 rB = contact.contactPointB - curr->b->m_collider->m_position;

			if (contact.isResting)
			{
				glm::vec3 P = contact.normalImpulse * contact.contactNormal;
				wA -= glm::vec3(iIA * glm::vec4(glm::cross(rA, P), 1.f));
				vA -= iMA * P;
				wB += glm::vec3(iIB * glm::vec4(glm::cross(rB, P), 1.f));
				vB += iMB * P;
			}
		}
	}
}

void Physics::SolveCollisions(float dt)
{
	if(!m_CollisionQueue.empty())
	{
		InitializeConstraints();
		WarmStart();

		for (int j = 0; j < m_velocitySolveIt; ++j)
		{
			for (int i = 0; i < m_CollisionQueue.size(); ++i)
			{
				auto col = m_CollisionQueue[i];
				if (col->collided)
					SolveVelocityConstraint(col, dt);

			}
		}

		//Integrate the position
		for (int j = 0; j < m_CollisionQueue.size(); ++j)
		{
			auto col = m_CollisionQueue[j];
			glm::quat newRotA, newRotB;
			if (col->a->IsDynamic())
				col->a->Rotate(dt);
			if (col->b->IsDynamic())
				col->b->Rotate(dt);
			
			col->a->m_collider->m_position += dt * col->a->Velocity();
			col->b->m_collider->m_position += dt * col->b->Velocity();
			col->a->m_collider->UpdateMatrix();
			col->b->m_collider->UpdateMatrix();
		}

		m_CollisionQueue.clear();
	}

}

//semi-implicit Euler (Symplectic Euler)
void Physics::Integrate(float dt)
{
	for (auto obj : m_DynamicPhysicsObjects)
	{
		RigidBody& rb = obj->rigidbody;
		rb.m_collider->m_color = glm::vec3(0, 0, 1);
		rb.m_collider->m_prevPos = rb.m_collider->m_position;
		rb.m_collider->m_prevRot = rb.m_collider->m_rotation;

		if (!rb.IsDynamic())
			continue;

		rb.SetGravityForce(m_Gravity);

		if (m_EnableGravity && rb.TakesGravity())
			rb.ApplyGravity();

		rb.Velocity() += dt * rb.NetForce() * rb.GetInverseMass();
		rb.AngularVelocity() += dt * rb.NetTorque();

		glm::quat newRot = 0.5f * dt * glm::quat(0.f, rb.AngularVelocity());
		rb.m_collider->m_position += dt * rb.Velocity();
		rb.Rotate(dt);

		// Clear force
		rb.SetNetForce({ 0.0f,0.0f,0.0f });
		rb.SetNetTorque({ 0.0f,0.0f,0.0f });

		rb.m_collider->UpdateMatrix();

		if (rb.m_collider->m_type == BoundingType::BOX)
			std::static_pointer_cast<OBBCollider>(rb.m_collider)->OBBUpdate();
		else if (rb.m_collider->m_type == BoundingType::CONVEX)
			std::static_pointer_cast<ConvexCollider>(rb.m_collider)->ConvexUpdate();
		else if (rb.m_collider->m_type == BoundingType::SPHERE)
			std::static_pointer_cast<SphereCollider>(rb.m_collider)->SphereUpdate();
		rb.m_collider->UpdateAABB();
	}

	for (auto obj : m_StaticPhysicsObjects)
	{
		RigidBody& rb = obj->rigidbody;
		if (rb.m_collider->m_type == BoundingType::BOX)
			std::static_pointer_cast<OBBCollider>(rb.m_collider)->OBBUpdate();
		else if (rb.m_collider->m_type == BoundingType::CONVEX)
			std::static_pointer_cast<ConvexCollider>(rb.m_collider)->ConvexUpdate();
		else if (rb.m_collider->m_type == BoundingType::SPHERE)
			std::static_pointer_cast<SphereCollider>(rb.m_collider)->SphereUpdate();
		rb.m_collider->UpdateAABB();
	}

	tree->Update();

}

void Physics::SolveVelocityConstraint(std::shared_ptr<CollisionData> col, float dt)
{
	auto colA = col->a->m_collider;
	auto colB = col->b->m_collider;

	float iMA = col->a->GetInverseMass();
	float iMB = col->b->GetInverseMass();

	glm::mat4 iIA = col->a->GetInverseIntertiaTensor();
	glm::mat4 iIB = col->b->GetInverseIntertiaTensor();

	glm::vec3 vA = col->a->Velocity();
	glm::vec3 vB = col->b->Velocity();
	glm::vec3 wA = col->a->AngularVelocity();
	glm::vec3 wB = col->b->AngularVelocity();

	float sumImpulse = 0.f;

	for (size_t i = 0; i < col->contactPoints.size(); ++i)
	{	
		auto& contact = col->contactPoints[i];

		glm::vec3 rA = contact.contactPointA - colA->m_position;
		glm::vec3 rB = contact.contactPointB - colB->m_position;

		glm::vec3 vDelta = vB + glm::cross(wB, rB) - vA - glm::cross(wA, rA);
		float dotDN = glm::dot(vDelta, contact.contactNormal);

		float biasImpulse = 0.f;
		if (contact.penetrationDepth > 0.005f) //slop = 0.005
			biasImpulse = (debugDraw->biasFactor / dt) * std::max(0.f, contact.penetrationDepth - 0.005f);
		float b = biasImpulse + contact.velocityBias;

		//current delta impulse
		float lambda = -(dotDN - b) * contact.normalMass;
		float newImpulse = std::max(contact.normalImpulse + lambda, 0.f);;
		lambda = newImpulse - contact.normalImpulse;
		contact.normalImpulse = newImpulse;

		glm::vec3 P = lambda * contact.contactNormal;
		vA -= iMA * P;
		wA -= glm::vec3(iIA * glm::vec4(glm::cross(rA, P), 1.f));
		vB += iMB * P;
		wB += glm::vec3(iIB * glm::vec4(glm::cross(rB, P), 1.f));

		sumImpulse += contact.normalImpulse;
	}

	col->a->Velocity() = vA;
	col->a->AngularVelocity() = wA;
	col->b->Velocity() = vB;
	col->b->AngularVelocity() = wB;
}

/// @brief Clears Phys Obj Vectors
void Physics::ClearVectors()
{
	m_DynamicPhysicsObjects.clear();
	m_StaticPhysicsObjects.clear();
	ClearCollisionQueue();
}

void Physics::ClearCollisionQueue()
{
	std::swap(m_CollisionQueue, std::vector<std::shared_ptr<CollisionData>>());
}

void Physics::RemoveStressObjects()
{
	for (int i = m_DynamicPhysicsObjects.size() - 1; i >= 0; --i)
	{
		if (m_DynamicPhysicsObjects[i]->name == "stressObject")
		{
			int index = tree->FindIndex(&m_DynamicPhysicsObjects[i]->rigidbody);
			tree->Remove(index);

			m_DynamicPhysicsObjects.erase(m_DynamicPhysicsObjects.begin() + i);
		}
	}

	g_Physics->tree->Update();

}
