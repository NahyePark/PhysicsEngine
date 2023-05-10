#include "RigidBody.h"

void RigidBody::Initialize()
{
	if(m_collider->m_type == BoundingType::BOX)
		std::static_pointer_cast<OBBCollider>(m_collider)->OBBResetCollider(glm::toMat4(m_collider->m_rotation));
	else if (m_collider->m_type == BoundingType::CONVEX)
		std::static_pointer_cast<ConvexCollider>(m_collider)->ConvexResetCollider(glm::toMat4(m_collider->m_rotation));
	else if (m_collider->m_type == BoundingType::SPHERE)
		std::static_pointer_cast<SphereCollider>(m_collider)->SphereResetCollider(glm::toMat4(m_collider->m_rotation));
	
	//cube
	float r = m_collider->m_scale.x * 2.f;
	float m = 1.0f / m_inverseMass;
	if (m_collider->m_type == BoundingType::CONVEX)
	{
		//calculate inertia tensor
		float mul = 1.0f / m_inverseMass * r * r;
		// Intertia tensors
		m_inertiaTensor = glm::mat4(
			0.66f, -0.25f, -0.25f, 0.0f,
			-0.25f, 0.66f, -0.25f, 0.0f,
			-0.25f, -0.25f, 0.66f, 0.0f,
			0.0f,    0.0f,   0.0f, 1.0f
		);
		m_inertiaTensor *= mul;
		m_inertiaTensor[3][3] = 1.0f;
		SetInverseInertiaTensor(glm::inverse(m_inertiaTensor));
	}
	else if (m_collider->m_type == BoundingType::SPHERE)
	{
		// Intertia tensors
		m_inertiaTensor = glm::mat4(
			0.4f * m * r * r, 0.0f, 0.0f, 0.0f,
			0.0f, 0.4f * m * r * r, 0.0f, 0.0f,
			0.0f, 0.0f, 0.4f * m * r * r, 0.0f,
			0.0f, 0.0f, 0.0f, 1.0f
		);
		SetInverseInertiaTensor(glm::inverse(m_inertiaTensor));
	}
}

bool RigidBody::IsDynamic()
{
	return m_isDynamic;
}

void RigidBody::SetDynamic(bool dynamic)
{
	m_isDynamic = dynamic;
	if (!m_isDynamic)
	{
		m_inverseMass = 0.f;
		m_inverseInertiaTensor = glm::mat4(0, 0, 0, 0, 
											0, 0, 0, 0, 
											0, 0, 0, 0, 
											0, 0, 0, 1);
		m_inertiaTensor = glm::mat4(0, 0, 0, 0,
									0, 0, 0, 0,
									0, 0, 0, 0,
									0, 0, 0, 1);
	}
	else
		m_inverseMass = 1.f;
}

void RigidBody::AddForce(const glm::vec3& force)
{
	m_netForce += force;
}

void RigidBody::AddForceL(float x, float y, float z)
{
	m_netForce += glm::vec3(x,y,z);
}

void RigidBody::AddTorque(const glm::vec3& force)
{
	m_netTorque += force;
}

void RigidBody::ApplyGravity()
{
	AddForce(m_gravityForce * m_inverseMass);
}

void RigidBody::SetGravityForce(const glm::vec3& g)
{
	m_gravityForce = g;
}

void RigidBody::SetNetForce(const glm::vec3& nf)
{
	m_netForce = nf;
}

void RigidBody::SetNetTorque(const glm::vec3& nt)
{
	m_netTorque = nt;
}

void RigidBody::SetBounciness(const float& b)
{
	m_bounciness = b;
}

void RigidBody::SetMass(const float& m)
{
	m_inverseMass = m;
}

void RigidBody::SetGravity(bool g)
{
	m_gravity = g;
}

void RigidBody::SetVelocity(const glm::vec3 vel)
{
	m_velocity = vel;
}

void RigidBody::SetAngularVelocity(const glm::vec3 ang_vel)
{
	m_angularVelocity = ang_vel;
}

void RigidBody::SetInverseInertiaTensor(const glm::mat4 inertiaTensor)
{
	m_inverseInertiaTensor = inertiaTensor;
}



void RigidBody::SetRotation(const glm::vec3& angualrVelocity)
{
	glm::quat rot = m_collider->m_rotation;
	glm::quat mul(-angualrVelocity.x * rot.x - angualrVelocity.y * rot.y - angualrVelocity.z * rot.z,
		angualrVelocity.x * rot.w + angualrVelocity.y * rot.z - angualrVelocity.z * rot.y,
		angualrVelocity.y * rot.w + angualrVelocity.z * rot.x - angualrVelocity.x * rot.z,
		angualrVelocity.z * rot.w + angualrVelocity.x * rot.y - angualrVelocity.y * rot.x
		);
	rot = rot + mul * 0.5f;
	if (glm::length2(rot) > DBL_EPSILON)
		rot = glm::normalize(rot);

	if (glm::length2(rot) > DBL_EPSILON)
		m_collider->m_rotation = rot;

	SetInverseInertiaTensor(glm::inverse(m_inertiaTensor));
}

void RigidBody::Rotate(float dt)
{
	glm::quat rot = m_collider->m_rotation;
	glm::vec3 a = m_angularVelocity;
	glm::quat mul(-a.x * rot.x - a.y * rot.y - a.z * rot.z,
		a.x * rot.w + a.y * rot.z - a.z * rot.y,
		a.y * rot.w + a.z * rot.x - a.x * rot.z,
		a.z * rot.w + a.x * rot.y - a.y * rot.x
	);
	rot = rot + mul * 0.5f * dt;
	if (glm::length2(rot) > DBL_EPSILON * DBL_EPSILON)
		rot = glm::normalize(rot);

	if (glm::length2(rot) > DBL_EPSILON * DBL_EPSILON)
		m_collider->m_rotation = rot;

	SetInverseInertiaTensor(glm::inverse( m_inertiaTensor));

}

glm::vec3& RigidBody::Velocity()
{
	return m_velocity;
}

glm::vec3& RigidBody::AngularVelocity()
{
	return m_angularVelocity;
}

glm::vec3& RigidBody::NetForce()
{
	return m_netForce;
}

glm::vec3& RigidBody::NetTorque()
{
	return m_netTorque;
}

glm::mat4& RigidBody::GetInverseIntertiaTensor()
{
	return m_inverseInertiaTensor;
}

float RigidBody::GetBounciness()
{
	return m_bounciness;
}

const glm::vec3& RigidBody::GetCenterOfMass()
{
	return m_centerOfMass;
}

float RigidBody::GetInverseMass()
{
	return m_inverseMass;
}

bool RigidBody::TakesGravity()
{
	return m_gravity;
}

void RigidBody::Reset()
{
	m_velocity = { 0.0f, 0.0f, 0.0f };
	m_angularVelocity = { 0.0f, 0.0f, 0.0f };
	m_gravityForce = { 0.0f, 0.0f, 0.0f };
	m_netForce = { 0.0f, 0.0f, 0.0f };
	m_netTorque = { 0.0f, 0.0f, 0.0f };

	if (m_isDynamic)
	{
		m_gravity = true;
		//UpdateInverseInertiaTensor();
	}
}
