#pragma once

#include "Collider.h"

class RigidBody 
{
public:

	/// @brief Constructor
	/// @param parent - GameObject the component is attached to
	RigidBody(Shape* boundingShape, BoundingType boundingType)
		: m_velocity(0.0f, 0.0f, 0.0f),
		m_angularVelocity(0.0f, 0.0f, 0.0f),
		m_gravityForce(0.0f, 0.0f, 0.0f),
		m_netForce(0.0f, 0.0f, 0.0f),
		m_netTorque(0.0f, 0.0f, 0.0f),
		m_bounciness(0.5f),
		m_inverseMass(1.f),
		m_gravity(true),
		m_isDynamic(true),
		m_inverseInertiaTensor(),
		m_centerOfMass(0.f, 0.f, 0.f)
	{
		switch (boundingType)
		{
		case BoundingType::BOX:
			m_collider = std::make_shared<OBBCollider>(boundingShape);
			break;
		case BoundingType::SPHERE:
			m_collider = std::make_shared <SphereCollider>(boundingShape);
			break;
		case BoundingType::CONVEX:
			m_collider = std::make_shared <ConvexCollider>(boundingShape);
			break;
		default:
			break;
		}
	}

	void Initialize();

	/// @brief Check if rigid body is dynamic or not
	/// @return - True, if it is dynamic. False, otherwise.
	bool IsDynamic();


	/// @brief Set the state of dynamic
	/// @param dynamic - bool type for changing the state
	void SetDynamic(bool dynamic);



	/// @brief Add force to net force
	/// @param force - 3D vector force to be added
	void AddForce(const glm::vec3& force);



	/// @brief Add force to net force
	/// @param x - X component of force to be added
	/// @param y - Y component of force to be added
	/// @param z - Z component of force to be added
	void AddForceL(float x, float y, float z);



	/// @brief Add torque to net torque
	/// @param force - 3D vector force to be added
	void AddTorque(const glm::vec3& force);


	/// @brief Apply gravity using AddForce
	void ApplyGravity();

	/// @brief Set the gravity force
	/// @param g - Gravity force
	void SetGravityForce(const glm::vec3& g);



	/// @brief Set the net force
	/// @param nf - Net force
	void SetNetForce(const glm::vec3& nf);



	/// @brief Set the net toruqe force
	/// @param nt - Net torque force
	void SetNetTorque(const glm::vec3& nt);



	/// @brief Set the bounciness
	/// @param b - bounciness
	void SetBounciness(const float& b);



	/// @brief Set the mass of rigidbody
	/// @param m - mass
	void SetMass(const float& m);



	/// @brief Set the gravity will be applied on this rigidbody or not
	/// @param g - gravity
	void SetGravity(bool g);



	/// @brief Set the velocity of rigidbody
	/// @param - velocity
	void SetVelocity(const glm::vec3);



	/// @brief Set the angular velocity of rigidbody
	/// @param - angular velocity
	void SetAngularVelocity(const glm::vec3);



	/// @brief Set inertia tensor of rigidbody
	/// @param - Inertia tensor
	void SetInverseInertiaTensor(const glm::mat4);

	//void UpdateInverseInertiaTensor();
	void SetRotation(const glm::vec3& angualrVelocity);
	void Rotate(float dt);

	/// @brief Get current velocity of rigidbody
	/// @return - velocity
	glm::vec3& Velocity();



	/// @brief Get current angular velocity of rigidbody
	/// @return - angular velocity
	glm::vec3& AngularVelocity();



	/// @brief Get current net force of rigidbody
	/// @return - net force
	glm::vec3& NetForce();



	/// @brief Get current net torque of rigidbody
	/// @return - net torque
	glm::vec3& NetTorque();



	/// @brief Get inertia tensor of rigidbody
	/// @return - inertia tensor
	glm::mat4& GetInverseIntertiaTensor();


	/// @brief Get bounciness of rigidbody
	/// @return - bounciness
	float GetBounciness();

	const glm::vec3& GetCenterOfMass();

	/// @brief Get inverse mass of rigidbody
	/// @return - inverse mass
	float GetInverseMass();


	/// @brief Get the state of gravity of rigidbody
	/// @return - True if gravity is on. False, otherwise.
	bool TakesGravity();

	/// @brief Reset the variables in rigid body
	void Reset();

	std::shared_ptr<Collider> m_collider;

private:

	glm::vec3 m_centerOfMass;
	glm::vec3 m_velocity;
	glm::vec3 m_angularVelocity;
	glm::vec3 m_gravityForce;
	glm::vec3 m_netTorque;
	glm::vec3 m_netForce;
	glm::mat4 m_inverseInertiaTensor;
	glm::mat4 m_inertiaTensor;
	float m_bounciness;
	float m_inverseMass;
	bool m_gravity;
	bool m_isDynamic;

};