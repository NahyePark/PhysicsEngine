#pragma once

#include "CollisionDetection.h"
#include "BVH.h"


class Physics
{
public:
	/// @brief Default Contructor for physics space class
	Physics();


	/// @brief Intitialize physics world
	// Set up inverse inertia matrices for spheres and boxes
	void Init();


	/// @brief Physics update loop
	/// @param dt - Delta time
	void Update(float dt);



	/// @brief Helper to add game object that is affected by physics
	/// @param obj - Physics Game Object
	void AddPhysicsObject(Object* obj);


	/// @brief Remove a gameobject from the physics world
	/// @param obj - Physics Game Object
	void RemovePhysicsObject(Object* obj);
	void RemoveAllDynamicObjects();

	/// @brief Clears Phys Obj Vectors
	void ClearVectors();
	void ClearCollisionQueue();

	void RemoveStressObjects();

	/// @brief Getter to get the list of all dynamic physics game objects
	/// @return Vector of of all dynamic physics game objects
	std::vector<Object*>& GetDynamicPhysicsObjects();



	/// @brief Getter to get the list of all static physics game objects
	/// @return Vector of of all static physics game objects
	std::vector<Object*>& GetStaticPhysicsObjects();

	void InitializeConstraints();
	void WarmStart();

	/// @brief Getter to get the list of all softbody physics game objects
	/// @return Vector of of all softbody physics game objects
	//std::vector<std::shared_ptr<Object>>& GetSoftBodyPhysicsObjects();

	AABBDynamicTree* tree;
	bool m_EnableGravity = true;
	int m_velocitySolveIt = 20;
	int m_positionSolveIt = 10;

protected:
	/// @brief GOs with a RB comp. (May or may not have a Collider comp.)
	std::vector<Object*> m_DynamicPhysicsObjects;

	/// @brief GOs with just a Collider comp.
	std::vector<Object*> m_StaticPhysicsObjects;

	/// @brief GOs with just a Collider comp.
	//std::vector<std::shared_ptr<Object>> m_SoftBodyPhysicsObjects;

	/// @brief Queue of all collisions detected in this frame
	std::vector<std::shared_ptr<CollisionData>> m_CollisionQueue;

	///// @brief Queue of all collisions detected in this frame
	//std::vector<CollisionData> m_TriggerQueue;

private:

	void DetectCollisionThread(int parent);

	/// @brief /// Check if two bodies are colliding.
	/// If so, add to collision queue
	/// @param colA - RigidBody of first colliding object
	/// @param colB - RigidBody of second colliding object
	bool Colliding(RigidBody* rbA, RigidBody* rbB);



	/// @brief Detect collision in the current frame
	/// @param dt - Delta time
	void DetectCollisions(float dt);



	/// @brief Resolve impenetration and velocity for this collision
	/// @param colData - Collision data needed for resolution
	/// @param dt - Delta time
	//void Resolve(CollisionData& colData, float dt);



	/// @brief Resolve velocities by adding impulse
	/// @param colData - Collision data:
	//void ResolveVelocity(CollisionData& colData);



	/// @brief Resolve impenetration
	/// @param colData - Collision related data: 
	/// Contact normal, penetration depth
	//void ResolveImpenetration(CollisionData& colData);



	/// @brief Resolve all collisions queued up
	/// @param dt - Delta time
	void SolveCollisions(float dt);


	/// @brief Euler intergation
	/// @param dt - Delta time
	void Integrate(float dt);

	void SolveVelocityConstraint(std::shared_ptr<CollisionData> col, float dt);


	/// @brief World gravity
	glm::vec3 m_Gravity{ 0.f,0.f,-9.8f };

	

};

extern std::unique_ptr<Physics> g_Physics;
