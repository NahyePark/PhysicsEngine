#pragma once

#include "RigidBody.h"
#include <memory>
#include <queue>

struct Node 
{
	AABB m_box;
	RigidBody* m_clientData;

	int m_left = -1;
	int m_right = -1;
	int m_parent;
	int m_hieght;

	bool IsLeaf() { return m_left == -1 && m_right == -1; }
};

class AABBDynamicTree
{
public:
	AABBDynamicTree(Shape* box) : aabbBox(box) {}
	~AABBDynamicTree();

	void Insert(RigidBody* data);
	void Remove(int index);
	void Update();
	int Balance(int index);
	void Draw(int programId);
	int FindIndex(RigidBody* data);

	AABB Union(const AABB& a, const AABB& b)
	{
		AABB c;
		c.m_shape = aabbBox;
		c.m_lower.x = std::min(std::min(a.m_lower.x, b.m_lower.x), std::min(a.m_upper.x, b.m_upper.x));
		c.m_lower.y = std::min(std::min(a.m_lower.y, b.m_lower.y), std::min(a.m_upper.y, b.m_upper.y));
		c.m_lower.z = std::min(std::min(a.m_lower.z, b.m_lower.z), std::min(a.m_upper.z, b.m_upper.z));
		c.m_upper.x = std::max(std::max(a.m_lower.x, b.m_lower.x), std::max(a.m_upper.x, b.m_upper.x));
		c.m_upper.y = std::max(std::max(a.m_lower.y, b.m_lower.y), std::max(a.m_upper.y, b.m_upper.y));
		c.m_upper.z = std::max(std::max(a.m_lower.z, b.m_lower.z), std::max(a.m_upper.z, b.m_upper.z));
		return c;
	}

	float Area(const AABB& a)
	{
		glm::vec3 diff = a.m_upper - a.m_lower;
		return (diff.x + diff.y + diff.z) * 2.0f;
	}

	std::vector<std::shared_ptr<Node>> nodes;
	int rootIndex = -1;
private:

	int AllocateNode();
	void FreeNode(int index);

	Shape* aabbBox;
	
	std::queue<int> freeList;

	float extent = 0.2f;

};


