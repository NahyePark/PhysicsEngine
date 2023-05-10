#include "BVH.h"
#include <iostream>


AABBDynamicTree::~AABBDynamicTree()
{
}

void AABBDynamicTree::Insert(RigidBody* data)
{
	
	int leaf = AllocateNode();

	nodes[leaf]->m_box.m_shape = aabbBox;
	nodes[leaf]->m_box.m_lower = data->m_collider->m_aabb.m_lower - extent;
	nodes[leaf]->m_box.m_upper = data->m_collider->m_aabb.m_upper + extent;
	nodes[leaf]->m_left = -1;
	nodes[leaf]->m_right = -1;
	nodes[leaf]->m_clientData = data;
	nodes[leaf]->m_hieght = 0;

	//If the tree is empty
	if (rootIndex == -1)
	{
		rootIndex = leaf;
		nodes[rootIndex]->m_parent = -1;
		return;
	}

	//1 : Find best sibling
	AABB leafAABB = nodes[leaf]->m_box;
	int sibling = rootIndex;
	while (!nodes[sibling]->IsLeaf())
	{
		int left = nodes[sibling]->m_left;
		int right = nodes[sibling]->m_right;
		
		AABB combined = Union(nodes[sibling]->m_box, leafAABB);
		float combinedArea = Area(combined);
		float cost = 2.0f * combinedArea;
		float inheritCost = 2.0f * (combinedArea - Area(nodes[sibling]->m_box));

		float leftCost = inheritCost;
		if (nodes[left]->IsLeaf()) //isLeaf
			leftCost += Area(Union(nodes[left]->m_box, leafAABB));
		else
			leftCost += Area(Union(nodes[left]->m_box, leafAABB)) - Area(nodes[left]->m_box);

		float rightCost = inheritCost;
		if (nodes[right]->IsLeaf())//isLeaf
			rightCost += Area(Union(nodes[right]->m_box, leafAABB));
		else
			rightCost += Area(Union(nodes[right]->m_box, leafAABB)) - Area(nodes[right]->m_box);

		if (cost < leftCost && cost < rightCost) //minimum cost
			break;

		if (leftCost < rightCost)
			sibling = left;
		else
			sibling = right;
	}

	//2 : Create a new parent
	int oldParent = nodes[sibling]->m_parent;

	int newParent = AllocateNode();

	nodes[newParent]->m_parent = oldParent;
	nodes[newParent]->m_clientData = nullptr;
	nodes[newParent]->m_box = Union(leafAABB, nodes[sibling]->m_box);
	nodes[newParent]->m_hieght = nodes[sibling]->m_hieght + 1;

	if (oldParent != -1)
	{
		if (nodes[oldParent]->m_left == sibling)
			nodes[oldParent]->m_left = newParent;
		else
			nodes[oldParent]->m_right = newParent;

		nodes[newParent]->m_left = sibling;
		nodes[newParent]->m_right = leaf;
		nodes[sibling]->m_parent = newParent;
		nodes[leaf]->m_parent = newParent;
	}
	else
	{
		nodes[newParent]->m_left = sibling;
		nodes[newParent]->m_right = leaf;
		nodes[sibling]->m_parent = newParent;
		nodes[leaf]->m_parent = newParent;
		rootIndex = newParent;
	}

	//3 : Walk back up the tree refitting AABBs
	int index = nodes[leaf]->m_parent;
	while (index != -1)
	{
		index = Balance(index);
		int left = nodes[index]->m_left;
		int right = nodes[index]->m_right;

		nodes[index]->m_hieght = std::max(nodes[left]->m_hieght, nodes[right]->m_hieght) + 1;
		nodes[index]->m_box = Union(nodes[left]->m_box, nodes[right]->m_box);

		index = nodes[index]->m_parent;
	}

}

void AABBDynamicTree::Remove(int index)
{
	if (index == rootIndex)
	{
		rootIndex = -1;
		FreeNode(index);
		return;
	}

	int parent = nodes[index]->m_parent;
	int grandParent = nodes[parent]->m_parent;
	int sibling;
	if (nodes[parent]->m_left == index)
		sibling = nodes[parent]->m_right;
	else
		sibling = nodes[parent]->m_left;

	if (grandParent == -1)
	{
		rootIndex = sibling;
		nodes[sibling]->m_parent = -1;
		FreeNode(parent);
	}
	else
	{
		if (nodes[grandParent]->m_left == parent)
			nodes[grandParent]->m_left = sibling;
		else
			nodes[grandParent]->m_right = sibling;

		nodes[sibling]->m_parent = grandParent;
		FreeNode(parent);

		int i = grandParent;
		while (i != -1)
		{
			i = Balance(i);
			int left = nodes[i]->m_left;
			int right = nodes[i]->m_right;

			nodes[i]->m_box = Union(nodes[left]->m_box, nodes[right]->m_box);
			nodes[i]->m_hieght = 1 + std::max(nodes[left]->m_hieght, nodes[right]->m_hieght);

			i = nodes[i]->m_parent;
		}
	}

	FreeNode(index);
}

void AABBDynamicTree::Update()
{
	std::queue<int> q;
	std::vector<std::pair<int, RigidBody*>> datas;

	if (rootIndex != -1)
		q.push(rootIndex);

	while (!q.empty())
	{
		int currIndex = q.front();
		q.pop();

		if (nodes[currIndex]->m_left != -1)
			q.push(nodes[currIndex]->m_left);
		if (nodes[currIndex]->m_right != -1)
			q.push(nodes[currIndex]->m_right);


		if (nodes[currIndex]->IsLeaf() && !nodes[currIndex]->m_box.Contains(nodes[currIndex]->m_clientData->m_collider->m_aabb))
			datas.emplace_back(std::make_pair(currIndex, nodes[currIndex]->m_clientData));
	}

	for (int i = 0; i < datas.size(); ++i)
		Remove(datas[i].first);

	for (int i = 0; i < datas.size(); ++i)
		Insert(datas[i].second);

}

int AABBDynamicTree::Balance(int index)
{
	//         A
	//      /     \
	//     B       C
	//   /   \    /  \
	//  D     E  F    G

	std::shared_ptr<Node> A = nodes[index];
	if (A->IsLeaf() || A->m_hieght < 2)
		return index;

	std::shared_ptr<Node> B = nodes[A->m_left];
	std::shared_ptr<Node> C = nodes[A->m_right];

	int balance = C->m_hieght - B->m_hieght;
	int a = index;
	int b = A->m_left;
	int c = A->m_right;

	// If right subtree height is bigger, rotate C up
	if (balance > 1)
	{
		std::shared_ptr<Node> F = nodes[C->m_left];
		std::shared_ptr<Node> G = nodes[C->m_right];
		int f = C->m_left;
		int g = C->m_right;

		// A <-> C
		C->m_left = a;
		C->m_parent = A->m_parent;
		A->m_parent = c;

		//correct the parent's children
		if (C->m_parent == -1)
			rootIndex = c;
		else
		{
			if (nodes[C->m_parent]->m_left == a)
				nodes[C->m_parent]->m_left = c;
			else
				nodes[C->m_parent]->m_right = c;
		}

		//rotate
		if (F->m_hieght > G->m_hieght)
		{
			C->m_right = f;
			A->m_right = g;
			G->m_parent = a;
			A->m_box = Union(B->m_box, G->m_box);
			C->m_box = Union(A->m_box, F->m_box);

			A->m_hieght = std::max(B->m_hieght, G->m_hieght) + 1;
			C->m_hieght = std::max(A->m_hieght, F->m_hieght) + 1;
		}
		else
		{
			C->m_right = g;
			A->m_right = f;
			F->m_parent = a;
			A->m_box = Union(B->m_box, F->m_box);
			C->m_box = Union(A->m_box, G->m_box);

			A->m_hieght = std::max(B->m_hieght, F->m_hieght) + 1;
			C->m_hieght = std::max(A->m_hieght, G->m_hieght) + 1;
		}
		
		return c;
	}

	// If left subtree height is bigger, rotate B up
	if (balance < -1)
	{
		int d = B->m_left;
		int e = B->m_right;
		std::shared_ptr<Node> D = nodes[d];
		std::shared_ptr<Node> E = nodes[e];

		// A <-> B
		B->m_left = a;
		B->m_parent = A->m_parent;
		A->m_parent = b;

		if (B->m_parent == -1)
			rootIndex = b;
		else
		{
			if (nodes[B->m_parent]->m_left == a)
				nodes[B->m_parent]->m_left = b;
			else
				nodes[B->m_parent]->m_right = b;
		}

		//rotate
		if (D->m_hieght > E->m_hieght)
		{
			B->m_right = d;
			A->m_left = e;
			E->m_parent = a;
			A->m_box = Union(C->m_box, E->m_box);
			B->m_box = Union(A->m_box, D->m_box);

			A->m_hieght = std::max(C->m_hieght, E->m_hieght) + 1;
			B->m_hieght = std::max(A->m_hieght, D->m_hieght) + 1;
		}
		else
		{
			B->m_right = e;
			A->m_left = d;
			D->m_parent = a;
			A->m_box = Union(C->m_box, D->m_box);
			B->m_box = Union(A->m_box, E->m_box);

			A->m_hieght = std::max(C->m_hieght, D->m_hieght) + 1;
			B->m_hieght = std::max(A->m_hieght, E->m_hieght) + 1;
		}

		return b;

	}

	return a;
}

void AABBDynamicTree::Draw(int programId)
{
	std::queue<int> q;
	if (rootIndex != -1)
		q.push(rootIndex);

	while (!q.empty())
	{
		int curr = q.front();
		q.pop();

		if (nodes[curr]->m_left != -1)
			q.push(nodes[curr]->m_left);
		if (nodes[curr]->m_right != -1)
			q.push(nodes[curr]->m_right);

		nodes[curr]->m_box.Draw(programId);
	}
}

int AABBDynamicTree::FindIndex(RigidBody* data)
{
	std::queue<int> q;
	if (rootIndex != -1)
		q.push(rootIndex);

	while (!q.empty())
	{
		int curr = q.front();
		q.pop();

		if (nodes[curr]->m_clientData == data)
			return curr;

		if (nodes[curr]->m_left != -1)
			q.push(nodes[curr]->m_left);
		if (nodes[curr]->m_right != -1)
			q.push(nodes[curr]->m_right);

	}

	return -1;
}

int AABBDynamicTree::AllocateNode()
{
	int leaf;
	if (freeList.empty())
	{
		//fatten aabb
		std::shared_ptr<Node> newNode = std::make_shared<Node>();
		nodes.emplace_back(newNode);
		leaf = nodes.size() - 1;
	}
	else
	{
		leaf = freeList.front();
		freeList.pop();
	}

	return leaf;
}

void AABBDynamicTree::FreeNode(int index)
{
	nodes[index]->m_clientData = nullptr;
	nodes[index]->m_hieght = 0;
	nodes[index]->m_left = -1;
	nodes[index]->m_right = -1;
	nodes[index]->m_parent = -1;
	freeList.push(index);
}
