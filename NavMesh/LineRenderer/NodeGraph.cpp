#include "NodeGraph.h"

#include <algorithm>
#include <iostream>

#include "LineRenderer.h"
#include "Utility.h"
#include "NavigationMesh.h"
#include "DelaunayTriangulation.h"
#include "Triangle.h"

NodeGraph::NodeGraph(NavigationMesh* navMesh) : mNavMesh(navMesh)
{
	ConstructNodeNeighbours(mNavMesh->GetTriangles());
}

Node* NodeGraph::GetNodeAt(int index) const
{
	if (mNodes.empty()) { return nullptr; }
	return mNodes[index];
}

void NodeGraph::ConstructNodeNeighbours(std::vector<Triangle*>& triangleList)
{
	mNodes.clear();
	
	for (int i = 0; i < (int)triangleList.size(); i++)
	{
		Node* node = new Node(GetTriangleCentre(triangleList[i]), triangleList[i]);
		mNodes.push_back(node);
	}
	
	for(int i = 0; i < (int)mNodes.size(); i++)
	{
		std::vector<Vec2> edgeList[3];
		edgeList[0] = { triangleList[i]->mPoints[0], triangleList[i]->mPoints[1] };
		edgeList[1] = { triangleList[i]->mPoints[1], triangleList[i]->mPoints[2] };
		edgeList[2] = { triangleList[i]->mPoints[2], triangleList[i]->mPoints[0] };
	
		for (std::vector<Vec2> e : edgeList)
		{
			Triangle* adjacentTriangle = FindAdjacentTriangleToEdge(i, e, triangleList);
			if (adjacentTriangle == nullptr) { continue; }
	
			Node* connectionNode = GetNode(GetTriangleCentre(adjacentTriangle));
			if (connectionNode == nullptr) { continue; }
	
			float nodeCost = (mNodes[i]->mPosition - connectionNode->mPosition).GetMagnitude();
			mNodes[i]->ConnectToNode(connectionNode, nodeCost);
		}
	}
}


Node* NodeGraph::GetClosestNode(Vec2 pos)
{
	//float closestDistance = FLT_MAX;
	//std::vector<Triangle> triList = mNavMesh->GetTriangleList();
	//int triIndex = 0;
	//
	//for (int i = 0; i < triList.size(); i++)
	//{
	//	float result = (GetTriangleCentre(triList[i]) - pos).GetMagnitude();
	//	if (result < closestDistance)
	//	{
	//		closestDistance = result;
	//		triIndex = i;
	//	}
	//}
	//
	//return GetNodeAt(triIndex);
	return nullptr;
}

Node* NodeGraph::GetNode(Vec2 pos) 
{
	//for(Node* n : mNodes)
	//{
	//	if(n->mPosition == pos)
	//	{
	//		return n;
	//	}
	//}
	return nullptr;
}
