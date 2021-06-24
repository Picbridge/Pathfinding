#pragma once
#include "Misc/PathfindingDetails.hpp"
struct Node
{
	float cost;
	float given;
	GridPos parentPos;
	GridPos position;
	//bool isOnOpenList;
	bool operator == (const Node& rhs)
	{
		if (this->position == rhs.position)
		{
			return true;
		}
		else
		{
			return false;
		}
	}
	bool operator != (const Node& rhs)
	{
		if (!(this->position == rhs.position))
		{
			return true;
		}
		else
		{
			return false;
		}
	}
};

using NodeList = std::vector<Node>;
using Vec3List = std::vector<Vec3>;
class AStarPather
{
public:
    /* 
        The class should be default constructible, so you may need to define a constructor.
        If needed, you can modify the framework where the class is constructed in the
        initialize functions of ProjectTwo and ProjectThree.
    */

    /* ************************************************** */
    // DO NOT MODIFY THESE SIGNATURES
    bool initialize();
    void shutdown();
    PathResult compute_path(PathRequest &request);
    /* ************************************************** */

    /*
        You should create whatever functions, variables, or classes you need.
        It doesn't all need to be in this header and cpp, structure it whatever way
        makes sense to you.
    */
	Node* AStarPather::FindNode(NodeList& list, Node& node)
	{
		unsigned i = 0;
		for (auto nodes : list)
		{
			if (nodes.position == node.position)
			{
				return &list.at(i);
			}
			i++;
		}
		return nullptr;
	}

	float AStarPather::HvalueFinder(PathRequest request,GridPos from,GridPos to)
	{
		float Hval;
		//each diff values
		float posX = static_cast<float>(abs(from.col - to.col));
		float posZ = static_cast<float>(abs(from.row - to.row));

		if (request.settings.heuristic == Heuristic::MANHATTAN)
		{
			Hval = posX + posZ;
		}
		else if (request.settings.heuristic == Heuristic::EUCLIDEAN)
		{
			Hval = (sqrt(posX*posX + posZ*posZ));
		}
		else if (request.settings.heuristic == Heuristic::CHEBYSHEV)
		{//max diff
			Hval = std::max(posX, posZ);
		}
		else if (request.settings.heuristic == Heuristic::OCTILE)
		{//min diff * sqrt(2) + max diff - min diff
			Hval = std::min(posX, posZ) * sqrt(2.f) + (std::max(posX, posZ) - std::min(posX, posZ));
		}

		Hval *= request.settings.weight;

		return Hval;
	}

	static Vec3 AStarPather::CatmullRom(const Vec3& v1, const Vec3& v2, const Vec3& v3, const Vec3& v4, float t)
	{
		Vec3 output = v1 * (-0.5f*t*t*t + t * t - 0.5f*t) +
					  v2 * (1.5f * t*t*t + -2.5f*t*t + 1.0f) +
					  v3 * (-1.5f*t*t*t + 2.0f*t*t + 0.5f*t) +
					  v4 * (0.5f*t*t*t - 0.5f*t*t);

		return output;
	}
	bool hasInitializedOnce = false;
	NodeList openList;
	NodeList closedList;
};