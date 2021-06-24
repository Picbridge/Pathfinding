#include <pch.h>
#include "Projects/ProjectTwo.h"
#include "P2_Pathfinding.h"

#pragma region Extra Credit
bool ProjectTwo::implemented_floyd_warshall()
{
    return false;
}

bool ProjectTwo::implemented_goal_bounding()
{
    return false;
}

bool ProjectTwo::implemented_jps_plus()
{
    return false;
}
#pragma endregion

bool AStarPather::initialize()
{
    // handle any one-time setup requirements you have

    /*
        If you want to do any map-preprocessing, you'll need to listen
        for the map change message.  It'll look something like this:

        Callback cb = std::bind(&AStarPather::your_function_name, this);
        Messenger::listen_for_message(Messages::MAP_CHANGE, cb);

        There are other alternatives to using std::bind, so feel free to mix it up.
        Callback is just a typedef for std::function<void(void)>, so any std::invoke'able
        object that std::function can wrap will suffice.
    */

    return true; // return false if any errors actually occur, to stop engine initialization
}

void AStarPather::shutdown()
{
    /*
        Free any dynamically allocated memory or any other general house-
        keeping you need to do during shutdown.
    */
}

PathResult AStarPather::compute_path(PathRequest &request)
{
    /*
        This is where you handle pathing requests, each request has several fields:

        start/goal - start and goal world positions
        path - where you will build the path upon completion, path should be
            start to goal, not goal to start
        heuristic - which heuristic calculation to use
        weight - the heuristic weight to be applied
        newRequest - whether this is the first request for this path, should generally
            be true, unless single step is on

        smoothing - whether to apply smoothing to the path
        rubberBanding - whether to apply rubber banding
        singleStep - whether to perform only a single A* step
        debugColoring - whether to color the grid based on the A* state:
            closed list nodes - yellow
            open list nodes - blue

            use terrain->set_color(row, col, Colors::YourColor);
            also it can be helpful to temporarily use other colors for specific states
            when you are testing your algorithms

        method - which algorithm to use: A*, Floyd-Warshall, JPS+, or goal bounding,
            will be A* generally, unless you implement extra credit features

        The return values are:
            PROCESSING - a path hasn't been found yet, should only be returned in
                single step mode until a path is found
            COMPLETE - a path to the goal was found and has been built in request.path
            IMPOSSIBLE - a path from start to goal does not exist, do not add start position to path
    */

    // WRITE YOUR CODE HERE
	GridPos start = terrain->get_grid_position(request.start);
	GridPos goal = terrain->get_grid_position(request.goal);
	GridPos findParent;
	if (request.settings.debugColoring)
	{
		terrain->set_color(start, Colors::Orange);
		terrain->set_color(goal, Colors::Orange);
	}

	int directionX[] = { 1, 0,-1, 0, 1, 1,-1,-1 };
	int directionY[] = { 0,-1, 0, 1, 1,-1,-1, 1 };
	//request.path.push_back(request.start);
	Node startNode;
	Node* findOpen;
	Node* findClose;
	request.path.clear();
	//float Hval;
	if ((request.settings.singleStep && !hasInitializedOnce) || 
		(!request.settings.singleStep || request.newRequest == true))
	{
		openList.clear();
		closedList.clear();
		startNode.parentPos = start;
		startNode.given = 0;
		startNode.position = start;
		startNode.cost = HvalueFinder(request,goal,start);	

		openList.push_back(startNode);
		hasInitializedOnce = true;
	}
	//right,down,left,up
	bool checkWall[] = {false,false,false,false};

	while (!openList.empty())
	{
		
		//Pop cheapest node off Openlist(parent node)
		Node current;
		current = openList.front();

		for (auto node : openList)
		{
			if (node.cost <= current.cost)
			{
				current = node;
			}
		}
		openList.erase(std::find(openList.begin(),openList.end(), current));

		if (current.position == goal)
		{ 
			closedList.push_back(current);
			std::reverse(closedList.begin(),closedList.end());
			findParent = closedList.front().parentPos;
			if ((!request.settings.rubberBanding&&!request.settings.smoothing))
			{
				request.path.push_front(terrain->get_world_position(closedList.front().position));
				for (auto node : closedList)
				{
					if (findParent == node.position)
					{
						findParent = node.parentPos;
						if(request.settings.debugColoring)
						terrain->set_color(node.position, Colors::Yellow);
						request.path.push_front(terrain->get_world_position(node.position));
					}
				}
			}
			else
			{
				NodeList reversedList;
				if (!closedList.empty())
					reversedList.push_back(closedList.front());
				for (auto node : closedList)
				{
					if (findParent == node.position)
					{
						findParent = node.parentPos;
						reversedList.push_back(node);
					}
				}
				if (request.settings.rubberBanding)
				{	

					//arrange all nodes in order
					int findAllNodes = 0;
					bool isWallFound = false;
					while (findAllNodes < (reversedList.size() - 2))
					{					//Push 3 from end
						int maxRow = std::max(reversedList.at(findAllNodes).position.row, reversedList.at(findAllNodes + 2).position.row);
						int minRow = std::min(reversedList.at(findAllNodes).position.row, reversedList.at(findAllNodes + 2).position.row);
						int maxCol = std::max(reversedList.at(findAllNodes).position.col, reversedList.at(findAllNodes + 2).position.col);
						int minCol = std::min(reversedList.at(findAllNodes).position.col, reversedList.at(findAllNodes + 2).position.col);

						if (minRow != maxRow)
						{
							for (int i = minRow; i <= maxRow; i++)
							{
								if (minCol != maxCol)
								{
									for (int j = minCol; j <= maxCol; j++)
									{
										//if(any wall in bounding box)
										//{move all nodes to next}
										if (terrain->is_valid_grid_position(i, j) &&
											terrain->is_wall(i, j))
										{
											isWallFound = true;
											break;
										}

									}
									if (isWallFound)
									{
										findAllNodes++;
										break;
									}
								}
								else
								{
									if (terrain->is_valid_grid_position(i, maxCol) &&
										terrain->is_wall(i, maxCol))
									{
										isWallFound = true;
										findAllNodes++;
										break;
									}
								}
							}
						}
						else if (minCol != maxCol)
						{

							for (int j = minCol; j <= maxCol; j++)
							{
								//if(any wall in bounding box)
								//{move all nodes to next}
								if (terrain->is_valid_grid_position(maxRow, j) &&
									terrain->is_wall(maxRow, j))
								{
									isWallFound = true;
									findAllNodes++;
									break;
								}

							}
						}
						if (isWallFound == false)
						{
							//else{pop out the 2nd node and put next node in}
							reversedList.erase(reversedList.begin() + (findAllNodes + 1));//std::find(reversedList.begin(), reversedList.end(),reversedList.at(foundAllNodes+1)));
						}
						isWallFound = false;
					}
					if (!request.settings.smoothing)
					{
						for (auto node : reversedList)
						{
							if (request.settings.debugColoring)
							terrain->set_color(node.position, Colors::Yellow);
							request.path.push_front(terrain->get_world_position(node.position));
						}
					}
					else if (request.settings.smoothing)
					{
						Vec3 Pos;
						WaypointList newWay;

						for (auto node : reversedList)
						{
							if (request.settings.debugColoring)
							terrain->set_color(node.position, Colors::Yellow);
							newWay.push_front(terrain->get_world_position(node.position));
						}

						Vec3 insertNode = newWay.front();
						auto pathFront = newWay.begin();
						auto pathBack = newWay.begin();
						if(newWay.size() > 1)
							std::advance(pathBack, 1);
						else
							pathBack = newWay.end();
						while (pathBack != newWay.end())//i < newWay.size()-1)
						{

							/*posX = static_cast<float>(abs(reversedList[i].position.col - reversedList[i+1].position.col));
							posZ = static_cast<float>(abs(reversedList[i].position.row - reversedList[i+1].position.row));*/
							Pos.x = ((pathBack->x - pathFront->x));
							Pos.z = ((pathBack->z - pathFront->z));
							if ((sqrt(Pos.x*Pos.x + Pos.z * Pos.z)) > 5.5f)
							{
								insertNode.x = pathFront->x + Pos.x / 2.f;
								insertNode.z = pathFront->z + Pos.z /2.f;
								
								//reversedList.emplace(reversedList[i], insertNode);
								newWay.insert(pathBack,insertNode);
								std::advance(pathBack, -1);
							}
							else
							{
								std::advance(pathFront, 1);
								std::advance(pathBack,	1);
							}
						}

						
						Vec3List rubberedPath;
						
						for (auto node : newWay)
						{
							//newNode.position = terrain->get_grid_position(node);
							rubberedPath.push_back(node);
							//reversedList.push_back(newNode);
						}

						Vec3List points;
						for (auto node : rubberedPath)
						{
							{//put in nodes from starting point
								points.push_back(node);
							}
						}
						WaypointList finalPath;

						Vec3 v1 = points[0];
						Vec3 v2 = points[0];
						Vec3 v3;
						Vec3 v4;
						if (points.size() >= 2)
							v3 = points[1];
						else
							v3 = points.back();
						if (points.size() >= 3)
							v4 = points[2];
						else
							v4 = points.back();

						float t = 0;
						for (unsigned i = 0; i < points.size() - 1; i++)
						{
							Vec3 input;

							for (t = 0; t < 1.f; t += 0.25f)
							{
								input = CatmullRom(v1, v2, v3, v4, t);
								if (terrain->is_valid_grid_position(terrain->get_grid_position(input).row, terrain->get_grid_position(input).col) &&
									!terrain->is_wall(terrain->get_grid_position(input).row, terrain->get_grid_position(input).col))
								{
									finalPath.push_front(input);
								}
							}

							if (v1 == v2)
							{
								if (v2 != points.back())
									v2 = points[i + 1];
								if (v3 != points.back())
									v3 = points[i + 2];
								if (v4 != points.back())
									v4 = points[i + 3];
							}
							else
							{
								if (v1 != points.back())
									v1 = points[i];
								if (v2 != points.back())
									v2 = points[i + 1];
								if (v3 != points.back())
									v3 = points[i + 2];
								if (v4 != points.back())
									v4 = points[i + 3];
							}
						}
						for (auto node : finalPath)
						{
							if (request.settings.debugColoring)
							terrain->set_color(terrain->get_grid_position(node), Colors::Yellow);
							request.path.push_front(node);
						}

					}
				}
				else if (request.settings.smoothing&&!request.settings.rubberBanding)
				{
					NodeList points;
					WaypointList newWay;
					for (auto node:reversedList)
					{//put in nodes from starting point
						points.push_back(node);
					}
					std::reverse(points.begin(), points.end());

					Vec3 v1 = terrain->get_world_position(points[0].position);
					Vec3 v2 = terrain->get_world_position(points[0].position);
					Vec3 v3;
					Vec3 v4;
					if(points.size() >= 2)
						v3 = terrain->get_world_position(points[1].position);
					else
						v3 = terrain->get_world_position(points.back().position);
					if (points.size() >= 3)
						v4 = terrain->get_world_position(points[2].position);
					else
						v4 = terrain->get_world_position(points.back().position);

					float t = 0;
					for (unsigned i = 0; i < points.size()-1; i++)
					{
						Vec3 input;
						
						for (t = 0; t < 1.f; t += 0.25f)
						{
							input = CatmullRom(v1, v2, v3, v4, t);
							if (terrain->is_valid_grid_position(terrain->get_grid_position(input).row, terrain->get_grid_position(input).col) &&
								!terrain->is_wall(terrain->get_grid_position(input).row, terrain->get_grid_position(input).col))
							{
								newWay.push_front(input);
							}
						}

						if (v1 == v2)
						{
							if(v2 != terrain->get_world_position(points.back().position))
							v2 = terrain->get_world_position(points[i + 1].position);
							if (v3 != terrain->get_world_position(points.back().position))
							v3 = terrain->get_world_position(points[i + 2].position);
							if (v4 != terrain->get_world_position(points.back().position))
							v4 = terrain->get_world_position(points[i + 3].position);
						}
						else 
						{
							if (v1 != terrain->get_world_position(points.back().position))
								v1 = terrain->get_world_position(points[i].position);
							if (v2 != terrain->get_world_position(points.back().position))
								v2 = terrain->get_world_position(points[i + 1].position);
							if (v3 != terrain->get_world_position(points.back().position))
								v3 = terrain->get_world_position(points[i + 2].position);
							if (v4 != terrain->get_world_position(points.back().position))
								v4 = terrain->get_world_position(points[i + 3].position);
						}
					}
					for (auto node : newWay)
					{
						terrain->set_color(terrain->get_grid_position(node), Colors::Yellow);
						request.path.push_front(node);
					}
					
				}

			}
			hasInitializedOnce = false;
			return PathResult::COMPLETE; 
		}

		//for(int i;i<neighboring childs;i++){
		//compute the cost, f(x) = g(x)+h(x)
		//if(childNode neither in Open or Closed list && isNotWall){Openlist.push_back(childNode);}
		//if(childNode either in Open or Closed list && newNode is cheaper){Openlist.pop(childNode); Closedlist.pop(childeNode); Openlist.push_back(newNode);}
		//}
		for (int i = 0; i < 8; i++)
		{
			Node neighbor = current;

			neighbor.position.col += directionX[i];
			neighbor.position.row += directionY[i];

			neighbor.parentPos = current.position;


			if (i < 4)
			{
				if (terrain->is_valid_grid_position(neighbor.position.row, neighbor.position.col)&&
					terrain->is_wall(neighbor.position.row, neighbor.position.col))
					checkWall[i] = true;

				neighbor.given += 1.f*1.5f;// *5.f;
			}
			else
			{
				neighbor.given += sqrt(2.f)*1.5f;// *5.f;
			}
			float Hval = (HvalueFinder(request, goal, neighbor.position));
			neighbor.cost = neighbor.given + Hval;

			findOpen = FindNode(openList, neighbor);
			findClose = FindNode(closedList, neighbor);
				if (findOpen == nullptr && findClose == nullptr && 
					terrain->is_valid_grid_position(neighbor.position.row, neighbor.position.col) &&
					!terrain->is_wall(neighbor.position.row, neighbor.position.col))
				{
					if (i < 4)
					{
						openList.push_back(neighbor);
						if (request.settings.debugColoring)
						terrain->set_color(neighbor.position, Colors::Blue);
					}
					switch (i)
					{
					case 4:
						//check up,right wall
						if (!(checkWall[0] || checkWall[3]))
						{
							openList.push_back(neighbor);
							if (request.settings.debugColoring)
							terrain->set_color(neighbor.position, Colors::Blue);
						}
						break;
					case 5:
						//check down,right wall
						if (!(checkWall[0] || checkWall[1]))
						{
							openList.push_back(neighbor);
							if (request.settings.debugColoring)
							terrain->set_color(neighbor.position, Colors::Blue);
						}
						break;
					case 6:
						//check down,left wall
						if (!(checkWall[1] || checkWall[2]))
						{
							openList.push_back(neighbor);
							if (request.settings.debugColoring)
							terrain->set_color(neighbor.position, Colors::Blue);
						}
						break;
					case 7:
						//check up,left wall
						if (!(checkWall[2] || checkWall[3]))
						{
							openList.push_back(neighbor);
							if (request.settings.debugColoring)
							terrain->set_color(neighbor.position, Colors::Blue);
						}
						break;
					default:
						break;
					}
				}
				else if (findOpen != nullptr && neighbor.cost < findOpen->cost)
				{
					if (i < 4)
					{
						openList.erase(std::find(openList.begin(), openList.end(), *findOpen));
						openList.push_back(neighbor);
						if (request.settings.debugColoring)
						terrain->set_color(neighbor.position, Colors::Blue);
					}
					switch (i)
					{
					case 4:
						//check up,right wall
						if (!(checkWall[0] || checkWall[3]))
						{
							openList.erase(std::find(openList.begin(), openList.end(), *findOpen));
							openList.push_back(neighbor);
							if (request.settings.debugColoring)
							terrain->set_color(neighbor.position, Colors::Blue);
						}
						break;
					case 5:
						//check down,right wall
						if (!(checkWall[0] || checkWall[1]))
						{
							openList.erase(std::find(openList.begin(), openList.end(), *findOpen));
							openList.push_back(neighbor);
							if (request.settings.debugColoring)
							terrain->set_color(neighbor.position, Colors::Blue);
						}
						break;
					case 6:
						//check down,left wall
						if (!(checkWall[1] || checkWall[2]))
						{
							openList.erase(std::find(openList.begin(), openList.end(), *findOpen));
							openList.push_back(neighbor);
							if (request.settings.debugColoring)
							terrain->set_color(neighbor.position, Colors::Blue);
						}
						break;
					case 7:
						//check up,left wall
						if (!(checkWall[2] || checkWall[3]))
						{
							openList.erase(std::find(openList.begin(), openList.end(), *findOpen));
							openList.push_back(neighbor);
							if (request.settings.debugColoring)
							terrain->set_color(neighbor.position, Colors::Blue);
						}
						break;
					default:
						break;
					}
					//openList.erase(std::find(openList.begin(), openList.end(), *findOpen));
					//openList.push_back(neighbor);
					//terrain->set_color(neighbor.position, Colors::Blue);
				}
				else if(findClose != nullptr && neighbor.cost < findClose->cost)
				{
				if (i < 4)
				{
					closedList.erase(std::find(closedList.begin(), closedList.end(), *findClose));
					openList.push_back(neighbor);
					if (request.settings.debugColoring)
					terrain->set_color(neighbor.position, Colors::Blue);
				}
				switch (i)
				{
				case 4:
					//check up,right wall
					if (!(checkWall[0] || checkWall[3]))
					{
						closedList.erase(std::find(closedList.begin(), closedList.end(), *findClose));
						openList.push_back(neighbor);
						if (request.settings.debugColoring)
						terrain->set_color(neighbor.position, Colors::Blue);
					}
					break;
				case 5:
					//check down,right wall
					if (!(checkWall[0] || checkWall[1]))
					{
						closedList.erase(std::find(closedList.begin(), closedList.end(), *findClose));
						openList.push_back(neighbor);
						if (request.settings.debugColoring)
						terrain->set_color(neighbor.position, Colors::Blue);
					}
					break;
				case 6:
					//check down,left wall
					if (!(checkWall[1] || checkWall[2]))
					{
						closedList.erase(std::find(closedList.begin(), closedList.end(), *findClose));
						openList.push_back(neighbor);
						if (request.settings.debugColoring)
						terrain->set_color(neighbor.position, Colors::Blue);
					}
					break;
				case 7:
					//check up,left wall
					if (!(checkWall[2] || checkWall[3]))
					{
						closedList.erase(std::find(closedList.begin(), closedList.end(), *findClose));
						openList.push_back(neighbor);
						if (request.settings.debugColoring)
						terrain->set_color(neighbor.position, Colors::Blue);
					}
					break;
				default:
					break;
				}
					//closedList.erase(std::find(closedList.begin(), closedList.end(), *findClose));
					//openList.push_back(neighbor);
					//terrain->set_color(neighbor.position, Colors::Blue);
				}

		}
		//Place parentNode on te Closed list
		closedList.push_back(current);
		if (request.settings.debugColoring)
		terrain->set_color(current.position, Colors::Yellow);
		checkWall[0] = false;
		checkWall[1] = false;
		checkWall[2] = false;
		checkWall[3] = false;
		//if(time took too much for this frame){return PathResult::PROCESSING;}
		if (request.settings.singleStep)
		{
			return PathResult::PROCESSING;
		}
	}
	hasInitializedOnce = false;
	return PathResult::IMPOSSIBLE;
	
    //// Just sample code, safe to delete
    //GridPos start = terrain->get_grid_position(request.start);
    //GridPos goal = terrain->get_grid_position(request.goal);
    //terrain->set_color(start, Colors::Orange);
    //terrain->set_color(goal, Colors::Orange);
    //request.path.push_back(request.start);
    //request.path.push_back(request.goal);
    //return PathResult::COMPLETE;
}
