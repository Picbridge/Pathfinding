#include <pch.h>
#include "Terrain/TerrainAnalysis.h"
#include "Terrain/MapMath.h"
#include "Agent/AStarAgent.h"
#include "Terrain/MapLayer.h"
#include "Projects/ProjectThree.h"

#include <iostream>

bool ProjectThree::implemented_fog_of_war() const // extra credit
{
    return false;
}

float distance_to_closest_wall(int row, int col)
{
    /*
        Check the euclidean distance from the given cell to every other wall cell,
        as well as the distance to the edges (they count as walls for this),
        and return the smallest distance.  Make use of the is_wall member function
        in the global terrain to determine if a cell is a wall or not.
    */

    // WRITE YOUR CODE HERE
	const int height = terrain->get_map_height();
	const int width = terrain->get_map_width();

	if (terrain->is_wall(row, col))
	{
		return -1.f;
	}

	float minDistance = FLT_MAX;
	float curDistance;
	for (int i = -1; i <= width; i++)
	{
		for (int j = -1; j <= height; j++)
		{
			if (j != col || i != row)
			{
				//if ((terrain->is_wall(i, j))
				//	&& minDistance > curDistance)
				//{
				//	minDistance = curDistance;
				//}

				float posX = static_cast<float>(abs(row - i));
				float posZ = static_cast<float>(abs(col - j));
				curDistance = (sqrt(posX*posX + posZ * posZ));


				if (terrain->is_valid_grid_position(i, j) && terrain->is_wall(i, j)&& curDistance < minDistance)
				{
					if (curDistance < minDistance)
						minDistance = curDistance;
				}
				else if (!terrain->is_valid_grid_position(i, j) && curDistance < minDistance)
				{
					if (curDistance < minDistance)
						minDistance = curDistance;
				}

			}
		}
	}

    return minDistance; // REPLACE THIS
}

bool is_clear_path(int row0, int col0, int row1, int col1)
{
    /*
        Two cells (row0, col0) and (row1, col1) are visible to each other if a line
        between their centerpoints doesn't intersect the four boundary lines of every
        wall cell.  You should puff out the four boundary lines by a very tiny amount
        so that a diagonal line passing by the corner will intersect it.  Make use of the
        is_wall member function in the global terrain to determine if a cell is a wall or not.
    */

    // WRITE YOUR CODE HERE
	const int height = terrain->get_map_height();
	const int width = terrain->get_map_width();
	const int split = 20;

	int posX = (row1 - row0);
	int posZ = (col1 - col0);
	if (posX == 0 && posZ == 0)
		return true;
	if (terrain->is_wall(row0, col0)|| terrain->is_wall(row1, col1))
		return false;

	bool dontFindLeft = false;
	bool dontFindDown = false;
	if (row0 < row1)
		dontFindLeft = true;
	if (col0 < col1)
		dontFindDown = true;
	int row;
	int col;

	for (int i = 1; i <= split; i++)
	{
		if (row0 + (posX*i / split) == row1 && col0 + (posZ*i / split) == col1)
			return true;
		for (int k = -1; k < 2; k++)
		{
			for (int l = -1; l < 2; l++)
			{
				row = row0 + (posX*i / split) + k;
				col = col0 + (posZ*i / split) + l;

				if ((row < row0 && dontFindLeft) ||
					(row0 < row && !dontFindLeft) ||
					(col < col0 && dontFindDown) ||
					(col0 < col && !dontFindDown))
					continue;

				if (row < 0 || col < 0 || row >= height || col >= width)
				{
					//out of boundary
					continue;
				}
				else
				{
					if (terrain->is_wall(row, col))
					{
						float cellWidth = 50.03f/ terrain->get_map_width();
						Vec2 wall01 = Vec2(terrain->get_world_position(row, col).x + cellWidth, terrain->get_world_position(row, col).z + cellWidth);
						Vec2 wall02 = Vec2(terrain->get_world_position(row, col).x - cellWidth, terrain->get_world_position(row, col).z - cellWidth);
						Vec2 wall11 = Vec2(terrain->get_world_position(row, col).x - cellWidth, terrain->get_world_position(row, col).z + cellWidth);
						Vec2 wall12 = Vec2(terrain->get_world_position(row, col).x + cellWidth, terrain->get_world_position(row, col).z - cellWidth);

						Vec2 og01 = Vec2(terrain->get_world_position(row0, col0).x, terrain->get_world_position(row0, col0).z);
						Vec2 og02 = Vec2(terrain->get_world_position(row1, col1).x, terrain->get_world_position(row1, col1).z);

						if (line_intersect(og01, og02, wall01, wall02) ||
							line_intersect(og01, og02, wall11, wall12))
						{
							return false;
						}
					}
				}

			}

		}


	}
	return true; // REPLACE THIS
}

void analyze_openness(MapLayer<float> &layer)
{
    /*
        Mark every cell in the given layer with the value 1 / (d * d),
        where d is the distance to the closest wall or edge.  Make use of the
        distance_to_closest_wall helper function.
    */

    // WRITE YOUR CODE HERE
	const int height = terrain->get_map_height();
	const int width = terrain->get_map_width();

	float dist;

	for (int i = 0; i < width; i++)
	{
		for (int j = 0; j < height; j++)
		{
			dist = distance_to_closest_wall(i, j);
			if(dist!=-1.f)
			layer.set_value(i, j,(1 / (dist*dist)));
		}
	}
}

void analyze_visibility(MapLayer<float> &layer)
{
    /*
        Mark every cell in the given layer with the number of cells that
        are visible to it, divided by 160 (a magic number that looks good).  Make sure
        to cap the value at 1.0 as well.

        Two cells are visible to each other if a line between their centerpoints doesn't
        intersect the four boundary lines of every wall cell.  Make use of the is_clear_path
        helper function.
    */

    // WRITE YOUR CODE HERE
	const int height = terrain->get_map_height();
	const int width = terrain->get_map_width();

	for (int i = 0; i < width; i++)
	{
		for (int j = 0; j < height; j++)
		{
			for (int k = 0; k < width; k++)
			{
				for (int l = 0; l < height; l++)
				{
					//if (k,j) can see (i,j) layer.setValue(i,j,layer.getValue(i,j) + (1.f/160.f))
					if (is_clear_path(i, j, k, l))
					{						
						layer.set_value(i,j,layer.get_value(i,j) + (1.f / 160.f));
						if (layer.get_value(i, j) > 1.f)
							layer.set_value(i, j, 1.f);
					}
				}
			}

		}
	}

}

void analyze_visble_to_cell(MapLayer<float> &layer, int row, int col)
{
    /*
        For every cell in the given layer mark it with 1.0
        if it is visible to the given cell, 0.5 if it isn't visible but is next to a visible cell,
        or 0.0 otherwise.

        Two cells are visible to each other if a line between their centerpoints doesn't
        intersect the four boundary lines of every wall cell.  Make use of the is_clear_path
        helper function.
    */

    // WRITE YOUR CODE HERE
	const int height = terrain->get_map_height();
	const int width = terrain->get_map_width();

	for (int i = 0; i < width; i++)
	{
		for (int j = 0; j < height; j++)
		{
			//if (k,j) can see (i,j) layer.setValue(i,j,layer.getValue(i,j) + (1.f/160.f))
			if (!is_clear_path(i, j, row, col))
			{
				if (layer.get_value(i, j) == 1)
				layer.set_value(i, j, 0);
				else if (layer.get_value(i, j) == 0.5)
				{
					bool isChanging = true;
					for (int k = -1; k < 2; k++)
					{
						for (int l = -1; l < 2; l++)
						{
							int row = i + k;
							int col = j + l;

							if (row < 0 || col < 0 || row >= height || col >= width)
							{
								//out of boundary
								continue;
							}
							else
							{
								if (k == 0 || l == 0)
								{
									//orthogonal
									if (layer.get_value(i, col) == 1 || layer.get_value(row, j) == 1)
									{
										isChanging = false;
									}

								}
								else if (k != 0 && l != 0)
								{
									//diagonal
									if (!terrain->is_wall(row,col)&&layer.get_value(row, col) == 1 &&
										!terrain->is_wall(i, col) && !terrain->is_wall(row, j))
									{
										isChanging = false;
									}

								}

							}

						}
					}
					if (isChanging)
					{
						layer.set_value(i, j, 0);
					}
				}
			}
			else
			{
				layer.set_value(i, j, 1);
				for (int k = -1; k < 2; k++)
				{
					for (int l = -1; l < 2; l++)
					{
						int row = i + k;
						int col = j + l;
						
						if (row < 0 || col < 0 || row >= height || col >= width)
						{
							//out of boundary
							continue;
						}
						else
						{
							if (k == 0 || l == 0)
							{
								//orthogonal
								if (layer.get_value(i, col) == 0)
								{
									layer.set_value(i, col, 0.5);
								}
								if (layer.get_value(row, j) == 0)
								{
									layer.set_value(row, j, 0.5);
								}
							}
							else if (k != 0 && l != 0)
							{
								//diagonal
								if ((!terrain->is_wall(i + k, j) || !terrain->is_wall(i, j + l))&&
									terrain->is_valid_grid_position(i + k, j + l))
								{
									if(layer.get_value(row, col) != 1)
									layer.set_value(row, col, 0.5);
								}
							}
						}

					}
				}
				
			}
		}
	}
}

void analyze_agent_vision(MapLayer<float> &layer, const Agent *agent)
{
	/*
		For every cell in the given layer that is visible to the given agent,
		mark it as 1.0, otherwise don't change the cell's current value.

		You must consider the direction the agent is facing.  All of the agent data is
		in three dimensions, but to simplify you should operate in two dimensions, the XZ plane.

		Give the agent a field of view slighter larger than 180 degrees.

		Two cells are visible to each other if a line between their centerpoints doesn't
		intersect the four boundary lines of every wall cell.  Make use of the is_clear_path
		helper function.
	*/

	// WRITE YOUR CODE HERE
	const int height = terrain->get_map_height();
	const int width = terrain->get_map_width();
	const auto pos = agent->get_position();
	const auto look = agent->get_forward_vector();

	Vec2 vP;
	Vec2 vL;
	float pMag, lMag, dot, angle, cos = 0;
	const float FOV = 181.f;
	for (int i = 0; i < height; i++)
	{
		for (int j = 0; j < width; j++)
		{
			if (layer.get_value(i, j) < 0)
			{
				layer.set_value(i, j, 0);
			}

			vP.x = terrain->get_world_position(i, j).x - pos.x;
			vP.y = terrain->get_world_position(i, j).z - pos.z;
			vL.x = look.x;
			vL.y = look.z;

			dot = (vP.x* vL.x) + (vP.y*vL.y);

			pMag = sqrt(vP.x*vP.x + vP.y * vP.y);
			lMag = sqrt(vL.x*vL.x + vL.y * vL.y);

			vP.x = vP.x / pMag;
			vP.y = vP.y / pMag;
			vL.x = vP.x / lMag;
			vL.y = vP.y / lMag;

			cos = dot / (pMag*lMag);

			if (cos == 0)
			{
				angle = 90;
			}
			else if (cos < 0)
			{
				continue;
			}
			else
			{
				angle = acos(cos);
				angle = 2 * angle * 180 / PI;
			}

			if (angle < FOV)
			{
				if (is_clear_path(terrain->get_grid_position(pos).row, terrain->get_grid_position(pos).col, i, j))
					layer.set_value(i, j, 1.f);
			}
		}
	}
}

void propagate_solo_occupancy(MapLayer<float> &layer, float decay, float growth)
{
    /*
        For every cell in the given layer:

            1) Get the value of each neighbor and apply decay factor
            2) Keep the highest value from step 1
            3) Linearly interpolate from the cell's current value to the value from step 2
               with the growing factor as a coefficient.  Make use of the lerp helper function.
            4) Store the value from step 3 in a temporary layer.
               A float[40][40] will suffice, no need to dynamically allocate or make a new MapLayer.

        After every cell has been processed into the temporary layer, write the temporary layer into
        the given layer;
    */

    // WRITE YOUR CODE HERE
	const int height = terrain->get_map_height();
	const int width = terrain->get_map_width();

	float tempLayer[40][40];
	float maxInfluence = 0;
	float oldVal = 0;

	for (int i = 0; i < height; i++)
	{
		for (int j = 0; j < width; j++)
		{
			oldVal = layer.get_value(i, j);
			for (int k = -1; k < 2; k++)
			{
				for (int l = -1; l < 2; l++)
				{
					int row = i + k;
					int col = j + l;
					if (terrain->is_wall(i, j))
					{
						break;
					}
					if (row < 0 || col < 0 || row >= height || col >= width)
					{
						//out of boundary
						continue;
					}
					else
					{
						if (k == 0 || l == 0)
						{
							//orthogonal
							float currentInfluence = layer.get_value(row, col) * exp(-1 * decay);
							if (maxInfluence < currentInfluence)
							{
								maxInfluence = currentInfluence;
							}
						}
						else if (k != 0 && l != 0)
						{
							//diagonal
							if (!(terrain->is_wall(i + k, j) && terrain->is_wall(i, j + l)))
							{
								float currentInfluence = layer.get_value(row, col) * exp(-sqrt(2.f) * decay);
								if (maxInfluence < currentInfluence)
								{
									maxInfluence = currentInfluence;
								}
							}
						}
					}

				}
			}
			tempLayer[i][j] = lerp(oldVal, maxInfluence, growth);
			maxInfluence = 0;
		}
	}

	for (int i = 0; i < height; i++)
	{
		for (int j = 0; j < width; j++)
		{
			layer.set_value(i, j, tempLayer[i][j]);
		}
	}
}

void propagate_dual_occupancy(MapLayer<float> &layer, float decay, float growth)
{
    /*
        Similar to the solo version, but the values range from -1.0 to 1.0, instead of 0.0 to 1.0

        For every cell in the given layer:

        1) Get the value of each neighbor and apply decay factor
        2) Keep the highest ABSOLUTE value from step 1
        3) Linearly interpolate from the cell's current value to the value from step 2
           with the growing factor as a coefficient.  Make use of the lerp helper function.
        4) Store the value from step 3 in a temporary layer.
           A float[40][40] will suffice, no need to dynamically allocate or make a new MapLayer.

        After every cell has been processed into the temporary layer, write the temporary layer into
        the given layer;
    */

    // WRITE YOUR CODE HERE

	const int height = terrain->get_map_height();
	const int width = terrain->get_map_width();

	float tempLayer[40][40];
	float maxInfluence = 0;
	float oldVal = 0;

	for (int i = 0; i < height; i++)
	{
		for (int j = 0; j < width; j++)
		{
			oldVal = layer.get_value(i, j);
			for (int k = -1; k < 2; k++)
			{
				for (int l = -1; l < 2; l++)
				{
					int row = i + k;
					int col = j + l;
					if (terrain->is_wall(i, j))
					{
						break;
					}
					if (row < 0 || col < 0 || row >= height || col >= width)
					{
						//out of boundary
						continue;
					}
					else
					{
						if (k == 0 || l == 0)
						{
							//orthogonal
							float currentInfluence = layer.get_value(row, col) * exp(-1 * decay);
							if (abs(maxInfluence) < abs(currentInfluence))
							{
								maxInfluence = currentInfluence;
							}
						}
						else if (k != 0 && l != 0)
						{
							//diagonal
							if (!(terrain->is_wall(i+k, j)&& terrain->is_wall(i, j+l)))
							{
								float currentInfluence = layer.get_value(row, col) * exp(-sqrt(2.f) * decay);
								if (abs(maxInfluence) < abs(currentInfluence))
								{
									maxInfluence = currentInfluence;
								}
							}
						}
					}

				}
			}
			tempLayer[i][j] = lerp(oldVal, maxInfluence, growth);
			maxInfluence = 0;
		}
	}

	for (int i = 0; i < height; i++)
	{
		for (int j = 0; j < width; j++)
		{
			layer.set_value(i, j, tempLayer[i][j]);
		}
	}
}

void normalize_solo_occupancy(MapLayer<float> &layer)
{
    /*
        Determine the maximum value in the given layer, and then divide the value
        for every cell in the layer by that amount.  This will keep the values in the
        range of [0, 1].
    */

    // WRITE YOUR CODE HERE
	const int height = terrain->get_map_height();
	const int width = terrain->get_map_width();
	float findMax = FLT_MIN;
	for (int i = 0; i < height; i++)
	{
		for (int j = 0; j < width; j++)
		{
			if (findMax < layer.get_value(i, j))
				findMax = layer.get_value(i, j);
		}
	}

	for (int i = 0; i < height; i++)
	{
		for (int j = 0; j < width; j++)
		{
			layer.set_value(i, j, (layer.get_value(i, j)/ findMax));
		}
	}
}

void normalize_dual_occupancy(MapLayer<float> &layer)
{
    /*
        Similar to the solo version, but you need to track greatest positive value AND 
        the least (furthest from 0) negative value.

        For every cell in the given layer, if the value is currently positive divide it by the
        greatest positive value, or if the value is negative divide it by -1.0 * the least negative value
        (so that it remains a negative number).  This will keep the values in the range of [-1, 1].
    */

    // WRITE YOUR CODE HERE
	const int height = terrain->get_map_height();
	const int width = terrain->get_map_width();
	float findMax = FLT_MIN;
	float findMin = FLT_MAX;
	for (int i = 0; i < height; i++)
	{
		for (int j = 0; j < width; j++)
		{
			if (findMin > layer.get_value(i, j) && layer.get_value(i, j) != 0)
				findMin = layer.get_value(i, j);

			if (findMax < layer.get_value(i, j) && layer.get_value(i, j)!= 0)
				findMax = layer.get_value(i, j);
		}
	}

	for (int i = 0; i < height; i++)
	{
		for (int j = 0; j < width; j++)
		{
			if(layer.get_value(i, j)>0)
				layer.set_value(i, j, (layer.get_value(i, j) / findMax));
			else
				layer.set_value(i, j, (layer.get_value(i, j) / -findMin));
		}
	}
}

void enemy_field_of_view(MapLayer<float> &layer, float fovAngle, float closeDistance, float occupancyValue, AStarAgent *enemy)
{
    /*
        First, clear out the old values in the map layer by setting any negative value to 0.
        Then, for every cell in the layer that is within the field of view cone, from the
        enemy agent, mark it with the occupancy value.

        If the tile is close enough to the enemy (less than closeDistance),
        you only check if it's visible to enemy.  Make use of the is_clear_path
        helper function.  Otherwise, you must consider the direction the enemy is facing too.
        This creates a radius around the enemy that the player can be detected within, as well
        as a fov cone.
    */
	
    // WRITE YOUR CODE HERE
	const int height = terrain->get_map_height();
	const int width = terrain->get_map_width();
	const auto pos = enemy->get_position();
	const auto look = enemy->get_forward_vector();

	Vec2 vP;
	Vec2 vL;
	float pMag,lMag,dot,angle,cos = 0;

	for (int i = 0; i < height; i++)
	{
		for (int j = 0; j < width; j++)
		{
			if (layer.get_value(i, j)<0)
			{
					layer.set_value(i, j, 0);
			}

			vP.x = terrain->get_world_position(i, j).x - pos.x;
			vP.y = terrain->get_world_position(i, j).z - pos.z;
			vL.x = look.x;
			vL.y = look.z;

			dot = (vP.x* vL.x) + (vP.y*vL.y);

			pMag = sqrt(vP.x*vP.x + vP.y * vP.y);
			if (pMag > closeDistance)
			{
				lMag = sqrt(vL.x*vL.x + vL.y * vL.y);

				vP.x = vP.x / pMag;
				vP.y = vP.y / pMag;
				vL.x = vP.x / lMag;
				vL.y = vP.y / lMag;

				cos = dot / (pMag*lMag);

				if (cos == 0)
				{
					angle = 90;
				}
				else if (cos < 0)
				{
					continue;
				}
				else
				{
					angle = acos(cos);
					angle = 2 * angle * 180 / PI;
				}

				if (angle < fovAngle)
				{
					if (is_clear_path(terrain->get_grid_position(pos).row, terrain->get_grid_position(pos).col, i, j))
						layer.set_value(i, j, occupancyValue);
				}
			}
			else
			{
				if (is_clear_path(terrain->get_grid_position(pos).row, terrain->get_grid_position(pos).col, i, j))
					layer.set_value(i, j, occupancyValue);
			}
		}	
	}
	
}

bool enemy_find_player(MapLayer<float> &layer, AStarAgent *enemy, Agent *player)
{
    /*
        Check if the player's current tile has a negative value, ie in the fov cone
        or within a detection radius.
    */

    const auto &playerWorldPos = player->get_position();

    const auto playerGridPos = terrain->get_grid_position(playerWorldPos);

    // verify a valid position was returned
    if (terrain->is_valid_grid_position(playerGridPos) == true)
    {
        if (layer.get_value(playerGridPos) < 0.0f)
        {
            return true;
        }
    }

    // player isn't in the detection radius or fov cone, OR somehow off the map
    return false;
}

bool enemy_seek_player(MapLayer<float> &layer, AStarAgent *enemy)
{
    /*
        Attempt to find a cell with a value of 1.0, and then set it as the new target,
        using enemy->path_to.

        If there are multiple cells with 1.0, then pick the cell closest to the enemy.

        Return whether a target cell was found.
    */

    // WRITE YOUR CODE HERE
	const int height = terrain->get_map_height();
	const int width = terrain->get_map_width();

	GridPos destination;
	float posX = 0;
	float posZ = 0;
	float minDistance = FLT_MAX;
	float curDistance = minDistance;
	const auto pos = enemy->get_position();

	destination.row = -1;

	for (int i = 0; i < height; i++)
	{
		for (int j = 0; j < width; j++)
		{
			if (layer.get_value(i, j) == 1.f)
			{
				if (destination.row == -1)
				{
					destination.row = i;
					destination.col = j;
				}
				else
				{
					float posX = (abs(pos.x - i));
					float posZ = (abs(pos.z - j));
					curDistance = (sqrt(posX*posX + posZ * posZ));
					if (curDistance < minDistance)
					{
						destination.row = i;
						destination.col = j;
						minDistance = curDistance;
					}
				}
			}
		}
	}
	if(destination.row==-1)
    return false; 
	
	enemy->path_to(terrain->get_world_position(destination));
	return true;
}


