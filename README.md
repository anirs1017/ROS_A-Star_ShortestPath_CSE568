# ROS_A-Star_ShortestPath_CSE568
Use A* planning algorithm to find shortest route between source and destination in ROS for a given map.
<br>The objective of this assignment is to plan a path for a robot from a given starting
point to a destination. Default start point: (-8.0, -2.0) to a default goal: (4.5, 9.0)
<br> The world files (<b>playground.pgm</b> and <b>playground.world</b>) are in the <i>world</i> folder. 
<br><br><i>Challenges:</i>
<br>The first challenge is to derive a graph representation of the workspace. This depends
on the map representation that the estimation block provides us. Typical examples of
such representations are occupancy grids - a grid representation with 1s and 0s with 1
indicating an obstacle in that cell and 0 representing an empty cell. We currently have one such occupancy grid, map.txt. It grids the world
as 1X1 cells. 
<br><br>The second challenge is the heuristic for the estimated cost between the current node
and the goal. If we know the current location and the goal, we can use Euclidean
distance between the current location and the goal as the heuristic cost.
Once planned, command the robot to execute the plan to go from start to
goal.
