# Robot-Planner
Planner and controller to help the turtle bot navigate through a maze

Launch the package using 
```
roslaunch <package name> obs_world.launch
```
The robot uses the RRT* path planning algorithm to plan a path to the end point. The nodes on the path are then communicate to the controller ros node which then uses a PID controller to help the robot move in the simulation to the goal. The controller was tuned according to the needs of the problem and relevant path correction techniques were employed.

Noob tip: Do use data analysis software instead of printing data on the terminal like I did :)
