# ros-distributed-rendezvous-contrrol

Consensus (rendezvous) control by three TurtleBot3s (TB3s).
![visualization](https://raw.githubusercontent.com/estshorter/ros-distributed-rendezvous-contrrol/visualization/visualization.gif)

## Commands
Run Gazebo and spawn TBs:
``` bash
roslaunch mas empty_world.launch
```
or
``` bash
roslaunch mas mas_square_world_amcl.launch
```
Control TBs
``` bash
roslaunch mas contol.launch
```

RViz:
``` bash
rosrun rviz rviz -d `rospack find mas`/rviz/odom.rviz
```
``` bash
rosrun rviz rviz -d `rospack find mas`/rviz/odom_map.rviz
```

Reset simulation
``` bash
rosservice call /gazebo/reset_simulation
```

Install dependencies
``` bash
rosdep install -i --from-paths PATH-TO-THIS-PACKAGE
```

## Ref
- https://demura.net/education/16400.html
- https://github.com/bbingham-nps/turtlebot3_nps
- [Multiple Goal Buttons in RVIZ?](https://answers.ros.org/question/31559/multiple-goal-buttons-in-rviz/?answer=268515#post-id-268515)

