# [Udacity Robotics Projects](https://www.udacity.com/course/robotics-software-engineer--nd209)

certificate: <https://graduation.udacity.com/confirm/DVYYUTRZ>

## Build My World

Create a Gazebo world, interact with it through plugins, and design models using Gazebo tools, such as the moderator tool and the building editor tool.

```
gazebo world/myworld
```



https://user-images.githubusercontent.com/7373750/178127931-666cabd9-0647-414b-b332-870778493d3c.mov




## Go Chase it

Build a bold chaser mobile robot with ROS
1. design the robot and house it inside my world
2. program it to chase white colored balls through the world

```
roslaunch my_robot world.launch
roslaunch ball_chaser ball_chaser.launch
```

https://user-images.githubusercontent.com/7373750/178127712-0104a459-7132-4545-b487-67427c4c2f4d.mov


## Where Am I

Utilize ROS AMCL package to accurately localize a mobile robot inside a map in the Gazebo simulation environments
* Create a ROS package that launches a custom robot model in a custom Gazebo world
* Utilize the ROS AMCL package and the Tele-Operation / Navigation Stack to localize the robot
* Explore, add, and tune specific parameters corresponding to each package to achieve the best possible localization results

```
roslaunch my_robot world.launch
roslaunch my_robot amcl.launch
```

https://user-images.githubusercontent.com/7373750/178127874-dfb523a4-2328-46e9-89c4-765a5b12bf5c.mov


## Map My World

Create a 2D occupancy grid and 3D octomap from a simulated environment using my own robot with the RTAB-Map package

```
roslaunch my_robot world.launch
rosrun teleop_twist_keyboard teleop_twist_keyboard.py
roslaunch my_robot mapping.launch
```

![mapping_2D](https://user-images.githubusercontent.com/7373750/178127722-efb8881e-4bda-4cd4-9119-5fa77bc943b9.png)
![mapping_3D](https://user-images.githubusercontent.com/7373750/178127727-1ce5df2e-df5b-41f2-a013-c67193383aa9.png)


## Home Service Robot

Program a robot that can autonomously map an environment and navigate to pick up and dropped virtual objects

```
bash src/scripts/home_service.sh
```

https://user-images.githubusercontent.com/7373750/178127730-48abbc01-9201-4109-91f6-227daa8105f3.mov
