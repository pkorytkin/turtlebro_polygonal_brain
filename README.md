#turtlebro_polygonal_brain

This package allows the TurtleBro robot to move around a polyhedron with a given number of sides. Python 2.7+ **(TurtleBro supported only Python 2.7)**

This is fork of https://github.com/pkorytkin/turtlesim_polygonal_brain 

**Installation:**

Clone this package in ~catkin_ws/src/


**Helper commands if it does not find this package:**

echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc

source ~/.bashrc

source /opt/ros/noetic/setup.bash


echo "source ${HOME}/catkin_ws/devel/setup.bash" >> ~/.bashrc

source ~/.bashrc

**Run:**

In first terminal:

roscore

In second terminal:

rosrun turtlebro_polygonal_brain brain.py

**or**

roslaunch turtlebro_polygonal_brain launch.launch 


