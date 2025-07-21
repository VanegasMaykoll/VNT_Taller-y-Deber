# TurtleBot4 - Simulation
This tutorial is to help how to know to simulate Turtlebot 4 

## 0. Content
  1. Install simulation packages.
  2. Launch Simulation on the scenaries warehouse,Depot and maze. 
  3. Turtlebot4 in rviz2.
  4. Visualize Rplidar in Ignition Gazebo. 
  5. Generating a map (Slam_toolbox)
  6. Navigation(Nav2)
  7. References

## Prerequisites:
   Ros 2 Humble and  all the necessary packages related to the simulation of turtlebot4 that are downloaded while the tutorial is being carried out. 
  
  
Here we are going to install everything about turtlebot4.
## 1. Install simulation packages
The turtlebot4_simulator metapackage contains packages used to simulate the TurtleBot 4 in Ignition Gazebo.
Please follow the steps. 
we need to Install useful development tools with next code:
```bash
sudo apt install ros-dev-tools
```
## Install Igntion Gazebo. 
```bash
sudo apt-get update && sudo apt-get install wget
sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
sudo apt-get update && sudo apt-get install ignition-fortress
```
## Install Debian package. 
```bash
sudo apt update
sudo apt install ros-humble-turtlebot4-simulator
```

The installation of the package ros-humble-turtlebot4-simulator is necessary if you want to simulate a TurtleBot 4 robot on your system. This package contains the    
necessary files and tools to simulate the TurtleBot 4 in the Ignition Gazebo simulation environment, which is a popular simulation platform used in robotics.     
   
## Source installation:
To manually install this metapackage from source, clone the git repository:
```bash
mkdir -p ~/turtlebot4_ws/src  
cd ~/turtlebot4_ws/src
git clone https://github.com/turtlebot/turtlebot4_simulator.git -b humble
```
## Install dependencies:
```bash
cd ~/turtlebot4_ws
rosdep install --from-path src -yi
```
## Build the packages:
```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install
```

## 2. Launch Simulation in world warehouse.
__Note important__ 
When we run the simulation for the first time we have an error related to "Requesting list of worlds". in this link we can find more information about it, https://github.com/gazebosim/gz-sim/issues/38
To avoid an error with "Requesting list of worlds", execute the following command:        
```bash
export IGN_IP=127.0.0.1
```
This command sets the IGN_IP environment variable to 127.0.0.1, resolving the "Requesting list of worlds" issue. Make sure to run this command before launching the
simulation.   

we launch in the two models robot standard and lite. 
The turtlebot4_ignition.launch.py launch file has several launch configurations that allow the user to customize the simulation.
This world for default is warehouse.
Default TurtleBot 4 launch is the standard:
```bash
cd turtlebot4_ws    
ros2 launch turtlebot4_ignition_bringup turtlebot4_ignition.launch.py
```

![image](https://github.com/user-attachments/assets/d3723def-668a-4372-8f40-dbae6c36676f)

TurtleBot 4 Lite launch:
```bash
ros2 launch turtlebot4_ignition_bringup turtlebot4_ignition.launch.py model:=lite
```


![image](https://github.com/user-attachments/assets/5066d70b-aebf-41c0-b9ce-5f216755e077)


## 2.1 Launch Simulation in the world maze:
Here, we are going to run the launch of turtlebot4_ignition.launch.py  but in the world maze. 
Default TurtleBot 4 launch is the standard:       
```bash
cd turtlebot4_ws
ros2 launch turtlebot4_ignition_bringup turtlebot4_ignition.launch.py world:=maze
```

![image](https://github.com/user-attachments/assets/69b63aad-3a5a-4771-b30f-da923870c6f3)


TurtleBot 4 Lite launch:   
```bash
cd turtlebot4_ws
ros2 launch turtlebot4_ignition_bringup turtlebot4_ignition.launch.py world:=maze model:=lite
```
![image](https://github.com/user-attachments/assets/873dbeef-d0e2-459d-ae7b-804b97005c9d)


## 2.2 Launch Simulation in the world depot:
Here, we are going to run the launch of turtlebot4_ignition.launch.py in the world depot:
Default TurtleBot 4 launch is the standard:   
```bash
cd turtlebot4_ws
ros2 launch turtlebot4_ignition_bringup turtlebot4_ignition.launch.py world:=depot
```
![image](https://github.com/user-attachments/assets/1e03f358-30c3-4de3-ac50-3953ad98efcb)


TurtleBot 4 Lite launch:   
```bash
cd turtlebot4_ws
ros2 launch turtlebot4_ignition_bringup turtlebot4_ignition.launch.py world:=depot model:=lite
```
![image](https://github.com/user-attachments/assets/c43e7a5d-0677-429c-9043-9d3ceb49544c)




## Note important 
we have some of forms to drive the turtlebot 4, so here we are going to explain two forms the terminal forms and Interface of Gui.
##  Terminal form 
we have to know what nodes are actives when the launch file is running. 
```bash
ros2 topic list
```

![image](https://github.com/user-attachments/assets/1edfeeef-fa6b-410c-90c8-4a9b3401773f)


as can be seen in the image, we are going to use the node /cmd_vel to drive the turtlebot4.
With this we can run the next code in the terminal, always the launch file should be running in our case the ingnition.launch.py 
```bash 
ros2 topic pub /cmd_vel geometry_msgs/Twist '{linear: {x: 0.7 , y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'
```
you have to change only the linear in x and angular in z. 
![image](https://github.com/user-attachments/assets/1223d0d9-85ed-4184-848f-e594f7e05a5b)

![image](https://github.com/user-attachments/assets/adaeda72-a43d-4559-bb4e-c3af56896915)




## Ignition GUI Plugins
In this case is so different and easier than terminal form, we have to set a velocity linear and velocity angular in interface. and then we have to press play, and the last we have to press the arrows in the direction we want our robot to move.
![image](https://github.com/user-attachments/assets/1add8455-0e4c-4968-aadc-be69617a078d)



## 3. Turtlebot4 in rviz2. 

To view the Turtlebot4 in rviz2, we need to install another package called "turtlebot4_desktop", so let's go ahead and do that.

## Install Debian package. 
```bash
sudo apt update
sudo apt install ros-humble-turtlebot4-desktop
```
## Source installation:
To manually install this metapackage from source, clone the git repository:
```bash
cd ~/turtlebot4_ws/src
git clone https://github.com/turtlebot/turtlebot4_desktop.git -b humble
```
## Install dependencies:
```bash
cd ~/turtlebot4_ws
rosdep install --from-path src -yi
```
## Build the packages:
```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install
```

## View robot:
With the next command we can see the robot in rviz 2. 
```bash
ros2 launch turtlebot4_viz view_robot.launch.py
```
![image](https://github.com/user-attachments/assets/14ae4efe-cfcc-40e6-a25e-d10d03946519)


## View Model 
To inspect the model :
```bash
ros2 launch turtlebot4_viz view_model.launch.py
```
Standard
![image](https://github.com/user-attachments/assets/ab98acdf-067a-4309-bf24-064a5d08898d)

Lite:
![image](https://github.com/user-attachments/assets/86816512-ed3b-426a-abdf-60be49017039)


__Note important__

As you can see, there is an error in the images in the part of the robot model in rviz, this was corrected by changing to ros humble since in ros galactic there were outdated packages that were not going to be updated. Because ros galactic is already in its final stage.



## Rqt of Turtlebot4 
Rqt_graph displays a graph of the active nodes in your ROS 2 system. This plugin allows you to visualize the connections between nodes and their associated topics, services, and actions.
```bash
ros2 run rqt_graph rqt_graph
```
for example when run the launch on Ignition Gazebo, we can see the next active nodes. 
![image](https://github.com/user-attachments/assets/5d35b069-e4d6-4f5e-a0cc-d15487970735)


## 4. How to visualize the Lidar in Gazebo 
__Note Important__   

Here's what you need to do if you don't have a GPU because ignition gazebo support the gpu lidar which use ign-rendering and detect visuals in the scene  and the LiDAR is configured to work with GPU. Open the terminal and type the following commands:
```bash
export LIBGL_ALWAYS_SOFTWARE=true
```
This command forces the LiDAR to work with the CPU instead of the GPU for rendering. Make sure to run this command before launching the simulation to ensure that the LiDAR operates correctly without a dedicated GPU.Read more about it in the next link https://github.com/gazebosim/gz-sensors/issues/26 and https://robotics.stackexchange.com/questions/24883/turtlebot-4-simulation-rplidar-not-working.    
Otherwise our rplidar sensor will not detect anything.      
First, we need to run the simulation, in this case, launch the ignition.launch.py  
```bash
ros2 launch turtlebot4_ignition_bringup ignition.launch.py 
```
Select the three points at the top right
![image](https://github.com/user-attachments/assets/eff8883d-6335-456b-a504-d1508261bc9e)


Start typing to find visualize lidar and select it
![image](https://github.com/user-attachments/assets/36a81193-b1a7-4bfa-a9c3-6878b78d0b25)



Scroll down until you find the Visualize Lidar section. Click on the refresh button and select the correct lidar that you want to visualize from the drop down.
![image](https://github.com/user-attachments/assets/d0cd6f7c-5eeb-4344-bd08-9a0dbef23a03)

you have to press the refresh. 

![image](https://github.com/user-attachments/assets/46140f70-8192-433b-b693-107f8fe42254)


If you don't see the lidar, ensure that your simulation is playing (should see a pause symbol in the bottom left).  

## how to change the range of the rplidar?
We have to modify the parameter "r_max" in the urdf of rplidar(rplidar.urdf.xacro in the package named turtlebot4_description. 
For example in the next picture the r_max sets in the 1.0 m:

![image](https://github.com/user-attachments/assets/ccea6278-722a-4e86-b5a5-5c5f2ac237f1)


![image](https://github.com/user-attachments/assets/7024d932-0fb5-4854-afca-dfc6b6fe0f34)


In this picture the r_max sets in the 12.0 m :

![image](https://github.com/user-attachments/assets/f2f441c1-2e47-4882-9e6a-7a3bb4926853)


![image](https://github.com/user-attachments/assets/b6c6040a-f6f4-434b-abeb-fd4a0319ff12)


## 5. Generating a map using the slam_toolbox package:
we will be mapping an area by driving the TurtleBot 4 around and using SLAM.The creation of the map in real time using the slam_toolbox package, built by default in TurtleBot 4.


## First, install turtlebot4_navigation:
```bash
sudo apt install ros-humble-turtlebot4-navigation
```
## Then run SLAM. It is recommended to run synchronous SLAM on a remote PC to get a higher resolution map.
```bash

ros2 launch turtlebot4_navigation slam.launch.py
```
## Option(Asynchronous SLAM can be used as well.)
```bash

ros2 launch turtlebot4_navigation slam.launch.py sync:=false
```
## Launch Rviz2
To visualise the map, launch Rviz2 with the view_robot launch file.
```bash
ros2 launch turtlebot4_viz view_robot.launch.py
```
![Captura de pantalla de 2024-03-12 12-17-02](https://github.com/nabihandres/COOP_tutorials/assets/93724428/666023f5-78cb-4c04-a31a-49d8afea8aa3)


when you launch rviz2 you must launch the Ignition gazebo. Because in this case is simulated. 
```bash
ros2 launch turtlebot4_ignition_bringup turtlebot4_ignition.launch.py---humble
```
![Captura de pantalla de 2024-02-28 11-42-50](https://github.com/nabihandres/COOP_tutorials/assets/93724428/4ed71a48-293b-4247-8292-8534123e3cab)


## Save the map    

Once you are happy with your map, you can save it with the following command:     
```bash
ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap "name:
  data: 'map_name'"
```
## Video 

https://github.com/user-attachments/assets/bb17deda-5221-4067-817d-1ce7f774255c





## 6 .Navigation 
This tutorial will cover various methods of navigating with the TurtleBot 4 and Nav2. 

SLAM or Localization. SLAM allows us to generate the map as we navigate, while localization requires that a map already exists.

__SLAM__

SLAM is useful for generating a new map, or navigating in unknown or dynamic environments. It updates the map as it detects and changes, but cannot see areas of the environment that it has not discovered yet.

__Localization__

Localization uses an existing map along with live odometry and laserscan data to figure out the position of the robot on the given map. It does not update the map if any changes have been made to the environment, but we can still avoid new obstacles when navigating. Because the map doesn't change, we can get more repeatable navigation results.

For this tutorial, we will be using localization to navigate on a map generated with SLAM.   
Nav2

The TurtleBot 4 uses the Nav2 stack for navigation.
As we are using the simulator, we need to run this command in the terminal. 
```bash 
ros2 launch turtlebot4_ignition_bringup turtlebot4_ignition.launch.py nav2:=true slam:=false localization:=true rviz:=true
```
We will explain what happens with each argument that is launched in that line:
__nav2:=true__ : This will launch the nav2 navigation system, which will have some configurations but for global planner it will use A* or dijkstra algorithm, for local planner it uses what is dwa.
__localization:=true__ : This will launch the robot localization system that uses the amcl package that implements a localization algorithm using a particle filter.
__slam__ : This is set false, it will not use the slam_toolbox package which uses a modification of the OpenKarto library is a set of libraries and tools that has become the ROS 2 standard, so it is installed by default in TrutleBot 4. It has loop-closing and is based on graph optimization., If this is true it means that it does not have a map created, so it will navigate and map it for the first time.
This will launch the simulation in the default warehouse world and will use the existing warehouse.yaml file for the map.

To launch a different supported world, see the simulation package for a list of supported worlds. You must pass the name of the chosen world and the path to the map file.

For example:
```bash 
ros2 launch turtlebot4_ignition_bringup turtlebot4_ignition.launch.py nav2:=true slam:=false localization:=true \
rviz:=true world:=depot map:=/opt/ros/humble/share/turtlebot4_navigation/maps/depot.yaml
```
## Interacting with Nav2

In a new terminal launch Rviz so that you can view the map and interact with navigation:
```bash
ros2 launch turtlebot4_viz view_robot.launch.py
```
At the top of the Rviz window is the toolbar. You will notice that there are three navigation tools available to you.
![image](https://github.com/user-attachments/assets/7fcbe5bb-1af4-42b9-9d21-9511c195cbc1)



## 2D Pose Estimate

The 2D Pose Estimate tool is used in localization to set the approximate initial pose of the robot on the map. This is required for the Nav2 stack to know where to start localizing from. Click on the tool, and then click and drag the arrow on the map to approximate the position and orientation of the robot.

![image](https://github.com/user-attachments/assets/12615739-1cc7-4597-a4b3-0ad4132463a0)



## Publish Point

The Publish Point tool allows you to click on a point on the map, and have the coordinates of that point published to the /clicked_point topic.

Open a new terminal and call:
```bash
ros2 topic echo /clicked_point
```
Then, select the Publish Point tool and click on a point on the map. You should see the coordinates published in your terminal.

![image](https://github.com/user-attachments/assets/da89a706-956f-4d3c-b48f-b3e8b634e8e8)

![image](https://github.com/user-attachments/assets/e4dc4693-4b69-419f-8fe8-8b463d9f7a08)



## Nav2 Goal

The Nav2 Goal tool allows you to set a goal pose for the robot. The Nav2 stack will then plan a path to the goal pose and attempt to drive the robot there. Make sure to set the initial pose of the robot before you set a goal pose.

![image](https://github.com/user-attachments/assets/f2df078a-70ac-4695-a29f-74ffe2210798)

First Nav2 goal.
![image](https://github.com/user-attachments/assets/727cab61-e3e7-42c4-9cf0-d7d790fa3a51)

continue, second nav2 goal. 
![image](https://github.com/user-attachments/assets/ae503fff-6bc7-43b4-aa76-4506f3b3abdd)



## Video 

https://github.com/nabihandres/Turtlebot4_ESPOL/blob/main/images_videos/navigation.mp4



## 7 .References 
https://turtlebot.github.io/turtlebot4-user-manual/overview/
https://github.com/turtlebot/turtlebot4/issues
https://docs.ros.org/en/galactic/index.html
