# Evry : TP Project

## Installation

You need Ubuntu 18.04 and ROS Melodic already installed. 

http://wiki.ros.org/melodic/Installation/Ubuntu

You need also git system : 

`sudo apt update && sudo apt install git` 

## Install rospkg for python3
```bash
sudo apt install python3-pip
sudo pip3 install rospkg
```

## Create your environment

In a terminal create catkin_ws folder, as follow: 

`mkdir -p ~/catkin_ws/src`

Then go to this folder, and copy the project :

```bash
cd ~/catkin_ws/src && git clone https://github.com/JohvanyROB/Evry_Project_2020.git
```

Finally, compile the project : 

```bash
cd ~/catkin_ws && catkin_make
source ~/catkin_ws/devel/setup.bash
echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
```

## Run the code

##### ON ONE TERMINAL : To run the Gazebo environment : 

`roslaunch evry_project_description simu_robot.launch group:=A nbr_robot:=3`

The arguments are : 

* group:=XXX : set the letter name of your group
* nbr_robot:=XXX : set the number of robot in your environment, maximum 3

##### ON ANOTHER TERMINAL : To run the robot's program : 

**1. If you want to have the output of your code on <u>multiple</u> terminals :** 

`roslaunch evry_project_strategy agent_terminal.launch group:=A nbr_robot:=3`

**2. If you want to have the output of your code on a <u>single</u> terminal :** 

`roslaunch evry_project_strategy agent.launch group:=A nbr_robot:=3`

## Edit the code

The robot's code is located at */evry_project_strategy/nodes/agent.py* 

You can edit directly the code itself and see the result.

![Gazebo's environment with robots](https://github.com/JohvanyROB/Evry_Project_2020/blob/main/Gazebo.PNG)

## Change the environment

By default, your robots operate in a simple environment with basic obstacles (lev1), but if you want to evaluate the robustness of your strategy, you can move to a more complex environment (lev2). 

You just need to modify the value of the argument **env** when launching the file **simu_robot.launch** as follow:

`roslaunch evry_project_description simu_robot.launch env:=lev2`
## Flag Detection Algorithm
In this algorithm three robots were used. This algorithm was inspired by GPS satellite. To
better understanding the algorithm, check the flow chart below.
<p align="center">
  <img width="500" height="600" src="https://user-images.githubusercontent.com/47057759/106003942-fcffcd80-60b2-11eb-9bc2-64cef59971d4.png">
</p>

### Go to Goal + Obstacle avoidance
First Go to Goal:
<p align="center">
  <img width="500" height="300" src="https://user-images.githubusercontent.com/47057759/106004323-641d8200-60b3-11eb-9fb0-2682ac20aa77.png">
</p>

<p align="center">
  <img width="500" height="300" src="https://user-images.githubusercontent.com/47057759/106004563-9fb84c00-60b3-11eb-921b-2f302c9ffc2a.png">
</p>

<p align="center">
  <img width="500" height="300" src="https://user-images.githubusercontent.com/47057759/106004698-c37b9200-60b3-11eb-8d54-2f2aad8ce0e8.png">
</p>

### Localization Procedure
As mentioned before the main idea is that the flag position can be calculated using the
following geometric relation:

<p align="center">
  <img width="500" height="300" src="https://user-images.githubusercontent.com/47057759/106004863-ec9c2280-60b3-11eb-8e2d-108dc0ff5aa3.png">
</p>
Each robot will calculate the flag position by solving three equations in three in two unknowns
(flag position). The following equations are used to calculate x and y of flag:
Using circle equation for the three robot motions
<p align="center">
  <img width="500" height="300" src="https://user-images.githubusercontent.com/47057759/106004988-0e95a500-60b4-11eb-9a49-866034afb7da.png">
</p>
For the searching procedure there is two possibilities:
1. Random search
2. Cover most interesting area of the map by passing through certain way points
After testing the two methods, the second one seems to be more efficient. Because for
random search, the robots may not visit certain area for a while.

### Robots Cooperation
In this strategy, robots are able to share information to enhance the flag position estimation
and to update the list of discovered flags. Each robot publish a topic “/flagInfo” which contains
a message of type vector3. This message contain the x and y position and the flag id of the
discovered flag.

### Challenges
1. Distance to flag Errors: for this problem we used four points instead of three. This gave
us two estimations for the flag position so we can take the average to get a better result.
In addition, we don’t take the estimation into account if the difference between the two
estimation is bigger than certain threshold.
2. Estimation accuracy: to improve the accuracy of the estimation, if the flag is already
discovered, update the flag position by taking the average between the new and the old
flag position. Also the bigger the simulation time, the more accurate localization you will
get.

### Testing and Validation
Finally, after testing the algorithm, this strategy accomplished the mission successfully. The
following screenshot is one of the trails. As you can see, the eight flags position were estimated
in less than 2.19 sec with high accuracy. <br>
Check the video
[![Watch the video](https://img.youtube.com/vi/oNLR5qCUIAU/hqdefault.jpg)](https://youtu.be/oNLR5qCUIAU)
