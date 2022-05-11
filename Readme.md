# FASTER: Fast and Safe Trajectory Planner for Navigation in Unknown Environments #

### **Accepted for publication in the IEEE Transactions on Robotics (T-RO)**

### **Finalist to the Best Paper Award on Safety, Security, and Rescue Robotics (IROS 2019)**

UAV               |  Ground Robot           | 
:-------------------------:|:-------------------------:|
[![IROS 2019: FASTER: Fast and Safe Trajectory Planner for Flights in Unknown Environments](./faster/imgs/uav_sim.gif)](https://www.youtube.com/watch?v=fkkkgomkX10 "IROS 2019: FASTER: Fast and Safe Trajectory Planner for Flights in Unknown Environments")      |  [![IROS 2019: FASTER: Fast and Safe Trajectory Planner for Flights in Unknown Environments](./faster/imgs/gr_sim.gif)](https://youtu.be/L13k44-krcc "IROS 2019: FASTER: Fast and Safe Trajectory Planner for Flights in Unknown Environments") |  
[![IROS 2019: FASTER: Fast and Safe Trajectory Planner for Flights in Unknown Environments](./faster/imgs/uav_hw.gif)](https://www.youtube.com/watch?v=fkkkgomkX10 "IROS 2019: FASTER: Fast and Safe Trajectory Planner for Flights in Unknown Environments")       |  [![IROS 2019: FASTER: Fast and Safe Trajectory Planner for Flights in Unknown Environments](./faster/imgs/gr_hw.gif)](https://www.youtube.com/watch?v=fkkkgomkX10 "IROS 2019: FASTER: Fast and Safe Trajectory Planner for Flights in Unknown Environments")    |  

## Citation

When using FASTER, please cite the following journal paper  ([pdf](https://arxiv.org/abs/2001.04420), [video](https://www.youtube.com/watch?v=fkkkgomkX10))
```bibtex
@article{tordesillas2021faster,
  title={{FASTER}: Fast and Safe Trajectory Planner for Navigation in Unknown Environments},
  author={Tordesillas, Jesus and How, Jonathan P},
  journal={IEEE Transactions on Robotics},
  year={2021},
  publisher={IEEE}
}
```

The conference version is [here](https://arxiv.org/abs/1903.03558): 
```bibtex
@inproceedings{tordesillas2019faster,
  title={{FASTER}: Fast and Safe Trajectory Planner for Flights in Unknown Environments},
  author={Tordesillas, Jesus and Lopez, Brett T and How, Jonathan P},
  booktitle={2019 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS)},
  year={2019},
  organization={IEEE}
}

```


## General Setup
FASTER has been tested with 
* Ubuntu 16.04/ROS Kinetic  
* Ubuntu 18.04/ROS Melodic 

Other ROS versions may require some minor changes, feel free to [create an issue](https://github.com/mit-acl/faster/issues) if you have any problems. The Gurobi versions tested are Gurobi 8.1, Gurobi 9.0, and Gurobi 9.1.

Install the [Gurobi Optimizer](https://www.gurobi.com/products/gurobi-optimizer/). You can test your installation typing `gurobi.sh` in the terminal. Have a look at [this section](#issues-when-installing-gurobi) if you have any issues.

Install the following dependencies:
```
sudo apt-get install ros-"${ROS_DISTRO}"-gazebo-ros-pkgs ros-"${ROS_DISTRO}"-mavros-msgs ros-"${ROS_DISTRO}"-tf2-sensor-msgs
```
```
python -m pip install pyquaternion
```

Create a workspace, and clone this repo and its dependencies:
```
mkdir ws && cd ws && mkdir src && cd src
git clone https://github.com/mit-acl/faster.git
wstool init
wstool merge ./faster/faster/install/faster.rosinstall

```

In the following, remember (once the workspace is compiled) to add this to your `~/.bashrc`:
```
source PATH_TO_YOUR_WS/devel/setup.bash
``` 

### Instructions to use FASTER with an aerial robot:

Compile the code:

```
wstool update -j8
cd ..
catkin config -DCMAKE_BUILD_TYPE=Release
catkin build
```

And finally open 5 terminals and execute these commands:
```
roslaunch acl_sim start_world.launch
roslaunch acl_sim perfect_tracker_and_sim.launch
roslaunch global_mapper_ros global_mapper_node.launch
roslaunch faster faster_interface.launch
roslaunch faster faster.launch
```
The blue grid shown in Rviz is the unknown space and the orange one is the occupied-known space. Now you can click `Start` in the GUI, and then, in RVIZ, press `G` (or click the option `2D Nav Goal` on the top bar of RVIZ) and click any goal for the drone. 

> **_NOTE (TODO):_**  Right now the radius of the drone plotted in Gazebo (which comes from the `scale` field of `quadrotor_base_urdf.xacro`) does not correspond with the radius specified in `faster.yaml`. 


### Instructions to use FASTER with a ground robot:

> **_IMPORTANT NOTE:_**  There are some important differences on the performance of the ground robot when using the Gazebo version that comes with ROS Kinetic and the one that comes with ROS Melodic. To achieve a good tracking error (like the one shown [here](https://github.com/mit-acl/faster/blob/master/faster/imgs/gr_sim.gif)), you may have to tune the [gains of the controller](https://github.com/mit-acl/faster/blob/master/faster/scripts/goal_odom_to_cmd_vel_state.py) depending on the specific verion of ROS/Gazebo that you are using. 

Install the following dependencies:
```
sudo apt-get install ros-"${ROS_DISTRO}"-control-toolbox ros-"${ROS_DISTRO}"-ros-control ros-"${ROS_DISTRO}"-robot-localization ros-"${ROS_DISTRO}"-lms1xx ros-"${ROS_DISTRO}"-interactive-marker-twist-server ros-"${ROS_DISTRO}"-hector-gazebo-plugins ros-"${ROS_DISTRO}"-move-base ros-"${ROS_DISTRO}"-ros-control ros-"${ROS_DISTRO}"-ros-controllers ros-"${ROS_DISTRO}"-pointgrey-camera-description ros-"${ROS_DISTRO}"-hardware-interface ros-"${ROS_DISTRO}"-message-to-tf ros-"${ROS_DISTRO}"-gazebo-ros-control
```
Then download the ground_robot-specific packages and compile the repo:

```
wstool merge ./faster/faster/install/faster_ground_robot.rosinstall
wstool update -j8
cd ..
catkin config -DCMAKE_BUILD_TYPE=Release
catkin build
```


Then, in [`faster.yaml`](https://github.com/mit-acl/faster/tree/master/faster/param), change these parameters:
```
drone_radius: 0.5  #[m]

z_max: 0.5         #[m] 
z_ground: -0.2

v_max: 1.4         #[m/s]  
a_max: 1.4         #[m/s2] 
j_max: 5.0         #[m/s3]

is_ground_robot: true  
```

And finally open 4 terminals and execute these commands
```
roslaunch faster ground_robot.launch
roslaunch global_mapper_ros global_mapper_node.launch quad:=JA01
roslaunch faster faster_interface.launch quad:=JA01 is_ground_robot:=true
roslaunch faster faster.launch quad:=JA01
```

Now you can click `Start` in the GUI, and then, in RVIZ, press `G` (or click the option `2D Nav Goal` on the top bar of RVIZ) and click any goal for the ground robot. 


## Running FASTER in Singularity

### Install Singularity

Install Singularity using [these](https://sylabs.io/guides/3.5/user-guide/quick_start.html#quick-installation-steps) instructions (compiled from source). The following combination is known to work :

* Ubuntu/Kubuntu 20.04
* ROS Noetic
* Go 1.18
* Singularity 3.9.8 

### Singularity Definition File

Save the below code as `fasterSing.def`. 

```
Bootstrap: docker
From: ros:melodic

%setup
    mkdir -p ${SINGULARITY_ROOTFS}/faster_ws/src/
    
%files

%post -c /bin/bash
    apt -y update; apt-get -y install wget python3 python tar python3-wstool python3-pip python-pip
	
    wget https://packages.gurobi.com/9.5/gurobi9.5.1_linux64.tar.gz
    tar xzvf gurobi9.5.1_linux64.tar.gz
    cd gurobi951/linux64
    python3 setup.py install

    export GUROBI_HOME="/gurobi951/linux64"
    export PATH="${PATH}:${GUROBI_HOME}/bin"
    export LD_LIBRARY_PATH="${LD_LIBRARY_PATH}:${GUROBI_HOME}/lib"

    bash --
    
    cd /gurobi951/linux64/src/build
    make
    cp libgurobi_c++.a ../../lib/

    cd /faster_ws/src/
    git clone https://github.com/mit-acl/faster.git
    wstool init
    wstool merge ./faster/faster/install/faster.rosinstall

    wstool merge ./faster/faster/install/faster_ground_robot.rosinstall
    wstool update -j8
    cd ..
   
    echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc
    source ~/.bashrc


%environment
    export GUROBI_HOME="/gurobi951/linux64"
    export PATH="${PATH}:${GUROBI_HOME}/bin"
    export LD_LIBRARY_PATH="${LD_LIBRARY_PATH}:${GUROBI_HOME}/lib"
```

### Building Container

Run the following command after saving the `def` file:

```
sudo singularity build --sandbox fasterSing.sif fasterSing.def 
```

(if the build fails contact me ASAP)

Run the container using the following code:

```
sudo singularity run --writable fasterSing.sif
```

you can run this in multiple terminals and they will be connected to the same instance of the Singularity container.

### Building FASTER

After running the container install the following dependencies in the container environment :

```
apt-get install ros-melodic-desktop-full
pip3 install pyquaternion
pip install pyquaternion


apt-get install ros-melodic-catkin python3-catkin-tools python3-osrf-pycommon libeigen3-dev ros-melodic-gazebo-ros-pkgs ros-melodic-mavros-msgs ros-melodic-tf2-sensor-msgs ros-melodic-rqt ros-melodic-rqt-common-plugins ros-melodic-gazebo-ros ros-melodic-lms1xx ros-melodic-robot-localization ros-melodic-interactive-marker-twist-server ros-melodic-mavros-msgs ros-melodic-hector-gazebo-plugins ros-melodic-move-base ros-melodic-tf2-sensor-msgs ros-melodic-pointgrey-camera-driver  ros-melodic-pointgrey-camera-description  ros-melodic-message-to-tf ros-melodic-control-toolbox ros-melodic-ros-control ros-melodic-robot-localization ros-melodic-lms1xx ros-melodic-interactive-marker-twist-server ros-melodic-hector-gazebo-plugins ros-melodic-move-base ros-melodic-ros-control ros-melodic-ros-controllers ros-melodic-pointgrey-camera-description ros-melodic-hardware-interface ros-melodic-message-to-tf ros-melodic-gazebo-ros-control 
```

For seting up Gurobi license, replace `<your_key>` in ` grbgetkey <your_key>` with your Gurobi license key, which you can get after creating an account and then applying for an academic license from your profile.

```
grbgetkey <your_key>
```

Then finally build FASTER:

```
cd /faster_ws/
catkin config -DCMAKE_BUILD_TYPE=Release
catkin build 
```

### Running The Simulation

After running the container, in [`faster.yaml`](https://github.com/mit-acl/faster/tree/master/faster/param) (in faster/param folder), change these parameters (use a tool like vim or nano):
```
drone_radius: 0.5  #[m]

z_max: 0.5         #[m] 
z_ground: -0.2

v_max: 1.4         #[m/s]  
a_max: 1.4         #[m/s2] 
j_max: 5.0         #[m/s3]

is_ground_robot: true  
```

And finally open 4 terminals (by running the singularity container in 4 terminals) and execute these commands. Dont forget to source `source /faster_ws/devel/setup.bash`

```
roslaunch faster ground_robot.launch
roslaunch global_mapper_ros global_mapper_node.launch quad:=JA01
roslaunch faster faster_interface.launch quad:=JA01 is_ground_robot:=true
roslaunch faster faster.launch quad:=JA01
```

Now you can click `Start` in the GUI, and then, in RVIZ, press `G` (or click the option `2D Nav Goal` on the top bar of RVIZ) and click any goal for the ground robot. If there is an error in connecting to X server then execute `xhost +` before running singularity.



## Architecture:


![](./faster/imgs/diagram.png) 

For the aerial robot, the option 3 is provided (a perfect tracker and a Gazebo simulation to obtain the depth). To make the simulation faster, the physics engine of Gazebo is disabled using [`disable_physics.cpp`](https://gitlab.com/mit-acl/lab/acl-gazebo/-/blob/master/acl_sim/src/disable_physics.cpp) of [`acl-gazebo`](https://gitlab.com/mit-acl/lab/acl-gazebo).

For the ground robot, the option 2 is provided (a controller, and a Gazebo simulation with both dynamics and depth (using the `multi_jackal` package)

## How do I use FASTER with a real robot?
To run the mapper, you simply need to run the node that publishes the depth image of the camera (for example using [`realsense-ros`](https://github.com/IntelRealSense/realsense-ros) if you are using an Intel RealSense camera). After launching this node, find the name of the topic of the depth image (by doing `rostopic echo`), and finally put that name on the [launch file of the mapper](https://gitlab.com/mit-acl/lab/acl-mapping/-/blob/dev/global-mapper/global_mapper_ros/launch/global_mapper_node.launch#L12). 

As shown in the diagram above, the UAV should subscribe to the [Goal message](https://gitlab.com/mit-acl/fsw/snap-stack/snapstack_msgs/-/blob/46a1911faed1a5b1be479df2f969ee4e17304f29/msg/QuadGoal.msg) published by FASTER. The estimator (or the motion capture) should then publish the current state of the UAV as a [State message](https://gitlab.com/mit-acl/fsw/snap-stack/snapstack_msgs/-/blob/master/msg/State.msg). If you are using a ground robot, you need to publish a [nav_msgs/Odometry message](http://docs.ros.org/en/api/nav_msgs/html/msg/Odometry.html) (see [this](https://github.com/mit-acl/faster/blob/1baccf08908ad5a049c9e3315e577b35214ce763/faster/scripts/goal_odom_to_cmd_vel_state.py#L218)), and it will be converted directly to a [State message](https://gitlab.com/mit-acl/fsw/snap-stack/snapstack_msgs/-/blob/master/msg/State.msg).


## Credits:
This package uses code from the [JPS3D](https://github.com/KumarRobotics/jps3d) and [DecompROS](https://github.com/sikang/DecompROS) repos (included in the `thirdparty` folder), so credit to them as well. 


## Issues when installing Gurobi:

If you find the error:
```
“gurobi_continuous.cpp:(.text.startup+0x74): undefined reference to
`GRBModel::set(GRB_StringAttr, std::__cxx11::basic_string<char,
std::char_traits<char>, std::allocator<char> > const&)'”
```
The solution is:

```bash
cd /opt/gurobi800/linux64/src/build  #Note that the name of the folder gurobi800 changes according to the Gurobi version
sudo make
sudo cp libgurobi_c++.a ../../lib/
```

## Issues with other possible errors:

You can safely ignore these terminal errors:
* `Error in REST request` (when using ROS Melodic)
* `[ERROR] [...]: GazeboRosControlPlugin missing <legacyModeNS> while using DefaultRobotHWSim, defaults to true.` (when using the ground robot)
* `[ERROR] [...]: No p gain specified for pid.  Namespace: /gazebo_ros_control/pid_gains/front_left_wheel.` (when using the ground robot)
