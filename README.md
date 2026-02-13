# icir_tutorial_pinocchio
CNU icir lab tutorial package for pinocchio library and mujoco simulation

All source codes are opimized for ROS-Noetic.

## 1. Install
# required installation dependencies
```bash
sudo apt install git curl cmake-curses-gui 
sudo apt install ros-noetic-moveit-visual-tools
sudo apt install nlohmann-json3-dev 
sudo apt install ros-noetic-moveit 
sudo apt install libqt53dextras5
sudo apt install qtbase5-private-dev
```

# install pinocchio : add robotpkg apt repository (https://stack-of-tasks.github.io/pinocchio/download.html)
```bash
sudo apt install -qqy lsb-release gnupg2 curl
sudo mkdir -p /etc/apt/keyrings
curl http://robotpkg.openrobots.org/packages/debian/robotpkg.asc | sudo tee /etc/apt/keyrings/robotpkg.asc
echo "deb [arch=amd64 signed-by=/etc/apt/keyrings/robotpkg.asc] http://robotpkg.openrobots.org/packages/debian/pub $(lsb_release -cs) robotpkg" | sudo tee /etc/apt/sources.list.d/robotpkg.list

sudo apt update 

sudo apt install -qqy robotpkg-py3*-pinocchio
sudo apt install robotpkg-py38-eigenpy
sudo apt install robotpkg-eiquadprog

echo "export PATH=/opt/openrobots/bin:$PATH" >> ~/.bashrc
echo "export PKG_CONFIG_PATH=/opt/openrobots/lib/pkgconfig:$PKG_CONFIG_PATH" >> ~/.bashrc
echo "export LD_LIBRARY_PATH=/opt/openrobots/lib:$LD_LIBRARY_PATH" >> ~/.bashrc
echo "export PYTHONPATH=/opt/openrobots/lib/python3.8/site-packages:/opt/openrobots/lib/python3/dist-packages:$PYTHONPATH" >> ~/.bashrc # Adapt your desired python version here
echo "export CMAKE_PREFIX_PATH=/opt/openrobots:$CMAKE_PREFIX_PATH" >> ~/.bashrc
```

# mujoco license
```bash
#ctrl+h to see hidden folder
mkdir .mujoco 
#copy mjkey.txt to .mujoco folder
```

## 2. Prerequisites
```bash
git clone https://github.com/iCIRLab/icir_gen3_robot_description.git
git clone https://github.com/iCIRLab/icir_mujoco_ros.git 
git clone https://github.com/iCIRLab/icir_tutorial_pinocchio.git
```

## 3. Run
```bash
roslaunch icir_tutorial_pinocchio icir_tutorial_pinocchio_simulation.launch 

from terminal, press 'h' to move the robot to the home pose
from terminal, press 'a' to move the robot to another pose

from terminal, press 'k' to move the robot to -0.05m in global z direction
from terminal, press 'v' to enable cartesian impedance control in translational direction
```