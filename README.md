# UAV Simulation

## Project Overview
This project simulates a UAV with ROS 2 through PX4 using C++. It offers two main capabilities: manual control and offboard control (automatic control).

## Project setup

### Step 1: Basic setup

- Install [ROS 2 Humble](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html) on Ubuntu 22.04

- Install [colcon]() to build ROS 2 packages (if it is not already installed)

### Step 2: Download firmware

- Download [PX4 firmware](https://docs.px4.io/main/en/dev_setup/building_px4.html) (v1.15.1)
```
git clone --branch v1.15.1 --recursive https://github.com/PX4/PX4-Autopilot.git
```

- Install [Gazebo-Classic 11](https://docs.px4.io/main/en/sim_gazebo_classic/)
```
sudo apt remove gz-harmonic
sudo apt install aptitude
sudo aptitude install gazebo libgazebo11 libgazebo-dev
sudo apt install libopencv-dev protobuf-compiler libeigen3-dev libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev
```

- Install [XRCE-DDS](https://docs.px4.io/main/en/ros2/user_guide.html#setup-micro-xrce-dds-agent-client) (bridge between PX4 and ROS 2)
```
git clone -b master https://github.com/eProsima/Micro-XRCE-DDS-Agent.git
cd Micro-XRCE-DDS-Agent
mkdir build
cd build
cmake ..
make
sudo make install
sudo ldconfig /usr/local/lib/
```

- Clone the [uav_simulation](https://github.com/mmhai202/uav_simulation) project

### Step 3: Build firmware

- Build PX4 firmware (Using a Simulator SITL)
```
cd PX4-Autopilot
make px4_sitl_default gazebo-classic_iris
```

- Build uav_simualtion project
```
cd uav_simualtion
colcon build
source install/setup.bash
```

### Step 4: Run project

Open some terminals

- Terminal 1: run PX4 firmware
```
cd PX4-Autopilot
make px4_sitl_default gazebo-classic_iris
```

- Terminal 2: run the comunication of ROS 2 and PX4
```
MicroXRCEAgent udp4 -p 8888
```

- Terminal 3: run uav_simulation project
----- for offboard_control -----
```
cd uav_simualtion
colcon build
source install/setup.bash
ros2 run drone_control offboard_control
```
----- or for manual control -----
```
cd uav_simualtion
colcon build
source install/setup.bash
ros2 run drone_control manual_control
```