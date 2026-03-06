# TEACar ROS2

## Setup
### Install ros2 humble or newer version
Follow the guideline [here](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html)
### Clone the repo
```bash
git clone https://github.com/Trustworthy-Engineered-Autonomy-Lab/TEACar-Open-Source-Autonomous-Driving-Platform.git
```
### Build the workspace
```
cd TEACar-Open-Source-Autonomous-Driving-Platform
colcon build
source install/setup.bash
```

## Usage
### Drive the car
```bash
ros2 launch  drive.launch.py
```
