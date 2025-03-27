# Simple Reactive Robots

**Intelligent Robotics Topics 2024/2025 | Masters in Artificial Intelligence**

Faculty of Engineering of University of Porto

## Group A1G

| Name           | Email                 |
| -------------- | --------------------- |
| Athos Freitas  | up202108792@fe.up.pt  |
| Luís Du       | up202105385@fe.up.pt  |
| Gonçalo Costa | up2002108814@fe.up.pt |

## Requirements

Make sure to have an environment with ROS2 and webots installed. In our case, the docker container initialized as described in moodle.

## **How to run project**

1. Clone the repository

```bash
git clone https://github.com/athoscf/TRI-assignment1
```

2. Run the docker (optional)

```bash
docker run --rm -it --gpus all  -e DISPLAY=$DISPLAY  -v /tmp/.X11-unix:/tmp/.X11-unix  -v /path_to_ROS2_workspace_in_local/:/path_to_workspace_in_docker tribe_simulation
```

3. Build and launch the package
   
```bash
cd
cd cd webots_ros2
colcon build --packages-select firstrobot_webots
source install/setup.bash
ros2 launch firstrobot_webots robot_launch.py
```

## **Directory Structure**

```
TRI-ASSIGNMENT1/
├── firstrobot_webots/
│   ├── build/
│   ├── include/
│   │   └── firstrobot_webots/
│   │       ├── MyRobotDriver.hpp
│   │       ├── ObstacleAvoider.hpp
│   │       └── RobotSeeker.hpp
│   ├── install/
│   ├── launch/
│   │   └── robot_launch.py
│   ├── log/
│   ├── resource/
│   │   ├── robot1.urdf
│   │   └── robot2.urdf
│   ├── src/
│   │   ├── MyRobotDriver.cpp
│   │   ├── ObstacleAvoider.cpp
│   │   └── RobotSeeker.cpp
│   ├── worlds/
│   │   ├── my_world.wbt
│   │   └── my_world2.wbt
│   ├── CMakeLists.txt
│   ├── LICENSE
│   ├── my_robot_driver.xml
│   ├── package.xml
└── README.md
```


### Folder Descriptions:
- **resource/**: Contains URDF files defining the structure and properties of the robots.  
- **src/**: Contains the implementation logic for robot control, including movement and obstacle avoidance.  
- **worlds/**: Defines the simulation environment for Webots, including terrain and object placement.  

## Demo


https://github.com/user-attachments/assets/54b933f0-c918-4246-a5db-08c0af35a644


