# Eurobot-2025-Main

## Setup Workspace Env
### Git
- `$ git clone git@github.com:DIT-ROBOTICS/Eurobot-2025-Main.git`
<!-- - `git checkout devel` -->
### Build Docker Environment
- `$ cd /docker`
- If you haven't build image
    - If Windows: `$ docker compose -f docker-compose-win.yml up -d --build`
    - If Linux: `$ docker compose -f docker-compose-lin.yml up -d --build`
- Already have image
    - If Windows: `$ docker compose -f docker-compose-win.yml up`
    - If Linux: `$ docker compose -f docker-compose-lin.yml up`
- On the robot
    - `$ docker compose up`
- `$ docker start main-ws`
### Open Work Space
- Attach Visual Studio Code
- `$ cd Eurobot-2025-Main-ws`
- `$ colcon build --symlink-install` for once
## How to Use Groot
### Install Groot (only the first time you open the project)
- `$ cd ~/groot`
- `$ ./install.sh`
### Open Groot
- `$ cd ~/groot`
- `$ ./groot.AppImage`
## Install micro ROS for one time
- `$ ./build_micro_ros.sh`

## bt_app_2025 & startup: Behavior Tree for Game
### Use `bt_m.cpp` to add BT node and execute the tree: 
- Use 
    ```c++
    factory.registerNodeType<${node_name}>("${node_name}");
    ``` 
    to register the nodes
- load the nodes into .xml file
- Use
    ```c++
    auto tree = factory.createTree("MainTree");
    ```
    to create tree. ("MainTree" is the tree you select)
- `$ ros2 run bt_app_test bt_ros2` to execute the tree

### Steps to add new nodes and tree
1. write new nodes
2. register the nodes in `bt_m.cpp`
3. `$ colcon build --packages-select bt_app_2025` to build the package
4. `$ ros2 run bt_app_2025 bt_m` to load the new nodes into .xml file
5. `$ ./groot.AppImage` to open groot
6. edit your tree, and save to load it into .xml file
7. `$ ros2 run bt_app_2025 bt_m` to execute the tree
### Use `bt_launch.py` to launch all the programs
- Select different plan and tree in `bt_m_config`. (bt_m_config store behavior tree config xml files)
- `ros2 launch bt_app_2025 bt_launch.py`
### Structure of package `bt_app_2025`:
```
├── CMakeLists.txt
├── bt_m_config
│   ├── Project.btproj
│   ├── bt_m_tree_node_model.xml
│   ├── bt_plan_a.xml
│   └── firmware_test.xml
├── include
│   └── bt_app_2025
│       ├── bt_nodes_firmware.h
│       ├── bt_nodes_navigation.h
│       ├── bt_nodes_others.h
│       └── bt_nodes_receiver.h
├── launch
│   └── bt_launch.py
├── package.xml
├── params
│   └── config_path.yaml
└── src
    ├── bt_m.cpp
    ├── bt_nodes_firmware.cpp
    ├── bt_nodes_navigation.cpp
    ├── bt_nodes_others.cpp
    └── bt_nodes_receiver.cpp
```
### Some briefly describe of the program files & BT nodes.
- `bt_nodes_firmware`: all the BT nodes about the firmware mission
- `bt_nodes_navigation`: 
    - `Navigation`: input the global position point of the goal with rad angle.
    - `Docking`: input the staging point and the offset code.
    - `Rotation`: input the degree that you want robot to rotate.
- `bt_nodes_receiver`: all BT nodes aim for receive message from other teams

### startup: Check everything then start the game
> For now, there's many things haven't complete. So some of the ckecking steps in the startup node is **skipped**.
- `st_point`: Used to choose initial pose of robot. Is setting as plan A now.
- `ready_feedback`: Receive StartUpSrv then set as True. Is setting as True initially.
- `team`: Used to choose yellow and blue team sides. Doesn't initialize now.

## Simulation
### Navigation(導航組)
- `$ ros2 launch navigation2_run sim_launch.py`
- `$ rviz2`, and then open demo.rviz
- Open the Foxglove connection program:
    `ros2 launch foxglove_bridge foxglove_bridge_launch.xml port:=8765`
- 
  ```
  ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose "{pose: {header: {stamp: {sec: 0, nanosec: 0}, frame_id: 'map'}, pose: {position: {x: 0.35, y: 1.7, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.707, w: 0.707}}}}"
  ```
### Rival Sim(導航組)
- `$ ros2 run rival_layer RivalSim --ros-args -p Rival_mode:=<rival mode>`
### rival_simulation(主程式)
> 有一個遙控器可以移動敵機
- `$ ros2 run rival_simulation my_teleop --ros-args -p Rival_mode:=<rival mode>`
> 有一棵 BT 可以寫腳本規劃敵機如何移動
- `$ ros2 launch rival_simulation bt_launch.py`
> 之後可以用來測試動態調整的效果
### firmware connection
- `$ use dmesg -w to search device location`
- `$ ros2 run micro_ros_agent micro_ros_agent serial -b 115200 -D <your device location>`

## Interfaces packages
### btcpp_ros2_interfaces
> It is created to stored all the self-defined message (msg, srv, action) that used by Eurobot-Main-2025. This can organize the self-defined message type easier.
- msg:
    - `NodeStatus`: Written in last year. No use for now.
- srv:
    - `StartUpSrv`: Received by startup node to start the robot. No use for now.
- action: 
    - `ExecuteTree`: No use for now.
    - `Navigation`: Used in bt_app_test. For simple navigaiton.

### opennav_docking_msgs
> An action interface provided by navigation
> Used to transmit doking message to navigation team

### example_interfaces
> Add this interfaces for basic communication test with the firmware

## Other Packages
### bt_app_test: Behavior Tree for Test
    給每個人自由測試沒用過的 BT node 功能，放在這邊是希望大家可以共享測試成果，並且如果比賽程式要用到可以非常方便的直接抄。
    等比賽程式版本穩定後，這個 package 會獨立出來變成另一個 repo，正式版本的主程式不會有這個 package 。

- `bt_action_node_lib`: write action nodes
- `decorator_node_lib`: write decorator nodes
- `bt_ros2.cpp`: Has the same function with `bt_m.cpp`