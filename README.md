# Eurobot-2025-Main

## Setup Workspace Env
### Git
- `$ git clone git@github.com:DIT-ROBOTICS/Eurobot-2025-Main.git`
<!-- - `git checkout devel` -->
### Build Docker Environment
- `$ cd /docker`
- If you haven't build image
    - If Windows: `$ docker-compose -f docker-compose-win.yml up -d --build`
    - If Linux: `$ docker-compose -f docker-compose-lin.yml up -d --build`
- Already have image
    - If Windows: `$ docker-compose -f docker-compose-win.yml up`
    - If Linux: `$ docker-compose -f docker-compose-lin.yml up`
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

## Features of Each Package
### bt_app_test: Behavior Tree for Test
    給每個人自由測試沒用過的 BT node 功能，放在這邊是希望大家可以共享測試成果，並且如果比賽程式要用到可以非常方便的直接抄。
    等比賽程式版本穩定後，這個 package 會獨立出來變成另一個 repo，正式版本的主程式不會有這個 package 。

- `bt_action_node_lib`: write action nodes
- `decorator_node_lib`: write decorator nodes
- `bt_ros2`: 
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
        to create tree
    - `$ ros2 run bt_app_test bt_ros2` to execute the tree

- Steps to add new nodes and tree
    1. write new nodes in `bt_action_node_lib` or `decorator_node_lib`
    2. register the nodes in `bt_ros2`
    3. `$ colcon build --packages-select bt_app_test` to build the package
    4. `$ ros2 run bt_app_test bt_ros2` to load the new nodes into .xml file
    5. `$ ./groot.AppImage` to open groot
    6. edit your tree, and save to load it into .xml file
    7. `$ ros2 run bt_app_test bt_ros2` to execute the tree

### bt_app_2025: Behavior Tree for Game
- Structure of package `bt_app_2025`
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
- bt_m_config: behavior tree config xml files
- launch: 
    - modify parameter `tree_name` as `NavTest` or `MainTree`
    - `ros2 launch bt_app_2025 bt_launch.py`
- params: yaml param files
- `bt_nodes_firmware`: 
    - `BTMission`:與韌體溝通的 action node
    - `CollectFinisher`: 完成一個區域的材料拿取後的統整，還沒寫
    - `ConstructFinisher`: 完成一個區域的看台建置後的統整，還沒寫
    - `SIMAactivate`: 原本寫來要啟動 SIMA 用的，但應該不需要了
- `bt_nodes_navigation`: 
    - `Navigation`: 輸入全域座標&弧度(rad)。機器移動並旋轉
    - `Docking`: 輸入起始位置&移動距離。機器移動
    - `Rotation`: 輸入角度(deg)。機器旋轉
    - `DynamicAdjustment`: 動態調整下一個目標點，還沒寫
- `bt_nodes_others`: 
    - `BTStarter`: 會持續接收比賽時間
    - `PointProvider`: 輸入一個移動距離，輸出這個距離的正反兩種方向
    - `TimerChecker`: 確認現在比賽時間
    - `BTFinisher`: 去年的 node，目前沒有用到
    - `Comparator`: 去年的 node，目前沒有用到
- `bt_nodes_receiver`: 
    - `NavReceiver`: 接收導航預判的敵機目標方向，儲存到 BT blackboard 提供給動態調整使用
    - `CamReceiver`: 接收相機資訊，然後整理成主程式需要的資料型態儲存到 BT blackbobard
    - `LocReceiver`: 接收定位組資訊並輸出我機位置&敵機位置
- `bt_m`: 用法與功能和 `bt_ros2` 相同

### btcpp_ros2_interfaces
> It is created to stored all the self-defined message (msg, srv, action) that used by Eurobot-Main-2025. This can organize the self-defined message type easier.
- msg:
    - `NodeStatus`: Written in last year. No use for now.
- srv:
    - `StartUpSrv`: Received by startup node to start the robot. No use for now.
- action: 
    - `ExecuteTree`: No use for now.
    - `Navigation`: Used in bt_app_test. For simple navigaiton.
    - `FirmwareMission`: Used in bt_app_2025. For mission firmware communication.

### startup: Check everything then start the game
> For now, there's many things haven't complete. So some of the ckecking steps in the startup node is **skipped**.
- `st_point`: Used to choose initial pose of robot. Is setting as plan A now.
- `ready_feedback`: Receive StartUpSrv then set as True. Is setting as True initially.
- `team`: Used to choose yellow and blue team sides. Doesn't initialize now.

## Simulation
### Navigation(導航組)
- `$ ros2 launch navigation2_run sim_launch.py`
- `$ rviz2` then open demo.rviz
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
