# Eurobot-2025-Main

## Setup Workspace Env
### Git
- `$ git clone git@github.com:DIT-ROBOTICS/Eurobot-2025-Main.git`
<!-- - `git checkout devel` -->
### Build Docker Environment
- `$ cd /docker`
- `$ docker compose up -d --build`
- `$ docker start main-ws`
### Open Work Space
- Attach Visual Studio Code
- `$ cd Eurobot-2025-Main-ws`
- `$ colcon build --symlink-install` for once
## How to Use Groot
### Install Groot
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

- Work flow to add new nodes and tree
    1. write new nodes in `bt_action_node_lib` or `decorator_node_lib`
    2. register the nodes in `bt_ros2`
    3. `$ colcon build --packages-select bt_app_test` to build the package
    4. `$ ros2 run bt_app_test bt_ros2` to load the new nodes into .xml file
    5. `$ ./groot.AppImage` to open groot
    6. edit your tree, and save to load it into .xml file
    7. `$ ros2 run bt_app_test bt_ros2` to execute the tree

### bt_app_2025: Behavior Tree for Game
    所有比賽的程式都寫在這，暫時先不要更改

### run sim world (no usage)
- `$ export TURTLEBOT3_MODEL=burger`
- `$ ros2 launch turtlebot3_gazebo empty_world.launch.py`