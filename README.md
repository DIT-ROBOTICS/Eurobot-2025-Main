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
- `$ source /opt/ros/humble/setup.bash`
- `$ colcon build --symlink-install`
<!-- ### Some Other Steps
- `echo "source /home/user/Eurobot-2025-Main-ws/devel/setup.bash" >> ~/.bashrc`
- `echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc`
- `source ~/.bashrc` -->

## Features
### Open Groot
- `$ cd ~/groot`
- `$ .\groot.AppImage`
### Behavior Tree for Test
- Open folder `bt_app`
- write action nodes in `bt_action_node_lib`
- write decorator nodes in `decorator_node_lib`
- `$ ros2 run bt_app bt_ros2` to execute the tree
### Behavior Tree for Game
- Open folder `bt_application_2024`
- write bt nodes in `bt_nodes`
- `$ ros2 run bt_application_2024 bt_m` to execute the tree
### run sim world
- `$ export TURTLEBOT3_MODEL=burger`
- `$ ros2 launch turtlebot3_gazebo empty_world.launch.py`