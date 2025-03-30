# Eurobot 2025 Main

## Workflow

1. Clone the repository:
   ```bash
   git clone https://github.com/DIT-ROBOTICS/Eurobot-2025-Main.git
   ```

2. Display the help message:
   ```bash
   ./main
   ```
   Output:
   ```
   ----- [ MAIN Usage ] ------------------------
   |   build           - Build the ROS workspace
   |   run             - Run the main program
   |   groot           - Launch Groot2
   |   enter           - Enter the container
   |   close           - Stop the container
   |   --command <cmd> - Run a custom command
   ---------------------------------------------
   ```

3. Build the workspace (both image and ROS):
   ```bash
   ./main build
   ```

4. Run the main program:
   ```bash
   ./main run
   ```

5. Launch Groot2 (only for Groot functionality):
   ```bash
   ./main groot
   ```

6. Enter the container (equivalent to `docker exec -it` but more powerful):
   ```bash
   ./main enter
   ```

7. Stop the container and retrieve all resources:
   ```bash
   ./main close
   ```
   Note: Resources are usually retrieved automatically after the program closes or crashes.

8. Run a custom command in the container (whether the container is running or not):
   ```bash
   ./main --command "your command"
   ```

## Notes for Groot Functionality

To use Groot, you need to first enter the container:
```bash
./main enter
```
Then, run the following command to pull the AppImage:
```bash
groot/install.sh
```
However, it is recommended to use the isolation application with backup functionality provided by [DIT-ROBOTICS/DIT-Tools](https://github.com/DIT-ROBOTICS/DIT-Tools).

## About `bt_app_2025`

### Using `bt_m.cpp` to Add BT Nodes and Execute the Tree
1. Register nodes:
   ```c++
   factory.registerNodeType<${node_name}>("${node_name}");
   ```
2. Load nodes into an `.xml` file.
3. Create the tree:
   ```c++
   auto tree = factory.createTree("MainTree");
   ```
   *(Replace `"MainTree"` with the desired tree name.)*
4. Execute the tree:
   ```bash
   $ ros2 run bt_app_test bt_ros2
   ```

### Steps to Add New Nodes and Trees
1. Write new nodes.
2. Register the nodes in `bt_m.cpp`.
3. Build the package:
   ```bash
   $ colcon build --packages-select bt_app_2025
   ```
4. Load the new nodes into an `.xml` file:
   ```bash
   $ ros2 run bt_app_2025 bt_m
   ```
5. Open Groot:
   ```bash
   ./main groot
   ```
6. Edit and save your tree in Groot.
7. Execute the tree:
   ```bash
   $ ros2 run bt_app_2025 bt_m
   ```

### Using `bt_launch.py` to Launch All Programs
1. Select the desired plan and tree in `bt_m_config` (stores behavior tree configuration `.xml` files).
2. Launch the programs:
   ```bash
   $ ros2 launch bt_app_2025 bt_launch.py
   ```

### Package Structure: `bt_app_2025`
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

### Program Files & BT Nodes Overview
- **`bt_nodes_firmware`**: BT nodes related to firmware missions.
- **`bt_nodes_navigation`**:
  - `Navigation`: Input global position and goal orientation.
  - `Docking`: Input staging point and offset code.
  - `Rotation`: Input desired rotation angle.
- **`bt_nodes_receiver`**: BT nodes for receiving messages from other teams.

### `startup`: Check Everything Before Starting the Game
- **`st_point`**: Sets the robot's initial pose (currently set to Plan A).
- **`ready_feedback`**: Receives `StartUpSrv` and sets it to `True` (currently initialized as `True`).
- **`team`**: Chooses between yellow and blue team sides (not initialized yet).

## Simulation

### Navigation
1. Launch the simulation:
   ```bash
   $ ros2 launch navigation2_run sim_launch.py
   ```
2. Open RViz:
   ```bash
   $ rviz2
   ```
   *(Load `demo.rviz`.)*
3. Open the Foxglove connection program:
   ```bash
   $ ros2 launch foxglove_bridge foxglove_bridge_launch.xml port:=8765
   ```
4. Send a navigation goal:
   ```bash
   ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose "{pose: {header: {stamp: {sec: 0, nanosec: 0}, frame_id: 'map'}, pose: {position: {x: 0.35, y: 1.7, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.707, w: 0.707}}}}"
   ```

### Rival Simulation
- Run the rival simulation:
  ```bash
  $ ros2 run rival_layer RivalSim --ros-args -p Rival_mode:=<rival mode>
  ```

### Rival Simulation (Main Program)
- **Remote Control**: Move the enemy robot manually:
  ```bash
  $ ros2 run rival_simulation my_teleop --ros-args -p Rival_mode:=<rival mode>
  ```
- **Behavior Tree**: Script enemy robot movements:
  ```bash
  $ ros2 launch rival_simulation bt_launch.py
  ```

### Firmware Connection
1. Use `dmesg` to find the device location:
   ```bash
   $ dmesg -w
   ```
2. Run the micro-ROS agent:
   ```bash
   $ ros2 run micro_ros_agent micro_ros_agent serial -b 115200 -D <your device location>
   ```

## Interface Packages

### `btcpp_ros2_interfaces`
- Stores all custom message types (`msg`, `srv`, `action`) for better organization.
- **Messages**:
  - `NodeStatus`: Currently unused.
- **Services**:
  - `StartUpSrv`: Received by the startup node to start the robot (currently unused).
- **Actions**:
  - `ExecuteTree`: Currently unused.
  - `Navigation`: Used in `bt_app_test` for simple navigation.

### `opennav_docking_msgs`
- Action interface provided by the navigation team for docking messages.

### `example_interfaces`
- Added for basic communication testing with the firmware.

## Other Packages

### `bt_app_test`: Behavior Tree for Testing
- A shared space for testing new BT node functionalities. Once the competition program stabilizes, this package will be moved to a separate repository.
- **Components**:
  - `bt_action_node_lib`: Write action nodes.
  - `decorator_node_lib`: Write decorator nodes.
  - `bt_ros2.cpp`: Functions similarly to `bt_m.cpp`.