# TurtleBot 4 – A* Global Planner Plugin for Nav2 (ICTE4001 Practical 7)

A ROS 2 (Humble) C++ package that implements a custom **A\* global path planner** as a Nav2 `nav2_core::GlobalPlanner` plugin (`iar_astar_planner/IarAstarPlanner`). It is built on a NavFn-style navigation function (`NavFn`), using A\* propagation (`propAstar`) over the global costmap to compute a path from the robot's pose to the goal.

The package also ships a Nav2 navigation launch file and a parameter file that select this plugin in the `planner_server`.

---

## Features

- Nav2 global planner plugin loaded via `pluginlib` (`iar_astar_planner.xml`)
- Lifecycle-aware `configure` / `activate` / `deactivate` / `cleanup` implementation
- `createPlan()` validates the start and goal against the global costmap (off-map and lethal-obstacle goal checks), then runs A\* propagation and traces the path from the potential field
- NavFn core (`src/navfn.cpp`) with A\* propagation and gradient path extraction
- Navigation launch file (`launch/navigation_launch.py`) that starts the controller, smoother, planner, behaviour, BT navigator, waypoint follower and velocity smoother servers with a lifecycle manager

## Repository Structure

```
.
├── CMakeLists.txt
├── package.xml
├── iar_astar_planner.xml            # pluginlib plugin description
├── config/
│   └── nav2_params.yaml            # Nav2 parameters (planner_server uses IarAstarPlanner)
├── include/iar_astar_planner/
│   ├── iar_astar_planner.hpp       # IarAstarPlanner plugin class
│   └── navfn.hpp                   # NavFn navigation function
├── launch/
│   └── navigation_launch.py        # Nav2 navigation servers
└── src/
    ├── iar_astar_planner.cpp       # plugin implementation
    └── navfn.cpp                   # NavFn / A* propagation
```

## Requirements

- Ubuntu 22.04
- ROS 2 Humble
- Nav2 (`nav2_core`, `nav2_costmap_2d`, `nav2_util`, `nav2_msgs`, `nav2_common`)
- TurtleBot 4 simulator (Ignition Gazebo) or a real TurtleBot 4

```bash
sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup
```

## Build

```bash
mkdir -p ~/tb4_ws/src && cd ~/tb4_ws/src
git clone https://github.com/TurtleBot-programming/turtlebot_7.git tb4_cpp_prac7
cd ~/tb4_ws
colcon build --packages-select tb4_cpp_prac7
source install/setup.bash
```

## Usage

Start the robot (or simulator) with localisation and a map running, then launch the navigation stack:

```bash
ros2 launch tb4_cpp_prac7 navigation_launch.py use_sim_time:=true
```

Launch arguments: `namespace`, `use_sim_time`, `params_file`, `autostart`, `use_composition`, `container_name`, `use_respawn`, `log_level`.

Send a goal from RViz (**Nav2 Goal**) or with `ros2 action send_goal /navigate_to_pose ...`.

## Configuration

The planner is selected in `config/nav2_params.yaml`:

```yaml
planner_server:
  ros__parameters:
    planner_plugins: ["GridBased"]
    GridBased:
      plugin: "iar_astar_planner/IarAstarPlanner"
```

## Notes

- Planning fails (empty path) if the goal is off the costmap or in a lethal-obstacle cell, or if the start is off the costmap.
- The start cell is cleared in the costmap copy used for planning, since it is the robot's current position.
