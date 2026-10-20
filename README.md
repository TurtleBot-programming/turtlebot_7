# TurtleBot 4 – AMCL Particle Filter Localisation (ICTE4001 Practical 7)

A ROS 2 (Humble) C++ package that runs a custom **Adaptive Monte Carlo Localisation (AMCL)** node for the TurtleBot 4. It is based on the Nav2 `nav2_amcl` implementation. The particle **resampling step** is written from scratch, using **KLD-sampling** to adapt the number of particles and **augmented MCL** (`w_slow` / `w_fast`) to recover from kidnapping.

The node localises the robot on a prebuilt occupancy-grid map of the ICTE4001 simulated office, using laser scans and odometry. It publishes the estimated pose, the particle cloud and the `map → odom` transform.

---

## Features

- Lifecycle-managed AMCL node (`iar_amcl`) built on `nav2_util::LifecycleNode`
- Custom `pf_resample()` implementation (`src/amcl_node.cpp`):
  1. Cumulative distribution of the old particle weights
  2. Augmented MCL: injects random particles with probability `max(0, 1 − w_fast / w_slow)`
  3. Importance (roulette-wheel) resampling from the cumulative distribution
  4. KLD-sampling bound to work out the required particle count `M` from the kd-tree bin count
  5. Weight normalisation of the new particle set
- Custom kd-tree helpers for particle histogramming and clustering (`src/iar_pf_kdtree.cpp`)
- Differential-drive motion model and likelihood-field laser model (from `nav2_amcl` plugins)
- Launch file that starts the map server, the AMCL node and the Nav2 lifecycle manager

## Repository Structure

```
.
├── CMakeLists.txt
├── package.xml
├── config/
│   └── localisation.yaml          # AMCL + map_server parameters
├── include/iar_amcl/
│   ├── amcl_node.hpp              # AmclNode class declaration
│   └── iar_pf_kdtree.hpp          # kd-tree helper declarations
├── launch/
│   └── localisation_launch.py     # map_server + iar_amcl + lifecycle manager
├── map/
│   ├── office_ICTE4001_sim.pgm    # occupancy grid image
│   └── office_ICTE4001_sim.yaml   # map metadata (0.05 m/px)
└── src/
    ├── main.cpp                   # node entry point
    ├── amcl_node.cpp              # AMCL node + custom resampling
    └── iar_pf_kdtree.cpp          # kd-tree implementation
```

## Requirements

- Ubuntu 22.04
- ROS 2 Humble
- Nav2 (`nav2_amcl`, `nav2_map_server`, `nav2_lifecycle_manager`, `nav2_util`, `nav2_msgs`)
- TurtleBot 4 simulator (Ignition Gazebo) or a real TurtleBot 4

```bash
sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup
```

## Build

```bash
mkdir -p ~/tb4_ws/src && cd ~/tb4_ws/src
git clone https://github.com/TurtleBot-programming/turtlebot_7.git tb4_cpp_prac6
cd ~/tb4_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-select tb4_cpp_prac6
source install/setup.bash
```

## Usage

1. Start the TurtleBot 4 simulation in the ICTE4001 office world (or bring up the real robot).
2. Launch localisation:

   ```bash
   ros2 launch tb4_cpp_prac6 localisation_launch.py use_sim_time:=true
   ```

3. Open RViz, add the `/map`, `/particle_cloud` and `/amcl_pose` displays, and set an initial pose with the **2D Pose Estimate** tool.
4. Drive the robot around (e.g. with `teleop_twist_keyboard`) and watch the particle cloud converge.

### Launch arguments

| Argument       | Default                                  | Description                       |
|----------------|------------------------------------------|-----------------------------------|
| `namespace`    | `''`                                     | Top-level namespace               |
| `map`          | `map/office_ICTE4001_sim.yaml`           | Map YAML file to load             |
| `use_sim_time` | `false`                                  | Use the Gazebo clock              |
| `params_file`  | `config/localisation.yaml`               | Parameter file for the nodes      |
| `autostart`    | `true`                                   | Automatically activate the nodes  |
| `log_level`    | `info`                                   | ROS log level                     |

### Topics and services

| Name                         | Type                                        | Direction |
|------------------------------|---------------------------------------------|-----------|
| `scan`                       | `sensor_msgs/LaserScan`                     | Sub       |
| `map`                        | `nav_msgs/OccupancyGrid`                    | Sub       |
| `initialpose`                | `geometry_msgs/PoseWithCovarianceStamped`   | Sub       |
| `amcl_pose`                  | `geometry_msgs/PoseWithCovarianceStamped`   | Pub       |
| `particle_cloud`             | `nav2_msgs/ParticleCloud`                   | Pub       |
| `tf` (`map → odom`)          | `tf2_msgs/TFMessage`                        | Pub       |
| `reinitialize_global_localization` | `std_srvs/Empty`                      | Service   |
| `request_nomotion_update`    | `std_srvs/Empty`                            | Service   |

## Key Parameters (`config/localisation.yaml`)

| Parameter                       | Value  | Meaning                                         |
|---------------------------------|--------|-------------------------------------------------|
| `min_particles` / `max_particles` | 500 / 2000 | Bounds on the adaptive particle count     |
| `pf_err`                        | 0.05   | KLD error bound ε                               |
| `pf_z`                          | 0.99   | Upper standard-normal quantile (1 − δ)          |
| `recovery_alpha_slow` / `fast`  | 0.0 / 0.0 | Augmented-MCL averaging rates (0 = disabled) |
| `alpha1` – `alpha5`             | 0.2    | Odometry motion-model noise                     |
| `max_beams`                     | 60     | Laser beams used per update                     |
| `z_hit` / `z_rand` / `z_short` / `z_max` | 0.5 / 0.5 / 0.05 / 0.05 | Sensor-model mixture weights |
| `update_min_d` / `update_min_a` | 0.25 m / 0.2 rad | Motion needed before a filter update  |

To try out kidnapped-robot recovery, set non-zero values such as `recovery_alpha_slow: 0.001` and `recovery_alpha_fast: 0.1`.

## How the Resampling Works

For each new particle, until the KLD bound is met:

- With probability `w_diff = max(0, 1 − w_fast / w_slow)`, a random pose is drawn uniformly over the free space of the map.
- Otherwise, a random number `r ∈ [0, W_total)` is drawn and the old particle whose cumulative-weight interval contains `r` is copied.
- The particle is inserted into the kd-tree. With `k` occupied bins, the required sample count is

  ```
  M = (k − 1) / (2ε) · [1 − 2 / (9(k − 1)) + sqrt(2 / (9(k − 1))) · z]³
  ```

  clamped to at least `min_particles`. Resampling stops once the sample count exceeds `M` (or reaches `max_particles`).

All new particles are then given equal weight and normalised.

## Author

Yasiru Fernando — Curtin University Colombo, ICTE4001, Year 3

Based on the Nav2 AMCL implementation (Apache 2.0 / LGPL).
