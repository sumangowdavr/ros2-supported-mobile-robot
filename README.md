# Supported mobile robot in ROS 2 and Gazebo

A small differential-drive robot prototype for **ROS 2 Humble and Gazebo Fortress**. It combines a simulated mobile base, RGB camera, lidar, IMU and odometry with guarded velocity commands and a point-following controller.

The approximate geometry is inspired by BENI, with two passive support balls providing stability. This is a supported mobile base; self-balancing and jumping are outside this implementation.

## What the project implements

- An original SDF robot model and a furnished 10 × 10 m room.
- ROS/Gazebo bridges for camera, lidar, IMU, odometry and simulation time.
- A velocity-command guard with input and scan timeouts, speed limits, and front/rear obstacle stopping.
- A point-following controller that turns toward an externally supplied coordinate and stops at a 1 m standoff.
- Fourteen controller tests, model checks and a reproducible Gazebo smoke-test runner.

```text
teleoperation or target-point follower
                 │
                 ▼
          /cmd_vel_input
                 │
        command and scan guard ◄── lidar
                 │
                 ▼
             /cmd_vel
                 │
                 ▼
        Gazebo differential drive
                 │
       odometry / camera / IMU
```

The follower consumes a target coordinate supplied by another component. Image-based person detection, SLAM, Nav2 and route planning are future integration work.

## Verification

| Check | Evidence |
|---|---|
| Controller unit tests | 14 tests covering timeouts, missing/invalid scans, obstacle stopping, speed limits and target following |
| Model checks | Python syntax, XML parsing, matching standalone/world robot models, joint references, positive inertias and wheel axes |
| Recorded Gazebo integration | October 12, 2026: sensor output, forward movement, positive turning and watchdog stopping passed in headless and GUI runs |

The recorded GUI smoke test measured **0.755 m** of wheel-odometry travel over **3.995 simulation seconds** and a **1.178 rad** turn. These are odometry measurements. The smoke test did not establish ground-truth trajectory accuracy or obstacle-avoidance performance.

See [validation details](docs/VALIDATION.md) for the checks and their limits.

## Requirements

Ubuntu 22.04, ROS 2 Humble and Gazebo Fortress / Ignition Gazebo 6. The runtime bridge package is `ros-humble-ros-gz`. Camera and lidar rendering require working OpenGL/EGL.

## Build and run

```bash
git clone https://github.com/sumangowdavr/ros2-supported-mobile-robot.git
cd ros2-supported-mobile-robot
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select beni_sim
source install/setup.bash
ros2 launch beni_sim sim.launch.py
```

For a headless world, add `gui:=false` to the launch command.

## Run the checks

The controller and model checks run without starting Gazebo:

```bash
PYTHONPATH=src/beni_sim python3 -m pytest -v src/beni_sim/test
python3 validate_local.py
```

After building, run the isolated Gazebo acceptance test:

```bash
bash scripts/runtime_test.sh
```

The runner saves its logs under `runtime_results/` and stops the simulation when finished. It uses its own Gazebo transport partition and localhost ROS domain 68; `BENI_TEST_DOMAIN_ID` overrides the domain.

## Try point following

With a fresh simulation running, use two terminals sourced with ROS and `install/setup.bash`:

```bash
ros2 run beni_sim follow_target
```

```bash
ros2 topic pub -r 5 /target_point geometry_msgs/msg/PointStamped \
  "{header: {frame_id: odom}, point: {x: 3.0, y: 0.0, z: 0.0}}"
```

Commands pass through `/cmd_vel_input` and the guard before reaching the simulated base. Run one command producer at a time. The guard provides local stopping behavior; it does not plan around obstacles.

## Repository layout

```text
src/beni_sim/    ROS package, model, world, controllers and tests
scripts/        Fresh-world integration test runner
docs/           Validation evidence and scope
validate_local.py
```

MIT licensed. This repository contains the supported-base prototype and preserves the original contributor notice.
