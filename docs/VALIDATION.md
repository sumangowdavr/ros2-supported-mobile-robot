# Validation and scope

## Current local checks

The public package is copied from the supported ROS 2 / Gazebo baseline. The controllers, robot geometry and smoke-test acceptance conditions are preserved.

```bash
PYTHONPATH=src/beni_sim python3 -m pytest -v src/beni_sim/test
python3 validate_local.py
```

On November 2, 2026 (UTC), all 14 tests were collected and passed in a fresh publication copy. Both SDF files passed `ign sdf -k`, and the integration runner passed `bash -n`.

The unit suite contains 14 collected tests. It checks target standoff, heading wrapping, turn-before-drive behavior, nonfinite targets/commands, command and scan timeouts, missing/invalid scans, front/rear obstacle stops, a side obstacle and speed saturation.

The model checker parses Python and XML, compares the standalone robot with its world instance, checks joint references and inertias, and verifies wheel-axis direction.

## Recorded integration evidence — October 12, 2026

| Run | Forward odometry | Simulation interval | Positive turn | Result |
|---|---:|---:|---:|---|
| Headless Fortress | 0.752 m | 3.999 s | 1.176 rad | Smoke test passed |
| GUI Fortress | 0.755 m | 3.995 s | 1.178 rad | Smoke test passed after shutdown handling fix |

Both recorded runs checked clock progression, odometry, RGB camera, lidar and IMU output, initially upright IMU orientation, forward movement, turning, fresh messages and stopping after command expiry.

The environment was Ubuntu 22.04.5, ROS 2 Humble and Gazebo Fortress 6.18.0. The initial run inside a socket-restricted sandbox failed to receive topics; the two native runs above passed.

## Practical limits

- Motion measurements came from wheel odometry, without an independent ground-truth trajectory comparison.
- The runtime smoke test did not exercise target following, sensor-rate accuracy, camera calibration, route planning or obstacle-avoidance scenarios.
- The point follower receives coordinates; it does not identify a person in images.
- The support balls remain part of the model. This controller does not balance the robot without them.

These records describe the supported simulation prototype. No physical robot deployment is claimed.
