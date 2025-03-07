# Threshold Monitor

This ROS 2 package provides a node that monitors sensor values and detects when they cross a threshold.
It also detects when no fresh data has been received for a configurable time, marking the data as stale.

## Features

- Monitors a numerical topic and detects threshold crossings.
- Publishes an alert if the threshold is exceeded.
- Detects stale data if no messages are received within a configurable timeout.

## Topics

| Topic Name                 | Message Type           | Description                                    |
| -------------------------- | ---------------------- | ---------------------------------------------- |
| `/threshold_monitor/input` | `std_msgs/msg/Float32` | The monitored sensor values.                   |
| `/threshold_monitor/alert` | `std_msgs/msg/String`  | `"ALERT_TRUE"`, `"ALERT_FALSE"`, or `"STALE"`. |

## Installation

1. Copy the package into your ROS 2 workspace (NOTE: Skip this step if the package was installed using the studio boilerplace installer.):

   ```bash
   cd ~/ros2_ws/src/
   unzip /path/to/threshold_monitor.zip
   ```

2. Source the ROS 2 environment setup script:

   ```bash
   source /opt/ros/humble/setup.bash
   ```

   - This command sets up the necessary environment variables for ROS 2, including paths for executables, libraries, and package dependencies.

   - It ensures that ROS 2 commands such as `ros2`, `colcon`, and `ament` work correctly in the terminal.

   - This step is required every time a new terminal session is started unless it is added to the shell's profile (e.g., `.bashrc` or `.zshrc`).

3. Restrict ROS 2 communication to localhost:

   ```bash
   export ROS_LOCALHOST_ONLY=1
   ```

   - This setting ensures that ROS 2 nodes communicate only within the local machine and do not attempt to discover or communicate with nodes on a network.
   - It is particularly useful for development and debugging, especially when running nodes on a single machine or using tools like ros2 run and ros2 launch.
   - This helps prevent unintended network discovery issues and enhances security by restricting inter-process communication (IPC) to the localhost.
   - Without this setting, ROS 2 may use multicast-based discovery over the local network, which could cause unexpected behavior in certain environments.

4. Build the package:

   ```bash
   cd ~/ros2_ws
   colcon build --packages-select threshold_monitor
   ```

5. Source your workspace:
   ```bash
   source install/setup.bash
   ```

## Running the Node

To start the threshold monitor node, run:

```bash
ros2 run threshold_monitor threshold_monitor
```

### Configuration

The node accepts the following parameters:

| Parameter Name    | Type     | Description                                                                       |
| ----------------- | -------- | --------------------------------------------------------------------------------- |
| `threshold`       | `float`  | The threshold value to trigger alerts.                                            |
| `comparison_mode` | `string` | `above` (trigger when above threshold) or `below` (trigger when below threshold). |
| `stale_timeout`   | `float`  | Time in seconds before marking data as stale.                                     |

Example of running with custom parameters:

```bash
ros2 run threshold_monitor threshold_monitor --ros-args -p threshold:=40.0 -p comparison_mode:=below -p stale_timeout:=10.0
```

## License

This package is released under the Apache-2.0 license.
