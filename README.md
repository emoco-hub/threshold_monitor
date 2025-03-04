# Threshold Monitor

This ROS 2 package provides a node that monitors sensor values and detects when they cross a threshold. 
It also detects when no fresh data has been received for a configurable time, marking the data as stale.

## Features

- Monitors a numerical topic and detects threshold crossings.
- Publishes an alert if the threshold is exceeded.
- Detects stale data if no messages are received within a configurable timeout.

## Topics

| Topic Name                  | Message Type                | Description |
|-----------------------------|----------------------------|-------------|
| `/threshold_monitor/input`  | `std_msgs/msg/Float32`     | The monitored sensor values. |
| `/threshold_monitor/alert`  | `std_msgs/msg/String`      | `"ALERT_TRUE"`, `"ALERT_FALSE"`, or `"STALE"`. |

## Installation

1. Copy the package into your ROS 2 workspace:
   ```bash
   cd ~/ros2_ws/src/
   unzip /path/to/threshold_monitor.zip
   ```

2. Build the package:
   ```bash
   cd ~/ros2_ws
   colcon build --packages-select threshold_monitor
   ```

3. Source your workspace:
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

| Parameter Name    | Type  | Description |
|------------------|-------|-------------|
| `threshold`      | `float` | The threshold value to trigger alerts. |
| `comparison_mode` | `string` | `above` (trigger when above threshold) or `below` (trigger when below threshold). |
| `stale_timeout`  | `float` | Time in seconds before marking data as stale. |

Example of running with custom parameters:

```bash
ros2 run threshold_monitor threshold_monitor --ros-args -p threshold:=40.0 -p comparison_mode:=below -p stale_timeout:=10.0
```

## License

This package is released under the Apache-2.0 license.
