# Operator GUI Quickstart

This first GUI version adds two VM-side ROS 2 packages:

- `agv_delivery_manager`: owns delivery task state.
- `agv_operator_ui`: provides supervisor, workbench, and stores PyQt5 windows.

The Pi-side robot packages and Arduino code do not change.

## Build

Copy the new packages into the VM fleet workspace, then build:

```bash
cd ~/fleet_ws
colcon build --symlink-install --packages-select agv_delivery_manager agv_operator_ui
source install/setup.bash
```

If this workspace was previously built with merged layout, use:

```bash
colcon build --symlink-install --merge-install --packages-select agv_delivery_manager agv_operator_ui
source install/setup.bash
```

## Run

Start the existing fleet manager:

```bash
ros2 launch agv_fleet_manager fleet_system.launch.py
```

Start the delivery manager:

```bash
ros2 launch agv_delivery_manager delivery_manager.launch.py
```

Start the supervisor GUI:

```bash
ros2 launch agv_operator_ui supervisor_gui.launch.py
```

Start the stores GUI:

```bash
ros2 launch agv_operator_ui stores_gui.launch.py
```

Start a workbench GUI:

```bash
ros2 launch agv_operator_ui workbench_gui.launch.py workbench_id:=bench_1
```

Use `bench_2` or `bench_3` for the other workbench screens.

## Topic Flow

Workbench GUI publishes:

```text
/workbench/request_delivery
/workbench/confirm_received
```

Stores GUI publishes:

```text
/stores/confirm_loaded
```

Delivery manager publishes:

```text
/delivery/tasks
/delivery/state
/delivery/events
```

Delivery manager dispatches through the existing fleet manager:

```text
/fleet/dispatch
```

The GUIs do not bypass the fleet manager.

## Current Assumption

`agv_delivery_manager/config/delivery_config.json` currently uses:

```text
stores
bench_1
bench_2
bench_3
charger_1
charger_2
```

These node names must exist in the VM fleet map config before automatic delivery dispatch can move the robot.
