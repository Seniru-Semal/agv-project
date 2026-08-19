# VM Bringup Quickstart

This package gives the VM side one clean place to launch the fleet and delivery
backend.

The existing fleet manager logic is unchanged. The bringup package only starts
the existing nodes in a cleaner structure.

## Build

Copy `agv_fleet_bringup` into the VM workspace, then build:

```bash
cd ~/fleet_ws
colcon build --symlink-install --packages-select agv_fleet_bringup
source install/setup.bash
```

If your workspace uses merged install layout:

```bash
colcon build --symlink-install --merge-install --packages-select agv_fleet_bringup
source install/setup.bash
```

## Recommended Production VM Launch

In the real factory setup, the VM should start only the backend:

- fleet manager
- delivery manager

The supervisor, stores, and workbench GUIs should run on their own PCs.

```bash
ros2 launch agv_fleet_bringup backend_system.launch.py
```

The older `factory_system.launch.py` command is kept only as a compatibility
alias. It now starts the backend only.

```bash
ros2 launch agv_fleet_bringup factory_system.launch.py
```

Start the older fleet HMI on the VM also:

```bash
ros2 launch agv_fleet_bringup backend_system.launch.py start_legacy_hmi:=true
```

## Operator Station Launches

Run these on the actual PC/tablet at each station.

Supervisor PC:

```bash
ros2 launch agv_operator_ui supervisor_gui.launch.py
```

Stores PC:

```bash
ros2 launch agv_operator_ui stores_gui.launch.py
```

Workbench 1 PC:

```bash
ros2 launch agv_operator_ui workbench_gui.launch.py workbench_id:=bench_1
```

Workbench 2 PC:

```bash
ros2 launch agv_operator_ui workbench_gui.launch.py workbench_id:=bench_2
```

Workbench 3 PC:

```bash
ros2 launch agv_operator_ui workbench_gui.launch.py workbench_id:=bench_3
```

## One-PC Demo Launch

Use this only for lab testing when all GUI windows are on the VM:

```bash
ros2 launch agv_fleet_bringup demo_all_guis.launch.py
```

## Modular Launches

Fleet manager only:

```bash
ros2 launch agv_fleet_bringup fleet_core.launch.py
```

Fleet manager plus delivery manager:

```bash
ros2 launch agv_fleet_bringup backend_system.launch.py
```

Operator GUIs only, for testing on one PC:

```bash
ros2 launch agv_fleet_bringup operator_guis.launch.py
```

Bench 2 and bench 3 GUIs can be enabled when needed:

```bash
ros2 launch agv_fleet_bringup operator_guis.launch.py start_bench_2:=true start_bench_3:=true
```

## Launch Arguments

| Argument | Default | Purpose |
| --- | --- | --- |
| `start_fleet` | `true` | Start the planned-route fleet manager. |
| `start_delivery` | `true` | Start the delivery manager. |
| `start_legacy_hmi` | `false` | Also start the older fleet HMI. |
| `start_supervisor` | `true` | Start supervisor GUI. |
| `start_stores` | `true` | Start stores GUI. |
| `start_bench_1` | `true` | Start workbench GUI instance 1. |
| `start_bench_2` | `false` | Start workbench GUI instance 2. |
| `start_bench_3` | `false` | Start workbench GUI instance 3. |
| `bench_1_id` | `bench_1` | Workbench ID for instance 1. |
| `bench_2_id` | `bench_2` | Workbench ID for instance 2. |
| `bench_3_id` | `bench_3` | Workbench ID for instance 3. |

## Suggested Launch Pattern

During AGV1 testing, use:

```bash
ros2 launch agv_fleet_bringup backend_system.launch.py
```

Only when testing all windows on the VM, use:

```bash
ros2 launch agv_fleet_bringup demo_all_guis.launch.py
```
