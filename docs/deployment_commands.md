# AGV Deployment Commands

This file keeps the command set for the current AGV system.

It avoids private details such as IP addresses, passwords, tokens, and local
network configuration.

## Production Layout

| Location | Runs |
| --- | --- |
| Raspberry Pi on AGV1 | Robot-side ROS 2 bringup |
| VM / main control PC | Fleet manager and delivery manager backend |
| Supervisor PC | Supervisor GUI only |
| Stores PC | Stores GUI only |
| Workbench PCs | One workbench GUI per station |
| Arduino | Uploaded low-level controller code |

## Raspberry Pi / AGV1

Use this on the Raspberry Pi:

```bash
cd ~/agv-project
git pull

rsync -av ~/agv-project/ros2_pi/agv1_ws_src/ ~/agv_ws/src/

cd ~/agv_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash

ros2 launch agv_bringup agv1_complete.launch.py
```

If the Pi workspace uses merged install layout:

```bash
cd ~/agv_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --merge-install
source install/setup.bash

ros2 launch agv_bringup agv1_complete.launch.py
```

## VM / Main Control PC Backend

Use this on the VM/main control PC:

```bash
cd ~/agv-project
git pull

rsync -av ~/agv-project/ros2_vm/fleet_ws_src/ ~/fleet_ws/src/

cd ~/fleet_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash

ros2 launch agv_fleet_bringup backend_system.launch.py
```

If the VM workspace uses merged install layout:

```bash
cd ~/fleet_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --merge-install
source install/setup.bash

ros2 launch agv_fleet_bringup backend_system.launch.py
```

## Supervisor PC

Use this in a new terminal on the supervisor PC:

```bash
cd ~/fleet_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch agv_operator_ui supervisor_gui.launch.py
```

## Stores PC

Use this in a new terminal on the stores PC:

```bash
cd ~/fleet_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch agv_operator_ui stores_gui.launch.py
```

## Workbench PCs

Workbench 1:

```bash
cd ~/fleet_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch agv_operator_ui workbench_gui.launch.py workbench_id:=bench_1
```

Workbench 2:

```bash
cd ~/fleet_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch agv_operator_ui workbench_gui.launch.py workbench_id:=bench_2
```

Workbench 3:

```bash
cd ~/fleet_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch agv_operator_ui workbench_gui.launch.py workbench_id:=bench_3
```

## One-PC Lab Demo Only

Use this only when testing all GUI windows on one machine.

Do not run this while `backend_system.launch.py` is already running, because it
also starts the backend.

```bash
cd ~/fleet_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch agv_fleet_bringup demo_all_guis.launch.py
```

## Quick Checks

Check AGV1 topics:

```bash
source /opt/ros/humble/setup.bash
ros2 topic list | grep /agv_1
```

Check duplicate delivery managers:

```bash
source /opt/ros/humble/setup.bash
ros2 node list | grep delivery
```

There should be only one:

```text
/delivery_manager_node
```

Check delivery task stream:

```bash
source /opt/ros/humble/setup.bash
ros2 topic echo /delivery/tasks
```

Check fleet state:

```bash
source /opt/ros/humble/setup.bash
ros2 topic echo /fleet/state
```

## Cleanup During Testing

If duplicate backend nodes were accidentally started:

```bash
pkill -f delivery_manager_node
pkill -f planned_route_fleet_manager_node
```

Then restart the backend once:

```bash
cd ~/fleet_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch agv_fleet_bringup backend_system.launch.py
```

## Privacy Rules

Do not commit:

- GitHub tokens or passwords
- Wi-Fi passwords
- SSH private keys
- IP addresses if they are not necessary for the project
- ROS logs
- `build/`, `install/`, or `log/` folders
- `__pycache__/` folders or `.pyc` files
- temporary machine-specific notes
