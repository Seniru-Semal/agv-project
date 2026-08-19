# AGV Launch Commands Reference

This file keeps the launch commands for the current AGV project structure.

The recommended normal setup is:

- Raspberry Pi: run AGV1 robot-side bringup.
- VM/main control PC: run fleet manager and delivery manager.
- Supervisor PC: run supervisor GUI only.
- Stores PC: run stores GUI only.
- Workbench PCs: run their own workbench GUI only.
- Arduino: no ROS launch command; it runs the uploaded low-level controller code.

## Raspberry Pi: AGV1 Robot Side

Update Pi workspace from the Git repo:

```bash
cd ~/agv-project
git pull

rsync -av ~/agv-project/ros2_pi/agv1_ws_src/ ~/agv_ws/src/

cd ~/agv_ws
colcon build --symlink-install
source install/setup.bash
```

If the workspace uses merged install layout:

```bash
cd ~/agv_ws
colcon build --symlink-install --merge-install
source install/setup.bash
```

Recommended full AGV1 launch:

```bash
ros2 launch agv_bringup agv1_complete.launch.py
```

Useful modular AGV1 launches:

```bash
ros2 launch agv_bringup agv1_robot.launch.py
ros2 launch agv_bringup agv1_fleet_gate.launch.py
ros2 launch agv_bringup agv1_lidar.launch.py
ros2 launch agv_bringup agv1_safety.launch.py
```

Legacy/old full launch location, kept only for reference:

```bash
ros2 launch agv_obstacle_guard agv1_complete.launch.py
```

## VM: Fleet And Delivery System

Update VM workspace from the Git repo:

```bash
cd ~/agv-project
git pull

rsync -av ~/agv-project/ros2_vm/fleet_ws_src/ ~/fleet_ws/src/

cd ~/fleet_ws
colcon build --symlink-install
source install/setup.bash
```

If the workspace uses merged install layout:

```bash
cd ~/fleet_ws
colcon build --symlink-install --merge-install
source install/setup.bash
```

Recommended real VM/backend launch:

```bash
ros2 launch agv_fleet_bringup backend_system.launch.py
```

Compatibility alias for the same backend-only launch:

```bash
ros2 launch agv_fleet_bringup factory_system.launch.py
```

Run VM backend with the older fleet HMI also enabled:

```bash
ros2 launch agv_fleet_bringup backend_system.launch.py start_legacy_hmi:=true
```

One-PC lab/demo launch with backend plus supervisor, stores, and all workbench
GUIs:

```bash
ros2 launch agv_fleet_bringup demo_all_guis.launch.py
```

One-PC lab/demo launch with only bench 1 GUI:

```bash
ros2 launch agv_fleet_bringup demo_all_guis.launch.py start_bench_2:=false start_bench_3:=false
```

## VM: Modular Bringup Launches

Fleet manager core only:

```bash
ros2 launch agv_fleet_bringup fleet_core.launch.py
```

Fleet manager plus delivery manager:

```bash
ros2 launch agv_fleet_bringup backend_system.launch.py
```

Older alias for fleet manager plus delivery manager:

```bash
ros2 launch agv_fleet_bringup delivery_system.launch.py
```

Operator GUIs only, for testing on one PC:

```bash
ros2 launch agv_fleet_bringup operator_guis.launch.py
```

Operator GUIs with all three benches:

```bash
ros2 launch agv_fleet_bringup operator_guis.launch.py start_bench_2:=true start_bench_3:=true
```

## VM: Individual Existing Launches

These are useful when testing one part at a time.

Existing fleet manager launch:

```bash
ros2 launch agv_fleet_manager fleet_system.launch.py
```

Delivery manager only:

```bash
ros2 launch agv_delivery_manager delivery_manager.launch.py
```

Supervisor GUI only:

```bash
ros2 launch agv_operator_ui supervisor_gui.launch.py
```

Stores GUI only:

```bash
ros2 launch agv_operator_ui stores_gui.launch.py
```

Workbench 1 GUI:

```bash
ros2 launch agv_operator_ui workbench_gui.launch.py workbench_id:=bench_1
```

Workbench 2 GUI:

```bash
ros2 launch agv_operator_ui workbench_gui.launch.py workbench_id:=bench_2
```

Workbench 3 GUI:

```bash
ros2 launch agv_operator_ui workbench_gui.launch.py workbench_id:=bench_3
```

## Useful Topic Checks

List all active ROS 2 topics:

```bash
ros2 topic list
```

Check AGV1 mission state:

```bash
ros2 topic echo /agv_1/mission/state
```

Check AGV1 current node:

```bash
ros2 topic echo /agv_1/mission/current_node
```

Check AGV1 safety:

```bash
ros2 topic echo /agv_1/safety/ok
```

Check AGV1 obstacle state:

```bash
ros2 topic echo /agv_1/obstacle/state
```

Check AGV1 fleet route acknowledgement:

```bash
ros2 topic echo /agv_1/fleet/route_ack
```

Check fleet state:

```bash
ros2 topic echo /fleet/state
```

Check delivery tasks:

```bash
ros2 topic echo /delivery/tasks
```

## Normal Startup Order

Use this order for normal testing:

1. Start the Raspberry Pi AGV1 bringup:

```bash
ros2 launch agv_bringup agv1_complete.launch.py
```

2. Start the VM backend:

```bash
ros2 launch agv_fleet_bringup backend_system.launch.py
```

3. Confirm the VM can see AGV1 topics:

```bash
ros2 topic list | grep /agv_1
```

4. Start each operator GUI on its own station PC.

Supervisor PC:

```bash
ros2 launch agv_operator_ui supervisor_gui.launch.py
```

Stores PC:

```bash
ros2 launch agv_operator_ui stores_gui.launch.py
```

Workbench PC example:

```bash
ros2 launch agv_operator_ui workbench_gui.launch.py workbench_id:=bench_1
```

## Notes

- Stop old terminals before using `backend_system.launch.py`, otherwise duplicate nodes may run.
- Use `demo_all_guis.launch.py` only for lab testing on one PC.
- The delivery manager destination names must match the fleet map node names.
- For the final factory map, use `stores`, `bench_1`, `bench_2`, `bench_3`, and the real charger node names consistently.
