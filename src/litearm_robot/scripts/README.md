# LiteArm demo scripts

These Python demos mirror the Panthera script layout, but use the LiteArm
Python serial driver in `teach/motor_driver.py` and the validated C++ gravity
compensation executables.

The top-level filenames match the Panthera SDK demo names, so the LiteArm
upper-computer Scripts panel can discover and run them.

Run from the workspace root or from this directory:

```bash
conda run --no-capture-output -n panthera python src/litearm_robot/scripts/0_robot_get_state.py --once
conda run --no-capture-output -n panthera python src/litearm_robot/scripts/0_robot_free_mode.py
conda run --no-capture-output -n panthera python src/litearm_robot/scripts/1_Joint_PD_hold.py --duration 5
conda run --no-capture-output -n panthera python src/litearm_robot/scripts/1_Joint_PosVel_control.py --side left --joint 2 --amplitude 0.15 --execute
conda run --no-capture-output -n panthera python src/litearm_robot/scripts/2_gravity_compensation_control.py --side both
```

Motion and torque demos are dry-run by default when launched manually. Add
`--execute` to enable hardware control. The web Scripts panel adds this flag
automatically for movement demos; the reset-zero demo remains protected.

Default config:

```text
/home/tk/Dual_LiteArm/litearm_backend/robot_param/litearm_arms.yaml
```

The scripts scan `/dev/ttyACM*` before opening the ports, matching the C++
LiteArm driver behavior.
