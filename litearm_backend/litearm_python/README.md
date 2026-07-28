# LiteArm Python Control Files

These files are the Python-side preparation for Panthera-style in-process
control-mode switching.

When the upper computer is started with
`/home/tk/Dual_LiteArm/litearm_backend/backend_arms.sh`, the backend releases
its serial handles before starting these mode scripts:

```text
Gravity    -> run_script.py 2_gravity_compensation_control.py
Impedance  -> run_script.py dual_arm_impedance_compensation.py
Position   -> backend app.py position loop
```

The impedance calculator and MIT-output script share the Kp, Kd, gravity gains,
and torque limits from `robot_param/litearm_arms.yaml`.

Use the same conda environment as `litearm_backend/backend.sh`:

```bash
cd /home/tk/Dual_LiteArm/litearm_backend/litearm_python
conda run --no-capture-output -n panthera python dual_arm_impedance_calc.py --samples 3
```

Hardware-output scripts:

```bash
conda run --no-capture-output -n panthera python dual_arm_gravity_compensation.py
conda run --no-capture-output -n panthera python dual_arm_impedance_compensation.py
```

Dry-run torque printing:

```bash
conda run --no-capture-output -n panthera python dual_arm_gravity_compensation.py --dry-run
conda run --no-capture-output -n panthera python dual_arm_impedance_compensation.py --dry-run
```

Panthera-style entry names:

```bash
conda run --no-capture-output -n panthera python 2_gravity_compensation_control.py
conda run --no-capture-output -n panthera python 2_Jointimpendence_control_with_gra_pd.py
```

All joint values are radians. The impedance target is latched from the current
joint positions unless `--left-target` and `--right-target` are provided.
