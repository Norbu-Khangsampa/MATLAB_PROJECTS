# MATLAB Projects

## PID Control of DC Motor Speed

`PID_Control_of_DC_Motor_Speed.m` models a DC motor as a second-order
transfer function and closes a PID speed loop around it. It compares the
open-loop motor with two sets of PID gains and reports how tuning changes the
transient response.

### Model

The motor's angular speed ω in response to armature voltage V is

```
ω(s)        K
──── = ─────────────────────
V(s)   (Js + b)(Ls + R) + K²
```

| Parameter | Symbol | Value |
|-----------|--------|-------|
| Armature resistance | R | 2 Ω |
| Armature inductance | L | 0.5 H |
| Motor constant | K | 0.05 V·s/rad |
| Rotor inertia | J | 0.02 kg·m² |
| Viscous friction | b | 0.001 N·m·s/rad |

### Controllers

| | Kp | Ki | Kd |
|---|---|---|---|
| Initial | 10 | 20 | 0.5 |
| Tuned | 30 | 50 | 1.2 |

### Results

Approximate results for a 1 rad/s step:

| Metric | Initial PID | Tuned PID |
|--------|-------------|-----------|
| Settling time (2%) | ~1.7 s | ~0.9 s |
| Overshoot | ~44 % | ~38 % |
| Steady-state error | ≈ 0 | ≈ 0 |

The tuned gains settle almost twice as fast with slightly less overshoot. The
integral term removes steady-state error in both cases. The open-loop motor,
by comparison, has a slow pole near s = −0.11 and takes tens of seconds to
settle.

The open-loop curve is scaled to unit DC gain so all responses share the same
1 rad/s target on one plot. Unscaled, the motor would settle at about
11 rad/s per volt.

### Running

Requires MATLAB with the **Control System Toolbox** (`tf`, `pid`,
`feedback`, `step`, `stepinfo`).

```matlab
run('PID_Control_of_DC_Motor_Speed.m')
```

The script prints the transfer function and a metrics table, and opens a
figure comparing the step responses.

### Ideas for extension

- Tune for lower overshoot (e.g. with `pidtune` or by reducing Ki)
- Add a load-torque disturbance and compare disturbance rejection
- Model actuator voltage saturation and add anti-windup
