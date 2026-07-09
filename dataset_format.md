# Dataset To Collect

Collect one CSV file per independent single-motor run.

Required CSV columns:

```text
time_s,qpos,qvel,qact
0.000,0.12,0.01,0.30
0.002,0.13,0.02,0.31
0.004,0.15,0.03,0.29
```

Column meaning:

- `time_s`: timestamp, seconds (`s`).
- `qpos`: measured motor position, radians (`rad`).
- `qvel`: measured motor velocity, radians per second (`rad/s`).
- `qact`: commanded target/action sent to the motor, radians (`rad`).

Rules:

- Each CSV is one continuous run.
- Rows must be ordered by time.
- Use the same fixed sampling period inside a run.
- Do not include lagged/history columns.
- Do not mix different physical setups inside one CSV. If load, weight, link length, friction, or another physical condition changes, start a new CSV file.
- Each CSV should contain one motor only.

Example files:

```text
raw/load30_run_001.csv
raw/load30_run_002.csv
raw/load40_run_001.csv
```

The code will build history windows and future prediction targets from these raw runs automatically.
