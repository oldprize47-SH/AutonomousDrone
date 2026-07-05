# Experiment Results

This folder reorganizes the final-report figures and selected log-derived plots by control topic. The goal is to make the repository read like a development report instead of a flat image dump.

## Categories

| Folder | Contents |
|---|---|
| `altitude-control/` | Altitude command tracking, altitude controller output signals, internal altitude-loop terms |
| `attitude-control/` | Roll-rate command, pitch-rate/torque response, yaw-rate response, yaw-rate internal signals |
| `position-guidance-control/` | PNG relative kinematics, collision-avoidance kinematics, PNG/yaw analysis, circular guidance path and mission plots |
| `vision-landing/` | Vision landing pipeline, helipad labeling/detection examples, final vision landing summary |
| `gcs-monitoring/` | GCS architecture, state machine, and Sub Station dashboard screenshots |

## Source

The report-extracted figures come from:

```text
D:\My_Projects\자율이동체 제어\report\자율이동체제어 최종보고서_완.docx
```

The log-derived plots and summary CSV files come from the original `GCS/data` folder. Full raw CSV logs are not included in this GitHub repository because of size and signal-to-noise.
