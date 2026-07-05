# Complementary Filter Study

This folder preserves the supporting complementary-filter work used as background for altitude estimation and sensor fusion in the drone project.

## Contents

| Path | Description |
|---|---|
| `matlab/` | MATLAB scripts for loading data, explaining logs, and running filter calculations |
| `simulink/` | Simulink models for complementary-filter experiments |
| `data-samples/` | Selected pressure/altitude/filter result logs |
| `docs/` | Plots and original presentation material |

## Context

The main drone controller includes altitude estimation logic based on pressure-to-altitude conversion, complementary filtering, and fading-memory filtering. This study folder keeps the experimental material that informed that design.

Representative result:

![Floor altitude average](docs/all_floor_average.png)
