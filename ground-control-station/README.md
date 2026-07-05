# Ground Control Station

This folder contains the Windows C++ ground-control station for the autonomous quadrotor project.

## Features

- Serial communication with the flight controller
- P8 telemetry and debug-frame parsing
- Mission waypoint upload/receive flow
- Realtime ImGui dashboard
- NTRIP correction client
- OptiTrack UDP receive path
- Flight/debug CSV logging

## Build

```powershell
cmake -S . -B build
cmake --build build --config Release
```

The project vendors the minimum Dear ImGui source files required for the dashboard. The original CMake file can also fetch ImGui v1.91.5 if `third_party/imgui` is missing.

## Main Files

| File | Purpose |
|---|---|
| `gcs_main.cpp` | Main runtime loop, command handling, mission receive/save, logging |
| `src/p8_comm.cpp` | Parser and packet sender for the P8 protocol |
| `src/dashboard.cpp` | ImGui dashboard UI |
| `src/ntrip_client.cpp` | NTRIP client |
| `src/OptiTrack.cpp` | OptiTrack receiver |
| `WayPoint/` | Mission waypoint text files |
