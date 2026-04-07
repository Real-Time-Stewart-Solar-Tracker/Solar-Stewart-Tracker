# Pipeline Demo

Demonstrates the full automatic tracking pipeline **without any hardware**.

## What it shows

```
SimulatedPublisher → SunTracker → Controller → Kinematics3RRS → stdout
```

A synthetic camera generates 640×480 frames at 30 fps with a bright spot that moves
slowly around the image.  Each stage processes the events in turn and the demo prints
one line per frame showing:

- the detected sun position (pixels) and confidence
- the computed tilt/pan setpoint (radians)
- the resulting servo angles for all 3 actuators (degrees)

## Expected output

```
=== Solar Stewart Tracker — Pipeline Demo ===
Running simulated pipeline for 5 seconds...

[frame    1] sun=( 320.0,  240.0) conf=1.00  →  tilt=+0.000 rad  pan=+0.000 rad  →  servos=[ 90.0,  90.0,  90.0]  status=OK
[frame    2] sun=( 323.1,  241.2) conf=1.00  →  tilt=-0.003 rad  pan=+0.005 rad  →  servos=[ 89.9,  89.8,  90.1]  status=OK
...

=== Demo complete ===
Total frames processed: 150
```

## Requirements

- Same build toolchain as the main project (CMake ≥ 3.16, GCC/Clang with C++17)
- No camera, no I²C, no servos needed

## Build

From the repository root:

```bash
cmake -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build --target pipeline_demo
```

Or add `add_subdirectory(demo)` to the root `CMakeLists.txt`
and rebuild normally.

## Run

```bash
./build/demo/pipeline_demo
```

The demo runs for 5 seconds and exits automatically.

## What to look for

| What you see | What it means |
|---|---|
| `conf=0.00` for the first few frames | The spot starts at centre — zero error, zero confidence needed |
| `tilt`/`pan` grow as the spot moves off-centre | The controller is reacting to the image error |
| `servos` values changing | Kinematics is solving the inverse-kinematics in real time |
| `status=DEGRADED` (if it appears) | The IK solver hit a geometry limit and fell back to the last valid command |
