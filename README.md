# AMR_Cablage: AMR in a cable-harness workshop

University project, module "Atelier intelligent". Solo work.

An Autonomous Mobile Robot (**KUKA youBot**: mecanum wheels + arm) transports material (cable
reels, harness boxes) between the stations of a simulated cable-harness factory in CoppeliaSim.
It plans its own paths with A*, avoids walking workers with its sensors, and streams its camera
feed live.

## How it works
- **CoppeliaSim Edu 4.10** = the world: workshop scene, physics, robot, sensors, walking workers.
- **Python 3.15** (external, ZMQ Remote API) = the brain: planning, control, camera processing.
- Simulation runs in stepping (synchronous) mode so Python and the simulation stay in sync.
- Localization: simulated indoor positioning (UWB beacons + gyro, with noise), no wheel odometry.

| Sensor | Role |
|---|---|
| Hokuyo 2D lidar (front, 240°, 5 m) | detects unexpected obstacles (workers), local avoidance |
| 8 ultrasonic sensors (ring, from the P3-DX model) | 360° short-range detection, emergency stop |
| Camera (640x480, front) | live video feed + ArUco markers to identify stations and dock |

Navigation: fixed map of the workshop (occupancy grid generated from `src\layout.py`) + A*
planner + route following. Unexpected obstacle: slow down / stop and wait / move out of the way
(mecanum) / re-plan.

## Workshop (usine de câblage)
Magasin (bobines) -> Coupe / dénudage -> Sertissage -> Assemblage (planches de câblage)
-> Test électrique (quality-control cell: conveyor + camera + UR5 arm sorting OK / REBUTS)
-> Expédition, plus a charging dock for the AMR. Three workers walk around the main aisle.

## Folders
| Folder | Content |
|---|---|
| `scenes\` | CoppeliaSim scenes (.ttt) |
| `src\` | Python code |
| `tests\` | test scripts for each part |
| `notes\` | explanations for the report (French) |
| `media\` | screenshots, videos |
| `tools\` | helper tools (if needed) |

Every installation is logged in [INSTALL_LOG.md](INSTALL_LOG.md).

## Roadmap
- [x] Part 0: project folder, README, install log, design decisions
- [x] Part 1: Python setup + connection test to CoppeliaSim
- [x] Part 2: KUKA youBot driven from Python (mecanum: forward / sideways / turn), pose from
      simulated UWB + gyro; 1 m square test (`tests\test_square.py`, open `scenes\test_youbot.ttt`)
- [x] Part 3: sensors on the youBot (Hokuyo lidar, 8 ultrasonic sensors, camera), read from Python;
      live view: `tests\test_sensors.py --live`
- [x] Part 4: workshop scene (CoppeliaSim library models + own shapes, real ArUco markers,
      quality-control cell: conveyor + camera + UR5 arm)
- [x] Part 5: occupancy grid (from `layout.py`) + A* planner; `tests\test_planner.py` (map picture
      `media\carte_chemins.png`); the youBot now waits at its dock in the workshop
- [~] Part 6 (in progress): 3 walking workers + route following and avoidance
      (`src\navigator.py`, test `tests\test_navigation.py`); last test froze CoppeliaSim, fix applied,
      to be re-tested
- [ ] Part 7: docking with ArUco + youBot arm pick / place
- [ ] Part 8: mission manager (transport orders, state machine, charging)
- [ ] Part 9: quality-control cell logic (camera decides OK / defect, UR5 sorts)
- [ ] Part 10: interface (live video, map, order tracking, send the robot on errands)
- [ ] Part 11: evaluation + report material, demo video

Report notes (French) are written as we go in [notes\](notes/README.md).
Version history: git (one snapshot per finished part).

## How to run
**Open the workshop:** CoppeliaSim → File → Open scene → `scenes\atelier_cablage.ttt`

**Rebuild the workshop after editing `src\layout.py`:** open `scenes\atelier_cablage.ttt`
(or an empty new scene), then
```
.\.venv\Scripts\python.exe src\build_scene.py
.\.venv\Scripts\python.exe src\snapshot.py      # pictures in media\
.\.venv\Scripts\python.exe src\inspect_models.py # measure library models (media\library\)
```

**Tests** (in PowerShell, after `cd C:\Users\Nassim\Documents\AMR_Cablage`; Ctrl+C stops a program):
```
.\.venv\Scripts\python.exe tests\test_planner.py        # map + A* (no CoppeliaSim needed)
.\.venv\Scripts\python.exe tests\test_navigation.py     # workshop scene open: robot drives among workers
.\.venv\Scripts\python.exe tests\test_sensors.py --live # test_youbot scene open: live camera + lidar view
```
