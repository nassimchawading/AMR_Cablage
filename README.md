# AMR_Cablage: AMR in a cable-harness workshop

University project, module "Atelier intelligent". Solo work.

An Autonomous Mobile Robot (Pioneer P3-DX) transports material (cable reels, harnesses,
connectors) between the stations of a simulated cable-harness factory in CoppeliaSim.
It plans its own paths, avoids obstacles (including walking workers) with its sensors,
and streams its camera feed live.

## How it works
- **CoppeliaSim Edu 4.10** = the world: workshop scene, physics, robot, sensors.
- **Python** (external, ZMQ Remote API) = the brain: planning, control, camera processing.
- Simulation runs in stepping (synchronous) mode so Python and the simulation stay in sync.

| Sensor | Role |
|---|---|
| 2D lidar | adds unexpected obstacles to the map, local avoidance |
| 16 ultrasonic sensors (P3-DX) | short-range safety stop |
| Camera (vision sensor) | live video feed + ArUco markers to identify stations and dock |

Navigation: fixed map of the workshop (occupancy grid) + A* global planner + path following.
If the path is blocked: slow down / stop and wait, then re-plan.

## Workshop (usine de câblage)
Magasin (bobines) -> Coupe / dénudage -> Sertissage -> Assemblage (planches de câblage)
-> Test électrique -> Expédition, plus a charging dock for the AMR.

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
- [ ] Part 6: path following + obstacle avoidance / re-planning
- [ ] Part 7: mission manager (transport orders, state machine, charging)
- [ ] Part 8: interface (live video, map, order tracking, send the robot on errands)
- [ ] Part 9: evaluation + report material

## How to run
**Open the workshop:** CoppeliaSim → File → Open scene → `scenes\atelier_cablage.ttt`

**Rebuild the workshop after editing `src\layout.py`:** open `scenes\atelier_cablage.ttt`
(or an empty new scene), then
```
.\.venv\Scripts\python.exe src\build_scene.py
.\.venv\Scripts\python.exe src\snapshot.py      # pictures in media\
.\.venv\Scripts\python.exe src\inspect_models.py # measure library models (media\library\)
```

**Run the robot (later parts):**
1. Open CoppeliaSim, load `scenes\<scene>.ttt`
2. In PowerShell:
   ```
   cd C:\Users\Nassim\Documents\AMR_Cablage
   .\.venv\Scripts\python.exe src\<main>.py
   ```
