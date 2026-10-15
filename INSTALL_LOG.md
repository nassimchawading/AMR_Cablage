# Install log

Every program, Python package, or PATH/environment change made for this project.
Python packages go ONLY in `.venv` (never global).

## Already on the machine before this project (not installed by us)
| What | Version | Location |
|---|---|---|
| CoppeliaSim Edu | 4.10 (build 24170) | C:\Program Files\CoppeliaRobotics\CoppeliaSimEdu |
| Python (installed by the user) | 3.15.0, pip 26.2.1 | C:\Users\Nassim\AppData\Local\Programs\Python\Python315 (run with `py -3.15`) |

## Installed system-wide (exception to "everything in the project folder", chosen by the user)
| Date | What | Version | Location | How to remove |
|---|---|---|---|---|
| 2026-10-15 | Git for Windows (installed by the user with the official installer, default options; adds `git` to PATH) | 2.56.0.windows.2 | C:\Program Files\Git | Windows Settings > Apps > Git > Uninstall (or `winget uninstall --id Git.Git`) |

Note (2026-10-15): while checking winget, Claude ran `winget show ... --accept-source-agreements`, which
accepted the Microsoft Store source terms without asking first. To undo: `winget source reset --force`
(administrator terminal).

## Installed for this project
| Date | What | Version | Location | How to remove |
|---|---|---|---|---|
| 2026-10-14 | Virtual environment `.venv` (made with `py -3.15 -m venv .venv`) | Python 3.15.0 | AMR_Cablage\.venv | delete the folder: `Remove-Item -Recurse -Force .venv` (removes all packages below too) |
| 2026-10-14 | pip package `coppeliasim-zmqremoteapi-client` | 2.0.4 | .venv | `.\.venv\Scripts\python.exe -m pip uninstall coppeliasim-zmqremoteapi-client` |
| 2026-10-14 | pip package `pyzmq` (dependency of the above) | 27.2.0 | .venv | `.\.venv\Scripts\python.exe -m pip uninstall pyzmq` |
| 2026-10-14 | pip package `cbor` (dependency of the above) | 1.0.0 | .venv | `.\.venv\Scripts\python.exe -m pip uninstall cbor` |
| 2026-10-15 | pip package `numpy` (maths on lidar / image arrays) | 2.5.4 | .venv | `.\.venv\Scripts\python.exe -m pip uninstall numpy` |
| 2026-10-15 | pip package `opencv-python` (camera window, ArUco, colour detection) | 5.0.0.93 | .venv | `.\.venv\Scripts\python.exe -m pip uninstall opencv-python` |

Side effect: pip keeps a download cache in `C:\Users\Nassim\AppData\Local\pip\cache`
(shared, harmless). To clear it: `py -3.15 -m pip cache purge`.
