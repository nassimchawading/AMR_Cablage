r"""Render pictures of the current CoppeliaSim scene into media\ (top plan + 3D views).

Uses a temporary vision sensor (a camera) that is removed afterwards.
Run (PowerShell, from the project folder):
  .\.venv\Scripts\python.exe src\snapshot.py
"""

import struct
import zlib
from pathlib import Path

from coppeliasim_zmqremoteapi_client import RemoteAPIClient

from build_scene import look_at_matrix

MEDIA = Path(__file__).resolve().parent.parent / "media"

# name: (eye, target, resolution, perspective view angle in degrees, or None = orthographic top view)
VIEWS = {
    "atelier_plan_dessus": ((10.0, 6.0, 30.0), (10.0, 6.0001, 0.0), (1400, 860), None),
    "atelier_vue_3d":      ((10.0, -7.0, 12.0), (10.0, 5.0, 0.0), (1280, 720), 60),
    "atelier_vue_haut":    ((10.0, 18.0, 11.0), (10.0, 6.0, 0.0), (1280, 720), 60),
    "zoom_controle_qualite": ((9.6, 6.6, 2.6), (9.9, 2.6, 0.5), (1280, 720), 60),
    "zoom_ur5":              ((12.2, 4.2, 1.5), (10.95, 3.0, 0.9), (960, 720), 55),
}


def save_png(path, rgb, w, h):
    """Minimal PNG writer (no extra library). CoppeliaSim images start at the bottom row."""
    rows = [b"\x00" + rgb[(h - 1 - y) * w * 3:(h - y) * w * 3] for y in range(h)]

    def chunk(tag, data):
        return struct.pack(">I", len(data)) + tag + data + struct.pack(">I", zlib.crc32(tag + data) & 0xFFFFFFFF)

    path.write_bytes(b"\x89PNG\r\n\x1a\n"
                     + chunk(b"IHDR", struct.pack(">IIBBBBB", w, h, 8, 2, 0, 0, 0))
                     + chunk(b"IDAT", zlib.compress(b"".join(rows), 6))
                     + chunk(b"IEND", b""))


def render(sim, eye, target, res, angle):
    options = 1 | 4 | (2 if angle else 0)     # explicit handling, invisible body, perspective?
    size = angle * 3.14159 / 180 if angle else 20.6   # view angle (rad) or ortho width (m)
    s = sim.createVisionSensor(options, [res[0], res[1], 0, 0],
                               [0.1, 100.0, size, 0.1, 0.0, 0.0, 0.85, 0.85, 0.85, 0.0, 0.0])
    try:
        sim.setObjectMatrix(s, look_at_matrix(eye, target), sim.handle_world)
        sim.handleVisionSensor(s)
        img, r = sim.getVisionSensorImg(s)
        return img, r
    finally:
        sim.removeObjects([s])


def main():
    sim = RemoteAPIClient().require("sim")
    MEDIA.mkdir(exist_ok=True)
    for name, (eye, target, res, angle) in VIEWS.items():
        img, r = render(sim, eye, target, res, angle)
        out = MEDIA / f"{name}.png"
        save_png(out, img, r[0], r[1])
        print("Saved", out)


if __name__ == "__main__":
    main()
