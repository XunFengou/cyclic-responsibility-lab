"""Render MP4 from logged autonomous MuJoCo states, never from a new rollout.

Usage (Python needs mujoco 3.8.1, numpy, Pillow, imageio-ffmpeg or system ffmpeg):
  python scripts/render-autonomous-video.py research/autonomous/video-normal.json \
      --output assets/result/autonomous-normal.mp4 --title "Autonomous coverage and transport"

Input: {sceneXml, config, events?, frames:[{time,qpos,agents?,robots?,object?,workflow?,row?}]}.
For model changes supply scenes:{key:xml}, frame.scene:key, or frame.sceneXml.
The renderer holds the last logged state between samples. It never interpolates
robot/object poses, integrates physics, retimes a failure, or snaps an endpoint.
"""
from __future__ import annotations

import argparse
import bisect
import functools
import hashlib
import json
import math
import os
from pathlib import Path
import re
import shutil
import subprocess
import sys

PROJECT = Path(__file__).resolve().parents[2]
for extra in [PROJECT / "work/tiago-python", PROJECT / "work/video_assets/python"]:
    if extra.is_dir():
        sys.path.insert(0, str(extra))

import mujoco
import numpy as np
from PIL import Image, ImageDraw, ImageFont

PALETTE = ["#0072b2", "#d55e00", "#009e73", "#cc79a7", "#e69f00", "#56b4e9", "#8c564b", "#6f4e7c", "#4d4d4d"]
INK, MUTED, BACKGROUND, BORDER = "#183345", "#607484", "#eef3f7", "#cfdae3"


def rgb(color):
    return tuple(int(color[i:i + 2], 16) for i in (1, 3, 5))


@functools.lru_cache(maxsize=32)
def font(size, bold=False, mono=False):
    names = ["consola.ttf"] if mono else ["segoeuib.ttf" if bold else "segoeui.ttf", "arial.ttf"]
    for name in names:
        candidate = Path(os.environ.get("WINDIR", "C:/Windows")) / "Fonts" / name
        if candidate.exists():
            return ImageFont.truetype(str(candidate), size)
    try:
        return ImageFont.truetype("DejaVuSans.ttf", size)
    except OSError:
        return ImageFont.load_default()


def ffmpeg_path():
    executable = shutil.which("ffmpeg")
    if executable:
        return executable
    import imageio_ffmpeg
    return imageio_ffmpeg.get_ffmpeg_exe()


def number(value, precision=3):
    if value is None or not isinstance(value, (int, float)) or not math.isfinite(value):
        return "n/a"
    return f"{value:.{precision}f}"


def first(mapping, keys, default=None):
    for key in keys:
        value = mapping.get(key)
        if value is not None:
            return value
    return default


def carrier_id(frame):
    workflow, obj = frame.get("workflow", {}), frame.get("object", {})
    value = first(workflow, ["carrierId", "carrier", "ownerId", "activeCarrier"], first(obj, ["carrierId", "ownerId", "carrier"]))
    if isinstance(value, dict):
        value = first(value, ["id", "robotId"])
    return int(value) if isinstance(value, (int, float)) and value > 0 else None


def scene_xml(recording, frame):
    if frame.get("sceneXml"):
        return frame["sceneXml"]
    if frame.get("scene") is not None:
        collection = recording.get("scenes", {})
        index = frame["scene"]
        value = collection[index] if isinstance(collection, list) else collection[str(index)]
        return value["sceneXml"] if isinstance(value, dict) else value
    return recording.get("sceneXml") or recording.get("modelXml")


class NativeScene:
    def __init__(self, xml):
        if not xml:
            raise ValueError("Recording has no sceneXml for native state rendering")
        self.digest = hashlib.sha256(xml.encode()).hexdigest()
        self.model = mujoco.MjModel.from_xml_string(xml)
        self.model.vis.global_.offwidth = 752
        self.model.vis.global_.offheight = 470
        self.model.vis.headlight.ambient[:] = .35
        self.model.vis.headlight.diffuse[:] = .65
        self.model.vis.headlight.specular[:] = .05
        self.options = mujoco.MjvOption()
        self.options.geomgroup[:] = 1  # The PAL primitive collision shapes use group 3.
        self.options.sitegroup[:] = 0
        self.data = mujoco.MjData(self.model)
        self.world = mujoco.Renderer(self.model, 470, 752, max_geom=12000)
        self.closeup = mujoco.Renderer(self.model, 334, 476, max_geom=5000)
        self.base = {}
        for j in range(self.model.njnt):
            name = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_JOINT, j) or ""
            match = re.fullmatch(r"r(\d+)_base_x", name)
            if match:
                robot = int(match[1])
                jy = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, f"r{robot}_base_y")
                yaw = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, f"r{robot}_base_yaw")
                self.base[robot] = (int(self.model.jnt_qposadr[j]), int(self.model.jnt_qposadr[jy]), int(self.model.jnt_qposadr[yaw]))
        self.tray = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, "tray")
        # Visual-only colors: no recorded physical parameter or pose is changed.
        for g in range(self.model.ngeom):
            body = int(self.model.geom_bodyid[g])
            name = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_BODY, body) or ""
            match = re.match(r"r(\d+)_", name)
            if match:
                self.model.geom_rgba[g] = [c / 255 for c in rgb(PALETTE[(int(match[1]) - 1) % len(PALETTE)])] + [1.0]
            elif name == "tray":
                self.model.geom_rgba[g] = [.80, .55, .15, 1]

    def render(self, frame, histories):
        q = np.asarray(frame.get("qpos"), dtype=float)
        if q.shape != (self.model.nq,) or not np.isfinite(q).all():
            raise ValueError(f"t={frame['time']}: qpos must contain exactly {self.model.nq} finite coordinates, got {q.shape}")
        self.data.qpos[:] = q
        self.data.time = float(frame["time"])
        if frame.get("qvel") is not None:
            velocity = np.asarray(frame["qvel"], dtype=float)
            if velocity.shape != (self.model.nv,):
                raise ValueError("qvel length does not match selected model")
            self.data.qvel[:] = velocity
        mujoco.mj_forward(self.model, self.data)
        camera = mujoco.MjvCamera()
        camera.lookat[:] = [0, 0, .55]
        camera.distance, camera.azimuth, camera.elevation = 26.5, 110, -63
        self.world.update_scene(self.data, camera=camera, scene_option=self.options)
        self.world.scene.flags[mujoco.mjtRndFlag.mjRND_SHADOW] = False
        self.world.scene.flags[mujoco.mjtRndFlag.mjRND_FOG] = False
        for robot, addresses in self.base.items():
            points = histories.get(robot, [])
            if len(points) > 400:
                points = [points[i] for i in np.linspace(0, len(points) - 1, 400).astype(int)]
            for a, b in zip(points, points[1:]):
                if np.linalg.norm(np.asarray(b) - np.asarray(a)) < .006:
                    continue
                if self.world.scene.ngeom >= self.world.scene.maxgeom:
                    break
                geom = self.world.scene.geoms[self.world.scene.ngeom]
                color = np.asarray([c / 255 for c in rgb(PALETTE[(robot - 1) % len(PALETTE)])] + [.85], dtype=np.float32)
                mujoco.mjv_initGeom(geom, mujoco.mjtGeom.mjGEOM_LINE, np.zeros(3), np.zeros(3), np.eye(3).reshape(-1), color)
                mujoco.mjv_connector(geom, mujoco.mjtGeom.mjGEOM_LINE, 2.0, np.asarray(a), np.asarray(b))
                self.world.scene.ngeom += 1
        world = self.world.render().copy()
        obj = frame.get("object", {})
        position = first(obj, ["pos", "tray", "p"])
        if not isinstance(position, list) or len(position) < 3:
            position = self.data.xpos[self.tray].tolist() if self.tray >= 0 else [0, 0, .8]
        owner = carrier_id(frame)
        center = np.asarray(position[:3], dtype=float)
        camera.lookat[:] = center
        camera.lookat[2] = max(.7, center[2])
        camera.distance, camera.azimuth, camera.elevation = 3.1, 135, -20
        if owner in self.base:
            x, y, yaw = self.base[owner]
            camera.lookat[:] = [.55 * center[0] + .45 * q[x], .55 * center[1] + .45 * q[y], .9]
            camera.azimuth = math.degrees(q[yaw]) + 135
        self.closeup.update_scene(self.data, camera=camera, scene_option=self.options)
        self.closeup.scene.flags[mujoco.mjtRndFlag.mjRND_FOG] = False
        closeup = self.closeup.render().copy()
        return world, closeup

    def close(self):
        self.world.close()
        self.closeup.close()


def chart(draw, rect, points, title, unit, color, total_time, ylim=None):
    left, top, width, height = rect
    draw.rounded_rectangle((left, top, left + width, top + height), radius=8, fill="white", outline=BORDER)
    draw.text((left + 10, top + 7), f"{title} ({unit})", font=font(13, True), fill=INK)
    finite = [(t, v) for t, v in points if isinstance(v, (int, float)) and math.isfinite(v)]
    if not finite:
        return
    lower, upper = ylim or (min(0, min(v for _, v in finite)), max(v for _, v in finite) * 1.05)
    if upper <= lower:
        upper = lower + 1
    x0, y0, x1, y1 = left + 38, top + 32, left + width - 12, top + height - 23
    draw.line((x0, y0, x0, y1, x1, y1), fill=BORDER)
    for fraction in (.0, .5, 1):
        y = y1 - fraction * (y1 - y0)
        draw.line((x0, y, x1, y), fill="#e9eef2")
        draw.text((left + 3, y - 6), f"{lower + fraction * (upper - lower):.1f}", font=font(10), fill=MUTED)
    path = [(x0 + (x1 - x0) * t / max(total_time, .1), y1 - (y1 - y0) * (v - lower) / (upper - lower)) for t, v in finite]
    if len(path) > 1:
        draw.line(path, fill=color, width=2)
    draw.text((x0, y1 + 4), "0", font=font(10), fill=MUTED)
    draw.text((x1 - 38, y1 + 4), f"{total_time:g} s", font=font(10), fill=MUTED)


def compose(world, closeup, frame, recording, curves, time, fps, title):
    image = Image.new("RGB", (1280, 720), BACKGROUND)
    draw = ImageDraw.Draw(image)
    draw.rectangle((0, 0, 1280, 76), fill="#17394f")
    draw.text((18, 10), title, font=font(25, True), fill="white")
    draw.text((20, 45), f"Actual recorded MuJoCo states | zero-order hold, no pose interpolation | {fps:g} encoded fps", font=font(14), fill="#ccdee9")
    image.paste(Image.fromarray(world), (16, 94))
    image.paste(Image.fromarray(closeup), (788, 94))
    draw.rectangle((16, 94, 768, 120), fill="#17394f")
    draw.text((25, 98), "FLEET / ACTUAL COLORED BASE TRAJECTORIES", font=font(13, True), fill="white")
    draw.rectangle((788, 94, 1264, 120), fill="#17394f")
    draw.text((797, 98), "MANIPULATION / ACTUAL ROBOT AND OBJECT", font=font(13, True), fill="white")
    workflow, row, obj = frame.get("workflow", {}), frame.get("row", {}), frame.get("object", {})
    stage = str(first(workflow, ["stage", "phase"], obj.get("phase", "unknown"))).replace("_", " ")
    owner = carrier_id(frame)
    n = int(first(row, ["N"], recording.get("config", {}).get("N", len(frame.get("robots", [])) or 0)))
    k = int(first(row, ["k"], recording.get("config", {}).get("k", 2)))
    ids = [r.get("id") for r in frame.get("robots", frame.get("agents", [])) if isinstance(r, dict) and isinstance(r.get("id"), int)]
    if ids:
        draw.rectangle((16, 542, 768, 564), fill="#edf3f7")
        for index, robot in enumerate(ids[:9]):
            x = 27 + index * 79
            color = PALETTE[(robot - 1) % len(PALETTE)]
            draw.rectangle((x, 548, x + 10, 558), fill=color)
            draw.text((x + 15, 545), f"ID {robot}" + (" *" if robot == owner else ""), font=font(11, True), fill=color)
    draw.rounded_rectangle((788, 442, 1264, 704), radius=8, fill="white", outline=BORDER)
    draw.text((800, 450), f"t={time:.1f} s  |  state sample t={frame['time']:.2f} s", font=font(15, True), fill=INK)
    color = "#9f352d" if any(word in stage for word in ["fault", "failed", "quarant", "recover"]) else "#126b96"
    draw.text((800, 475), "Phase: " + stage[:46], font=font(17, True), fill=color)
    lines = [
        f"Carrier: {'unassigned' if owner is None else 'ID '+str(owner)}  |  N={n}, k={k}, gcd={math.gcd(n,k) if n else 'n/a'}",
        f"Success={bool(workflow.get('success', False))}  |  background={number(100*row.get('backgroundComplete',0),1)}%  |  handovers={workflow.get('handoverCount',0)}",
        f"H={number(row.get('H'))} m^2  |  speed RMS={number(row.get('inputRms'))} m/s",
        f"Arm torque RMS={number(row.get('armTorqueRms'))} N m  |  byte-hops={number(row.get('bytes'),0)}",
        f"Object placement error={number(first(obj,['xy','xyError'],row.get('objectXYError')))} m",
        f"Active grasps={sum(bool(a) for a in obj.get('activeGrasps', []))}/2  |  bilateral={bool(obj.get('contact',{}).get('bilateral',False))}  |  support={bool(first(obj.get('contact',{}),['delivery','support'],False))}",
    ]
    for i, line in enumerate(lines):
        draw.text((800, 505 + i * 20), line, font=font(12), fill=INK)
    events = [e for e in recording.get("events", []) if e.get("time", 1e30) <= frame["time"]]
    if events:
        last = events[-1]
        draw.text((800, 636), "Latest event:", font=font(12, True), fill=MUTED)
        event = str(first(last, ["type", "message", "event"], "event"))
        draw.text((800, 656), event[:66], font=font(12), fill=INK)
        if len(event) > 66:
            draw.text((800, 675), event[66:132], font=font(12), fill=INK)
    end = recording["frames"][-1]["time"]
    history = [f for f in recording["frames"] if f["time"] <= frame["time"]]
    chart(draw, (16, 580, 366, 124), [(f["time"], 100 * f.get("row", {}).get("backgroundComplete", 0)) for f in history], "Background inspected", "%", "#009e73", end, (0, 100))
    chart(draw, (398, 580, 370, 124), [(f["time"], f.get("row", {}).get("H")) for f in history], "Common geometric H", "m^2", "#7956a5", end, curves["Hlim"])
    return image


def validate_mp4(ffmpeg, output, count, fps):
    check = subprocess.run([ffmpeg, "-hide_banner", "-loglevel", "error", "-i", str(output), "-progress", "pipe:1", "-f", "null", "-"], capture_output=True, text=True, check=True)
    decoded = [int(v) for v in re.findall(r"^frame=(\d+)$", check.stdout, re.M)]
    times = [int(v) / 1e6 for v in re.findall(r"^out_time_us=(\d+)$", check.stdout, re.M)]
    if not decoded or decoded[-1] != count:
        raise RuntimeError(f"MP4 decoded {decoded[-1] if decoded else None} frames, expected {count}")
    if times and abs(times[-1] - count / fps) > 2 / fps:
        raise RuntimeError("Decoded MP4 duration does not match the recorded time grid")
    return {"decodedFrames": decoded[-1], "durationSeconds": times[-1] if times else count / fps, "decodeStatus": "passed"}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("input", type=Path)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--title", default="Autonomous multi-robot coverage and transport")
    parser.add_argument("--fps", type=float, default=10)
    parser.add_argument("--max-seconds", type=float, default=None, help="Render a recorded excerpt of this duration; provenance explicitly reports truncation")
    parser.add_argument("--crf", type=int, default=27)
    args = parser.parse_args()
    source = args.input.read_bytes()
    if source[:2] == b"\x1f\x8b":
        import gzip
        source = gzip.decompress(source)
    recording = json.loads(source)
    frames = recording.get("frames", [])
    if not frames or not 1 <= args.fps <= 60:
        raise ValueError("Provide a recording with frames and 1..60 fps")
    times = [float(f["time"]) for f in frames]
    if times != sorted(times) or not all(math.isfinite(t) for t in times):
        raise ValueError("Frame timestamps must be finite and nondecreasing")
    final = min(times[-1], times[0] + args.max_seconds) if args.max_seconds is not None else times[-1]
    count = max(1, int(round((final - times[0]) * args.fps)) + 1)
    h_values = [f.get("row", {}).get("H") for f in frames]
    h_values = [v for v in h_values if isinstance(v, (float, int)) and math.isfinite(v)]
    curves = {"Hlim": (0, max(h_values) * 1.05) if h_values else (0, 1)}
    ffmpeg = ffmpeg_path()
    args.output.parent.mkdir(parents=True, exist_ok=True)
    # Keep ordinary research clips below the static host's individual-file limit.
    maxrate = max(350, min(1600, int(21 * 1024 * 1024 * 8 / (1000 * count / args.fps))))
    command = [ffmpeg, "-y", "-hide_banner", "-loglevel", "warning", "-f", "rawvideo", "-pix_fmt", "rgb24", "-s", "1280x720", "-r", str(args.fps), "-i", "pipe:0", "-an", "-c:v", "libx264", "-preset", "fast", "-crf", str(args.crf), "-maxrate", str(maxrate)+"k", "-bufsize", str(2*maxrate)+"k", "-pix_fmt", "yuv420p", "-movflags", "+faststart", str(args.output)]
    process = subprocess.Popen(command, stdin=subprocess.PIPE, stdout=subprocess.DEVNULL, stderr=subprocess.PIPE)
    native = None
    histories = {}
    last_state = -1
    native_hashes = []
    previews = {0, count // 3, 2 * count // 3, count - 1}
    try:
        for index in range(count):
            time = min(final, times[0] + index / args.fps)
            at = max(0, bisect.bisect_right(times, time + 1e-9) - 1)
            frame = frames[at]
            xml = scene_xml(recording, frame)
            digest = hashlib.sha256(xml.encode()).hexdigest() if xml else None
            if native is None or native.digest != digest:
                if native:
                    native.close()
                native = NativeScene(xml)
                native_hashes.append(native.digest)
            if at != last_state:
                for logged in frames[last_state + 1:at + 1]:
                    positions = logged["qpos"]
                    if len(positions) == native.model.nq:
                        for robot, (x, y, _) in native.base.items():
                            histories.setdefault(robot, []).append([positions[x], positions[y], .025])
                last_state = at
            world, closeup = native.render(frame, histories)
            image = compose(world, closeup, frame, recording, curves, time, args.fps, args.title)
            process.stdin.write(image.tobytes())
            if index in previews:
                image.save(args.output.with_name(args.output.stem + f"-frame-{index:05d}.png"))
            if index % 100 == 0:
                print(json.dumps({"rendered": index, "total": count, "time": time}), flush=True)
        process.stdin.close()
        errors = process.stderr.read().decode(errors="replace")
        if process.wait() != 0:
            raise RuntimeError("FFmpeg encode failed: " + errors)
    except Exception:
        process.kill()
        process.wait()
        raise
    finally:
        if native:
            native.close()
    decoded = validate_mp4(ffmpeg, args.output, count, args.fps)
    report = {"status": "passed", "input": str(args.input.resolve()), "inputSha256": hashlib.sha256(source).hexdigest(), "output": str(args.output.resolve()), "outputSha256": hashlib.sha256(args.output.read_bytes()).hexdigest(), "bytes": args.output.stat().st_size, "width": 1280, "height": 720, "fps": args.fps, "frames": count, "sourceFirstTime": times[0], "sourceLastTime": times[-1], "renderedLastTime": final, "truncated": args.max_seconds is not None and final < times[-1], "mujoco": mujoco.__version__, "sceneSha256": list(dict.fromkeys(native_hashes)), "statePolicy": "Forward-kinematic rendering of actual logged qpos; zero-order hold between samples; no physics rollout or pose interpolation", "visualChanges": "Visible primitive collision-geometry groups, lighting, persistent per-robot colors and decorative actual base-history lines only", "validation": decoded, "browserVerification": False, "hardwareVerification": False}
    args.output.with_suffix(".video.json").write_text(json.dumps(report, indent=2), encoding="utf-8")
    print(json.dumps(report), flush=True)


if __name__ == "__main__":
    main()
