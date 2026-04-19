"""
Persistent PyBullet simulation for the web frontend.

Reads natural-language commands from stdin, runs each through the
Gemini -> K2 pipeline, executes the compiled actions, then prints
COMMAND_DONE so the server knows when the robot is idle again.

The GUI stays open until __QUIT__ is written to stdin or the process
is killed via the web UI.

Usage (internal — spawned by web/serve.py):
    python sim_persistent.py
"""
import sys
import queue
import threading
import time

import pybullet as p
import pybullet_data

from llm import generate_plan
import actions

# ── Setup (mirrors main.py) ─────────────────────────────────────────────────
p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())

p.loadURDF("plane.urdf")
robot = p.loadURDF("franka_panda/panda.urdf", useFixedBase=True)
p.loadURDF("table/table.urdf", [0.75, 0, 0], globalScaling=0.5)
block = p.loadURDF("cube_small.urdf", [0.75, 0, 0.35])

p.setGravity(0, 0, -9.8)
for _ in range(240):
    p.stepSimulation()
    time.sleep(1. / 240.)

actual_pos, _ = p.getBasePositionAndOrientation(block)
print(f"Block settled at: {[f'{x:.4f}' for x in actual_pos]}", flush=True)

p.resetDebugVisualizerCamera(
    cameraDistance=1.5, cameraYaw=50, cameraPitch=-35,
    cameraTargetPosition=[0.75, 0, 0.3],
)
p.setRealTimeSimulation(1)
actions.init(robot, block)

VALID_CALLS = ("grasp(", "lift(", "carry(", "drop(", "place(", "flip(", "push(")


def execute_plan(code: str) -> None:
    plan = []
    for line in code.split("\n"):
        line = line.split("#")[0].strip()
        if any(line.startswith(c) for c in VALID_CALLS):
            plan.append(line)

    if not plan:
        print("No executable actions found.", flush=True)
        return

    # Safeguard: bare place() → drop()
    plan = ["drop()" if l == "place()" else l for l in plan]
    # Ensure drop() is last
    if "drop()" in plan and plan[-1] != "drop()":
        plan = [l for l in plan if l != "drop()"] + ["drop()"]

    ns = {
        "grasp": actions.grasp,
        "lift":  actions.lift,
        "carry": actions.carry,
        "drop":  actions.drop,
        "place": actions.place,
        "flip":  actions.flip,
        "push":  actions.push,
    }
    for line in plan:
        print(f"  >> {line}", flush=True)
        try:
            eval(line, {}, ns)
        except Exception as e:
            print(f"     ERROR: {e}", flush=True)


# ── Stdin reader thread ──────────────────────────────────────────────────────
# Runs on a background thread so the main thread is free for PyBullet actions.
cmd_queue: queue.Queue = queue.Queue()


def _stdin_reader() -> None:
    for raw in sys.stdin:
        cmd = raw.strip()
        if cmd:
            cmd_queue.put(cmd)
    cmd_queue.put(None)  # EOF sentinel


threading.Thread(target=_stdin_reader, daemon=True).start()

# Signal to the server that startup is complete
print("READY", flush=True)

# ── Main command loop ────────────────────────────────────────────────────────
while True:
    try:
        cmd = cmd_queue.get(timeout=0.1)
    except queue.Empty:
        continue

    if cmd is None or cmd == "__QUIT__":
        break

    print(f"\n[sim] Received: {cmd}", flush=True)
    try:
        plan_code = generate_plan(cmd)
        execute_plan(plan_code)
    except Exception as e:
        print(f"[sim] Pipeline error: {e}", flush=True)

    # Sentinel — server polls for this to know the robot is idle
    print("COMMAND_DONE", flush=True)

p.disconnect()
print("GUI closed.", flush=True)
