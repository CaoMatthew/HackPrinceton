"""
One-shot simulation runner for use by the web frontend (subprocess).
Not imported by main.py — duplicates the execution path from main.py so the
interactive main.py file does not need to change.
"""
import sys
import pybullet as p
import pybullet_data
import time

from llm import generate_plan
import actions

# ---------------- SETUP (mirrors main.py) ----------------
p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())

p.loadURDF("plane.urdf")
robot = p.loadURDF("franka_panda/panda.urdf", useFixedBase=True)

p.loadURDF("table/table.urdf", [0.75, 0, 0], globalScaling=0.5)
block = p.loadURDF("cube_small.urdf", [0.75, 0, 0.35])

p.setGravity(0, 0, -9.8)
for _ in range(240):
    p.stepSimulation()
    time.sleep(1.0 / 240.0)

actual_pos, _ = p.getBasePositionAndOrientation(block)
print(f"Block settled at: {[f'{x:.4f}' for x in actual_pos]}")

p.resetDebugVisualizerCamera(
    cameraDistance=1.5,
    cameraYaw=50,
    cameraPitch=-35,
    cameraTargetPosition=[0.75, 0, 0.3],
)
p.setRealTimeSimulation(1)

actions.init(robot, block)

VALID_CALLS = ("grasp(", "lift(", "carry(", "drop(", "place(", "flip(", "push(")


def execute_plan(code: str):
    print("\n--- K2 RAW OUTPUT ---")
    print(code)

    plan = []
    for line in code.split("\n"):
        line = line.split("#")[0].strip()
        if any(line.startswith(c) for c in VALID_CALLS):
            plan.append(line)

    if not plan:
        print("No executable actions found in plan.")
        return

    print("\n--- EXECUTING ---")
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
        print(f"  >> {line}")
        try:
            eval(line, {}, ns)
        except Exception as e:
            print(f"     ERROR: {e}")


def main():
    task = " ".join(sys.argv[1:]).strip()
    if not task:
        print("Usage: python sim_once.py <natural language task>")
        sys.exit(1)

    print("\nThinking...")
    try:
        plan = generate_plan(task)
        execute_plan(plan)
    except Exception as e:
        print(f"Error: {e}")
        sys.exit(2)

    print("\n--- Done. ---")
    # Keep GUI open for 5 s so the result is visible before the window closes
    time.sleep(5)
    p.disconnect()
    print("Simulation ended.")


if __name__ == "__main__":
    main()
