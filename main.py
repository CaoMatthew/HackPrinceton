import pybullet as p
import pybullet_data
import time

from llm import generate_plan
import actions

# ---------------- SETUP ----------------
p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())

p.loadURDF("plane.urdf")
robot = p.loadURDF("franka_panda/panda.urdf", useFixedBase=True)

# Table and block positions match the tested push configuration
p.loadURDF("table/table.urdf",   [0.75, 0, 0],    globalScaling=0.5)
block = p.loadURDF("cube_small.urdf", [0.75, 0, 0.35])

p.setGravity(0, 0, -9.8)
for _ in range(240):
    p.stepSimulation()
    time.sleep(1. / 240.)

actual_pos, _ = p.getBasePositionAndOrientation(block)
print(f"Block settled at: {[f'{x:.4f}' for x in actual_pos]}")

p.resetDebugVisualizerCamera(
    cameraDistance=1.5, cameraYaw=50, cameraPitch=-35,
    cameraTargetPosition=[0.75, 0, 0.3]
)
p.setRealTimeSimulation(1)

actions.init(robot, block)

VALID_CALLS = ("grasp(", "lift(", "carry(", "drop(", "place(", "flip(", "push(")

# ---------------- EXECUTION ----------------
def execute_plan(code: str):
    print("\n--- K2 RAW OUTPUT ---")
    print(code)

    plan = []
    for line in code.split("\n"):
        line = line.split("#")[0].strip()   # strip inline comments
        if any(line.startswith(c) for c in VALID_CALLS):
            plan.append(line)

    if not plan:
        print("No executable actions found in plan.")
        return

    # Safeguard: bare place() has no args — treat as drop().
    plan = ["drop()" if line == "place()" else line for line in plan]

    # Enforce drop() always executes last — K2 sometimes puts it before carry().
    # Releasing mid-air drops the block; it must come after all movement.
    if "drop()" in plan and plan[-1] != "drop()":
        plan = [l for l in plan if l != "drop()"] + ["drop()"]

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

# ---------------- MAIN LOOP ----------------
print("\n" + "="*50)
print("  Robot ready. Type a command and press Enter.")
print("  Examples: 'push the block to the left'")
print("            'push it forward then right'")
print("            'flip the block'")
print("  Type 'q' to quit.")
print("="*50)

while True:
    try:
        task = input("\n> ")
    except EOFError:
        break

    if task.strip().lower() in ("q", "quit", "exit"):
        break
    if not task.strip():
        continue

    print("\nThinking...")
    try:
        plan = generate_plan(task)
        execute_plan(plan)
    except Exception as e:
        print(f"Error: {e}")

    print("\n--- Done. Ready for next command. ---")

# ---------------- CLEAN EXIT ----------------
p.disconnect()
print("Simulation ended.")