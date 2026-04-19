import pybullet as p
import pybullet_data
import time

from scene import scene
from llm import generate_plan
import actions

# ---------------- SETUP ----------------
p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())

plane = p.loadURDF("plane.urdf")
robot = p.loadURDF("franka_panda/panda.urdf", useFixedBase=True)

p.resetDebugVisualizerCamera(
    cameraDistance=1.5,
    cameraYaw=50,
    cameraPitch=-35,
    cameraTargetPosition=[0.5, 0, 0]
)

p.setGravity(0, 0, -9.8)
p.setRealTimeSimulation(1)

# ---------------- OBJECT ----------------
mug = p.loadURDF("cube_small.urdf", [0.5, 0, 0.1])

# ---------------- DEBUG DRAW ----------------
def draw_point(pos, color=[1, 0, 0]):
    offset = 0.03

    p.addUserDebugLine([pos[0]-offset, pos[1], pos[2]],
                       [pos[0]+offset, pos[1], pos[2]], color, 2)
    p.addUserDebugLine([pos[0], pos[1]-offset, pos[2]],
                       [pos[0], pos[1]+offset, pos[2]], color, 2)
    p.addUserDebugLine([pos[0], pos[1], pos[2]-offset],
                       [pos[0], pos[1], pos[2]+offset], color, 2)

draw_point(scene["mug"]["body"], [0, 1, 0])
draw_point(scene["mug"]["handle"], [1, 0, 0])

# ---------------- INIT ACTIONS ----------------
actions.init(robot)

# ---------------- EXECUTION ----------------
def execute_plan(code):
    print("\nRAW OUTPUT:\n", code)

    lines = code.split("\n")
    plan = []

    # ✅ Extract ONLY valid commands (ignore everything else)
    for line in lines:
        line = line.strip()

        # remove comments
        if "#" in line:
            line = line.split("#")[0].strip()

        if (
            line.startswith("move_to(") or
            line.startswith("grasp(") or
            line.startswith("lift(") or
            line.startswith("place(")
        ):
            plan.append(line)

    # 🔥 KEEP ONLY FIRST VALID PLAN (max 3 steps)
    plan = plan[:3]

    print("\nFINAL PLAN:")
    for p in plan:
        print(p)

    # ✅ Execute ONLY ONCE
    for line in plan:
        try:
            print("Executing:", line)
            eval(line, {}, {
                "move_to": actions.move_to,
                "grasp": actions.grasp,
                "lift": actions.lift,
                "place": actions.place
            })
        except Exception as e:
            print("Execution error:", e)

# ---------------- MAIN LOOP ----------------
while True:
    task = input("\nEnter command (or 'q' to quit): ")

    if task.lower() == "q":
        break

    print("\n⏳ Thinking...")

    try:
        plan = generate_plan(task)
        execute_plan(plan)
    except Exception as e:
        print("Error:", e)

    print("\n--- Done ---\n")

# ---------------- CLEAN EXIT ----------------
p.disconnect()
print("Simulation ended.")