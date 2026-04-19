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

# ---------------- OBJECT ----------------
table = p.loadURDF("table/table.urdf", [0.7, 0, 0], globalScaling=0.5)
mug = p.loadURDF("cube_small.urdf", [0.7, 0, 0.35])

# Enable gravity BEFORE actions so the block rests on the ground
p.setGravity(0, 0, -9.8)

# Let the block settle on the ground plane
for _ in range(240):
    p.stepSimulation()
    time.sleep(1./240.)

# Read the actual resting position of the block
actual_pos, _ = p.getBasePositionAndOrientation(mug)
print(f"Block settled at: {[f'{x:.4f}' for x in actual_pos]}")

#TEST ACTIONS
actions.init(robot, mug)

actions.move_to("mug.body")
actions.grasp("mug.body")
actions.lift(0.4)
actions.flip()

p.resetDebugVisualizerCamera(
    cameraDistance=1.5,
    cameraYaw=50,
    cameraPitch=-35,
    cameraTargetPosition=[0.5, 0, 0]
)

# gravity already set above
p.setRealTimeSimulation(1)

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

# (init already called above)

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
            line.startswith("place(") or
            line.startswith("flip(")
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
                "place": actions.place,
                "flip": actions.flip
            })
        except Exception as e:
            print("Execution error:", e)

# ---------------- MAIN LOOP ----------------
while True:
    task = input("\nEnter command (or 'q' to quit): ")

    if task.lower() == "q":
        break

    print("\nThinking...")

    try:
        plan = generate_plan(task)
        execute_plan(plan)
    except Exception as e:
        print("Error:", e)

    print("\n--- Done ---\n")

# ---------------- CLEAN EXIT ----------------
p.disconnect()
print("Simulation ended.")