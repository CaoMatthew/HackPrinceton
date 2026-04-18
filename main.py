import pybullet as p
import pybullet_data
import time

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

# ---------------- OBJECT ----------------
mug = p.loadURDF("cube_small.urdf", [0.5, 0, 0.1])

# ---------------- FAKE WORLD MODEL ----------------
scene = {
    "mug": {
        "body": [0.5, 0, 0.1],
        "handle": [0.5, 0.05, 0.15]
    }
}

# ---------------- DEBUG DRAW ----------------
def draw_point(pos, color=[1, 0, 0]):
    offset = 0.03

    # X axis
    p.addUserDebugLine(
        [pos[0] - offset, pos[1], pos[2]],
        [pos[0] + offset, pos[1], pos[2]],
        color, 2
    )

    # Y axis
    p.addUserDebugLine(
        [pos[0], pos[1] - offset, pos[2]],
        [pos[0], pos[1] + offset, pos[2]],
        color, 2
    )

    # Z axis
    p.addUserDebugLine(
        [pos[0], pos[1], pos[2] - offset],
        [pos[0], pos[1], pos[2] + offset],
        color, 2
    )

# Draw scene points
draw_point(scene["mug"]["body"], [0, 1, 0])     # green = body
draw_point(scene["mug"]["handle"], [1, 0, 0])   # red = handle

# IK Controls
endEffectorIndex = 11  # Panda

# Move ABOVE the handle (important)
handle_pos = scene["mug"]["handle"]
targetPos = [
    handle_pos[0],
    handle_pos[1],
    handle_pos[2] + 0.1  # hover above
]

# Solve IK
jointPoses = p.calculateInverseKinematics(
    robot,
    endEffectorIndex,
    targetPos
)

# Move arm
for step in range(200):
    for i in range(len(jointPoses)):
        p.setJointMotorControl2(
            robot,
            i,
            p.POSITION_CONTROL,
            targetPosition=jointPoses[i]
        )
    p.stepSimulation()
    time.sleep(1/240)

while True:
    p.stepSimulation()
    time.sleep(1/240)