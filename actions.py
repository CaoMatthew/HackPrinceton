import pybullet as p
import time
from scene import scene

# Global robot reference (set from main)
robot = None
endEffectorIndex = 11

def init(r):
    global robot
    robot = r

# -------------------------
# RESOLVE TARGET
# -------------------------
def resolve(target):
    obj, part = target.split(".")
    return scene[obj][part]

# -------------------------
# CORE: SMOOTH IK MOTION
# -------------------------
def move_ee_smooth(target_pos, target_orn, steps=120, sleep=1./240.):
    """
    Smoothly move end effector using interpolation + IK
    """
    state = p.getLinkState(robot, endEffectorIndex)
    current_pos = state[0]

    for i in range(steps):
        alpha = i / steps

        interp_pos = [
            current_pos[j] + alpha * (target_pos[j] - current_pos[j])
            for j in range(3)
        ]

        jointPoses = p.calculateInverseKinematics(
            robot,
            endEffectorIndex,
            interp_pos,
            targetOrientation=target_orn
        )

        for j in range(7):  # panda arm joints
            p.setJointMotorControl2(
                robot,
                j,
                p.POSITION_CONTROL,
                targetPosition=jointPoses[j],
                force=300
            )

        p.stepSimulation()
        time.sleep(sleep)

# -------------------------
# MOVE TO TARGET
# -------------------------
def move_to(target):
    pos = resolve(target)

    # hover above target (important!)
    target_pos = [pos[0], pos[1], pos[2] + 0.1]

    # gripper facing downward
    target_orn = p.getQuaternionFromEuler([3.14, 0, 0])

    print("Moving to", target)

    move_ee_smooth(target_pos, target_orn, steps=120)

# -------------------------
# GRASP (simple visual fake)
# -------------------------
def grasp(target):
    print("Grasping", target)

    # close gripper (Franka fingers)
    for _ in range(50):
        p.setJointMotorControl2(robot, 9, p.POSITION_CONTROL, 0.0, force=100)
        p.setJointMotorControl2(robot, 10, p.POSITION_CONTROL, 0.0, force=100)
        p.stepSimulation()
        time.sleep(1./240.)

# -------------------------
# LIFT (REAL MOTION)
# -------------------------
def lift(height=0.15):
    print("Lifting...")

    state = p.getLinkState(robot, endEffectorIndex)
    current_pos = list(state[0])
    current_orn = state[1]

    target_pos = [
        current_pos[0],
        current_pos[1],
        current_pos[2] + height
    ]

    move_ee_smooth(target_pos, current_orn, steps=100)

# -------------------------
# PLACE (basic for now)
# -------------------------
def place(target):
    print("Placing at", target)

    pos = resolve(target)
    target_pos = [pos[0], pos[1], pos[2] + 0.1]
    target_orn = p.getQuaternionFromEuler([3.14, 0, 0])

    move_ee_smooth(target_pos, target_orn, steps=120)

    # open gripper
    for _ in range(50):
        p.setJointMotorControl2(robot, 9, p.POSITION_CONTROL, 0.04, force=100)
        p.setJointMotorControl2(robot, 10, p.POSITION_CONTROL, 0.04, force=100)
        p.stepSimulation()
        time.sleep(1./240.)