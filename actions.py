import pybullet as p
from scene import scene

# Global robot reference (set from main)
robot = None
endEffectorIndex = 11

def init(r):
    global robot
    robot = r

def resolve(target):
    obj, part = target.split(".")
    return scene[obj][part]

def move_to(target):
    pos = resolve(target)

    # hover slightly above target
    targetPos = [pos[0], pos[1], pos[2] + 0.1]

    jointPoses = p.calculateInverseKinematics(
        robot,
        endEffectorIndex,
        targetPos
    )

    # FAST movement (reduced steps, no sleep)
    for step in range(30):
        for i in range(len(jointPoses)):
            p.setJointMotorControl2(
                robot,
                i,
                p.POSITION_CONTROL,
                targetPosition=jointPoses[i]
            )
        p.stepSimulation()

def grasp(target):
    print(f"Grasping {target} (mock for now)")

def lift(height):
    print(f"Lifting {height} (mock for now)")

def place(target):
    print(f"Placing at {target} (mock for now)")