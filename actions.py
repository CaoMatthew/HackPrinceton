import pybullet as p
import time
from scene import scene

# Global robot reference (set from main)
robot = None
mug_id = None
endEffectorIndex = 11
world_cid = None
grasp_cid = None

def init(r, m=None):
    global robot, mug_id, world_cid
    robot = r
    mug_id = m
    # No world constraint - gravity holds the block on the ground

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
            targetOrientation=target_orn,
            maxNumIterations=500,
            residualThreshold=1e-5
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
    # Read actual block position from physics
    if mug_id is not None:
        actual_pos = list(p.getBasePositionAndOrientation(mug_id)[0])
    else:
        actual_pos = list(resolve(target))

    # 1. First go high above the cube
    target_pos = [actual_pos[0], actual_pos[1], actual_pos[2] + 0.3]

    # point the claw completely perpendicular to the ground
    target_orn = p.getQuaternionFromEuler([3.14, 0, 0])

    print("Moving to hover high above", target)
    move_ee_smooth(target_pos, target_orn, steps=600)

    # 2. Then open the claw completely
    print("Opening claw...")
    for _ in range(120):
        p.setJointMotorControl2(robot, 9, p.POSITION_CONTROL, targetPosition=0.04, force=18.0)
        p.setJointMotorControl2(robot, 10, p.POSITION_CONTROL, targetPosition=0.04, force=18.0)
        p.stepSimulation()
        time.sleep(1./240.)

# -------------------------
# GRASP
# -------------------------
def grasp(target):
    print("Grasping", target)
    global world_cid, grasp_cid

    # Read the ACTUAL block position from physics
    if mug_id is not None:
        actual_pos = list(p.getBasePositionAndOrientation(mug_id)[0])
        print(f"  Actual block at: {[f'{x:.4f}' for x in actual_pos]}")
    else:
        actual_pos = list(resolve(target))

    # Finger pads are 0.027m above link 11.
    # Target link11 at block center so finger pads are at block_z + 0.027
    # (near block top). Fingers are wider than the block so they
    # close inward and grip the upper portion without the arm
    # crashing into the block during descent.
    target_z = actual_pos[2]
    target_pos = [actual_pos[0], actual_pos[1], target_z]
    target_orn = p.getQuaternionFromEuler([3.14, 0, 0])
    print(f"  Descending to z={target_z:.4f}")
    move_ee_smooth(target_pos, target_orn, steps=600)

    # Short settle
    for _ in range(120):
        p.stepSimulation()
        time.sleep(1./240.)

    # Log where we actually ended up
    ee_state = p.getLinkState(robot, endEffectorIndex)
    f9_state = p.getLinkState(robot, 9)
    f10_state = p.getLinkState(robot, 10)
    print(f"  EE at: {[f'{x:.4f}' for x in ee_state[0]]}")
    print(f"  Finger9 at: {[f'{x:.4f}' for x in f9_state[0]]}")
    print(f"  Finger10 at: {[f'{x:.4f}' for x in f10_state[0]]}")
    if mug_id:
        bpos = p.getBasePositionAndOrientation(mug_id)[0]
        print(f"  Block at: {[f'{x:.4f}' for x in bpos]}")

    # close gripper
    settled_count = 0
    with open("grasp_debug.log", "w") as f:
        f.write("step,pos9,vel9,pos10,vel10,block_x,block_y,block_z,finger9_x,finger9_y,finger9_z,finger10_x,finger10_y,finger10_z\n")
        for step in range(400):
            p.setJointMotorControl2(robot, 9, p.POSITION_CONTROL, targetPosition=0.0, force=5.0)
            p.setJointMotorControl2(robot, 10, p.POSITION_CONTROL, targetPosition=0.0, force=5.0)
            p.stepSimulation()
            time.sleep(1./240.)

            state9 = p.getJointState(robot, 9)
            state10 = p.getJointState(robot, 10)

            block_pos = p.getBasePositionAndOrientation(mug_id)[0] if mug_id else (0,0,0)
            f9_pos = p.getLinkState(robot, 9)[0]
            f10_pos = p.getLinkState(robot, 10)[0]
            f.write(f"{step},{state9[0]:.6f},{state9[1]:.6f},{state10[0]:.6f},{state10[1]:.6f},"
                    f"{block_pos[0]:.6f},{block_pos[1]:.6f},{block_pos[2]:.6f},"
                    f"{f9_pos[0]:.6f},{f9_pos[1]:.6f},{f9_pos[2]:.6f},"
                    f"{f10_pos[0]:.6f},{f10_pos[1]:.6f},{f10_pos[2]:.6f}\n")

            if abs(state9[1]) < 0.01 and abs(state10[1]) < 0.01 and step > 30:
                settled_count += 1
            else:
                settled_count = 0

            if settled_count > 10:
                print(f"Gripper contact at step {step}, pos9={state9[0]:.4f}, pos10={state10[0]:.4f}")
                break

    if mug_id is not None:
        if world_cid is not None:
            p.removeConstraint(world_cid)
            world_cid = None
        if grasp_cid is None:
            ee_pos, ee_orn = p.getLinkState(robot, endEffectorIndex)[0:2]
            m_pos, m_orn = p.getBasePositionAndOrientation(mug_id)
            inv_ee_pos, inv_ee_orn = p.invertTransform(ee_pos, ee_orn)
            local_pos, local_orn = p.multiplyTransforms(inv_ee_pos, inv_ee_orn, m_pos, m_orn)
            grasp_cid = p.createConstraint(robot, endEffectorIndex, mug_id, -1, p.JOINT_FIXED, [0, 0, 0], local_pos, [0, 0, 0], childFrameOrientation=local_orn)

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

    # move slower (quarter speed)
    move_ee_smooth(target_pos, current_orn, steps=400)

# -------------------------
# PLACE (basic for now)
# -------------------------
def place(target):
    print("Placing at", target)
    global world_cid, grasp_cid

    pos = resolve(target)
    target_pos = [pos[0], pos[1], pos[2] + 0.1]
    target_orn = p.getQuaternionFromEuler([3.14, 0, 0])

    # hover (quarter speed)
    move_ee_smooth(target_pos, target_orn, steps=480)

    # move down to place (quarter speed)
    target_pos = [pos[0], pos[1], pos[2]]
    move_ee_smooth(target_pos, target_orn, steps=400)

    # open gripper
    for _ in range(80):
        p.setJointMotorControl2(robot, 9, p.POSITION_CONTROL, targetPosition=0.04, force=18.0)
        p.setJointMotorControl2(robot, 10, p.POSITION_CONTROL, targetPosition=0.04, force=18.0)
        p.stepSimulation()
        time.sleep(1./240.)

    if grasp_cid is not None:
        p.removeConstraint(grasp_cid)
        grasp_cid = None

    if mug_id is not None and world_cid is None:
        p_pos, p_orn = p.getBasePositionAndOrientation(mug_id)
        world_cid = p.createConstraint(mug_id, -1, -1, -1, p.JOINT_FIXED, [0,0,0], [0,0,0], p_pos, childFrameOrientation=p_orn)

# -------------------------
# FLIP
# -------------------------
def flip():
    print("Flipping mug...")
    state = p.getLinkState(robot, endEffectorIndex)
    current_pos = list(state[0])
    current_orn = state[1]

    euler = list(p.getEulerFromQuaternion(current_orn))
    # Flip upside down by rotating roll (X-axis) by pi
    euler[0] += 3.14
    target_orn = p.getQuaternionFromEuler(euler)

    # flip (quarter speed)
    move_ee_smooth(current_pos, target_orn, steps=600)