"""
Franka Panda Skill Primitives for PyBullet
==========================================
Proven, tested manipulation primitives for the Franka Panda robot arm.
Each skill reads actual physics state (not hardcoded positions) and uses
smooth IK interpolation with force-limited position control.

Usage by LLM planner:
    move_to(obj_id)               - Hover above an object with open gripper
    grasp(obj_id)                 - Descend and grip an object
    lift(height)                  - Raise the arm vertically
    flip()                        - Rotate the held object 180 degrees
    place(target_pos)             - Lower and release an object
    push(obj_id, direction, dist) - Slide an object on a surface

Key design rules (learned from testing):
    - Always query p.getBasePositionAndOrientation() for real object positions
    - Gravity must be enabled BEFORE any actions run
    - Objects must be on a table (z~0.34) — arm can't reach ground level
    - Link 11 (grasptarget) sits 0.027m below finger pad center
    - Grip force=5.0 prevents squeezing objects out; force=18.0 is too strong
    - Contact detection needs 30+ steps warmup to avoid false positives
"""

import pybullet as p
import time
import math

# =============================================
# GLOBAL STATE
# =============================================
robot = None
obj_id = None          # currently tracked object
endEffectorIndex = 11  # panda_grasptarget link
grasp_cid = None       # constraint binding object to EE

# Tuned constants
GRIPPER_OPEN = 0.04          # max finger opening (meters)
GRIPPER_CLOSED = 0.0         # fully closed position
GRIP_FORCE = 5.0             # gentle grip — won't squeeze out small objects
GRIPPER_FORCE_OPEN = 18.0    # force for opening fingers
ARM_FORCE = 300              # joint motor force for arm motion
IK_ITERATIONS = 500          # high iteration IK for precision
IK_THRESHOLD = 1e-5          # IK residual threshold
PERP_DOWN_ORN = [math.pi, 0, 0]  # Euler angles for perpendicular-to-ground


def init(r, obj=None):
    """Initialize the skills library with a robot and optional tracked object.

    Args:
        r: PyBullet body ID of the Franka Panda robot
        obj: PyBullet body ID of the object to manipulate (optional)
    """
    global robot, obj_id
    robot = r
    obj_id = obj


def get_object_pos(target_obj=None):
    """Get the actual position of an object from the physics engine.

    Args:
        target_obj: PyBullet body ID. If None, uses the tracked obj_id.

    Returns:
        list of [x, y, z] position
    """
    oid = target_obj if target_obj is not None else obj_id
    if oid is not None:
        return list(p.getBasePositionAndOrientation(oid)[0])
    return None


def get_ee_pos():
    """Get the current end-effector position and orientation.

    Returns:
        tuple of (position, orientation) from link state
    """
    state = p.getLinkState(robot, endEffectorIndex)
    return list(state[0]), state[1]


# =============================================
# CORE: SMOOTH IK MOTION
# =============================================
def move_ee_smooth(target_pos, target_orn, steps=120, sleep=1./240.):
    """Smoothly move end effector from current position to target using
    linear interpolation + high-precision IK at each step.

    This is the core motion primitive. All other skills use this.

    Args:
        target_pos: [x, y, z] target position for link 11 (grasptarget)
        target_orn: quaternion [x, y, z, w] target orientation
        steps: number of interpolation steps (higher = slower/smoother)
        sleep: seconds to wait between steps (1/240 for real-time)

    Design notes:
        - 500 IK iterations prevents wrist drift during linear descent
        - Interpolation ensures smooth path (no sudden joint jumps)
        - force=300 on arm joints maintains rigidity against gravity
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
            robot, endEffectorIndex, interp_pos,
            targetOrientation=target_orn,
            maxNumIterations=IK_ITERATIONS,
            residualThreshold=IK_THRESHOLD
        )

        for j in range(7):
            p.setJointMotorControl2(
                robot, j, p.POSITION_CONTROL,
                targetPosition=jointPoses[j],
                force=ARM_FORCE
            )

        p.stepSimulation()
        time.sleep(sleep)


def open_gripper(steps=120, force=None):
    """Open the gripper fingers to maximum width (0.04m each side = 8cm gap).

    Args:
        steps: simulation steps to hold the open command
        force: motor force (default: 18.0)
    """
    f = force if force is not None else GRIPPER_FORCE_OPEN
    for _ in range(steps):
        p.setJointMotorControl2(robot, 9, p.POSITION_CONTROL, targetPosition=GRIPPER_OPEN, force=f)
        p.setJointMotorControl2(robot, 10, p.POSITION_CONTROL, targetPosition=GRIPPER_OPEN, force=f)
        p.stepSimulation()
        time.sleep(1./240.)


def close_gripper(force=None, max_steps=400, min_warmup=30, settle_threshold=10):
    """Close gripper with contact detection.

    Closes fingers until both finger velocities drop below 0.01 for
    `settle_threshold` consecutive steps (indicating contact with an object
    or full closure).

    Args:
        force: grip force (default: 5.0 — gentle hold)
        max_steps: maximum steps before giving up
        min_warmup: minimum steps before checking for contact (avoids
                    false positives during initial finger acceleration)
        settle_threshold: consecutive low-velocity steps needed to confirm grip

    Returns:
        dict with 'pos9', 'pos10' (final finger positions) and 'step' (when contact detected)

    Design notes:
        - force=5.0 prevents squeezing small objects out of the gripper
        - force=18.0 caused the block to be ejected in testing
        - min_warmup=30 is critical: fingers accelerate from rest and
          briefly pass through low-velocity states before reaching the object
    """
    f = force if force is not None else GRIP_FORCE
    settled_count = 0

    for step in range(max_steps):
        p.setJointMotorControl2(robot, 9, p.POSITION_CONTROL, targetPosition=GRIPPER_CLOSED, force=f)
        p.setJointMotorControl2(robot, 10, p.POSITION_CONTROL, targetPosition=GRIPPER_CLOSED, force=f)
        p.stepSimulation()
        time.sleep(1./240.)

        state9 = p.getJointState(robot, 9)
        state10 = p.getJointState(robot, 10)

        if abs(state9[1]) < 0.01 and abs(state10[1]) < 0.01 and step > min_warmup:
            settled_count += 1
        else:
            settled_count = 0

        if settled_count > settle_threshold:
            print(f"  Gripper contact at step {step}, pos9={state9[0]:.4f}, pos10={state10[0]:.4f}")
            return {"pos9": state9[0], "pos10": state10[0], "step": step}

    # Max steps reached — return final state anyway
    state9 = p.getJointState(robot, 9)
    state10 = p.getJointState(robot, 10)
    print(f"  Gripper closed (max steps), pos9={state9[0]:.4f}, pos10={state10[0]:.4f}")
    return {"pos9": state9[0], "pos10": state10[0], "step": max_steps}


def settle(steps=120):
    """Step the simulation forward to let physics settle.

    Args:
        steps: number of simulation steps (at 240Hz, 120 steps = 0.5 seconds)
    """
    for _ in range(steps):
        p.stepSimulation()
        time.sleep(1./240.)


def bind_object_to_ee(target_obj=None):
    """Create a fixed constraint binding an object to the end-effector.

    Uses multiplyTransforms to compute the object's position in the EE's
    local frame, so the object stays exactly where it was grabbed.

    Args:
        target_obj: PyBullet body ID. If None, uses tracked obj_id.
    """
    global grasp_cid
    oid = target_obj if target_obj is not None else obj_id
    if oid is None or grasp_cid is not None:
        return

    ee_pos, ee_orn = p.getLinkState(robot, endEffectorIndex)[0:2]
    m_pos, m_orn = p.getBasePositionAndOrientation(oid)
    inv_ee_pos, inv_ee_orn = p.invertTransform(ee_pos, ee_orn)
    local_pos, local_orn = p.multiplyTransforms(inv_ee_pos, inv_ee_orn, m_pos, m_orn)
    grasp_cid = p.createConstraint(
        robot, endEffectorIndex, oid, -1, p.JOINT_FIXED,
        [0, 0, 0], local_pos, [0, 0, 0],
        childFrameOrientation=local_orn
    )


def release_object():
    """Remove the fixed constraint binding an object to the EE."""
    global grasp_cid
    if grasp_cid is not None:
        p.removeConstraint(grasp_cid)
        grasp_cid = None


# =============================================
# HIGH-LEVEL SKILLS (what the LLM calls)
# =============================================

def move_to(target_obj=None):
    """Move the arm to hover 30cm above an object with gripper open.

    Sequence:
        1. Read actual object position from physics
        2. Move to [obj_x, obj_y, obj_z + 0.3] with perpendicular orientation
        3. Open the gripper fully

    Args:
        target_obj: PyBullet body ID. If None, uses tracked obj_id.
    """
    actual_pos = get_object_pos(target_obj)
    if actual_pos is None:
        print("ERROR: No object to move to")
        return

    target_pos = [actual_pos[0], actual_pos[1], actual_pos[2] + 0.3]
    target_orn = p.getQuaternionFromEuler(PERP_DOWN_ORN)

    print(f"Moving to hover above object at {[f'{x:.3f}' for x in actual_pos]}")
    move_ee_smooth(target_pos, target_orn, steps=600)

    print("Opening gripper...")
    open_gripper()


def grasp(target_obj=None):
    """Descend to an object and grip it with contact detection.

    Sequence:
        1. Read actual object position from physics
        2. Descend with perpendicular orientation to object center height
           (finger pads land ~0.027m above link 11, near object top)
        3. Settle for 120 steps to let arm converge
        4. Close gripper with force=5.0 and contact detection
        5. Bind object to end-effector with a fixed constraint

    Args:
        target_obj: PyBullet body ID. If None, uses tracked obj_id.

    Design notes:
        - Targets link 11 at object center Z. Finger pads end up at
          obj_z + 0.027 (near object top). Fingers are wider than the
          object so they close inward and clamp the upper portion.
        - force=5.0 gives gentle hold. force=18.0 squeezes objects out.
        - The fixed constraint ensures the object follows the arm
          through lift/flip even if finger grip isn't perfectly tight.
    """
    global grasp_cid
    oid = target_obj if target_obj is not None else obj_id
    actual_pos = get_object_pos(oid)
    if actual_pos is None:
        print("ERROR: No object to grasp")
        return

    print(f"Grasping object at {[f'{x:.3f}' for x in actual_pos]}")

    # Descend: target link 11 at object center
    target_pos = [actual_pos[0], actual_pos[1], actual_pos[2]]
    target_orn = p.getQuaternionFromEuler(PERP_DOWN_ORN)
    move_ee_smooth(target_pos, target_orn, steps=600)

    # Let arm converge at target
    settle(120)

    # Close and detect contact
    grip_result = close_gripper()

    # Bind object to EE with constraint
    bind_object_to_ee(oid)


def lift(height=0.15):
    """Raise the arm vertically from its current position.

    Maintains current orientation. Used after grasping to pick up objects.

    Args:
        height: meters to lift (default 0.15m = 15cm)
    """
    print(f"Lifting {height:.2f}m...")
    current_pos, current_orn = get_ee_pos()
    target_pos = [current_pos[0], current_pos[1], current_pos[2] + height]
    move_ee_smooth(target_pos, current_orn, steps=400)


def flip():
    """Flip the held object 180 degrees by rotating around the X-axis.

    Rotates the end-effector by pi radians around the roll axis while
    maintaining position. The bound object rotates with it.
    """
    print("Flipping object...")
    current_pos, current_orn = get_ee_pos()
    euler = list(p.getEulerFromQuaternion(current_orn))
    euler[0] += math.pi
    target_orn = p.getQuaternionFromEuler(euler)
    move_ee_smooth(current_pos, target_orn, steps=600)


def place(target_pos):
    """Lower and release an object at a target position.

    Sequence:
        1. Move to hover 10cm above target
        2. Descend to target position
        3. Open gripper
        4. Release constraint

    Args:
        target_pos: [x, y, z] where to place the object
    """
    print(f"Placing at {[f'{x:.3f}' for x in target_pos]}")
    target_orn = p.getQuaternionFromEuler(PERP_DOWN_ORN)

    # Hover above
    hover_pos = [target_pos[0], target_pos[1], target_pos[2] + 0.1]
    move_ee_smooth(hover_pos, target_orn, steps=480)

    # Descend
    move_ee_smooth(target_pos, target_orn, steps=400)

    # Release
    open_gripper(steps=80)
    release_object()


def push(target_obj=None, direction="forward", distance=0.1):
    """Push/slide an object along a surface using the hand as a paddle.

    The arm navigates to the opposite side of the object, closes the
    gripper, descends so the hand body contacts the object, then
    translates horizontally to push it.

    Args:
        target_obj: PyBullet body ID. If None, uses tracked obj_id.
        direction: "forward" (+X), "back" (-X), "left" (-Y), "right" (+Y)
        distance: how far to push in meters (default 0.1m)

    Direction mapping (world frame):
        forward = +X (away from robot base)
        back    = -X (toward robot base)
        left    = -Y
        right   = +Y

    Design notes:
        - The hand body (link 8) is 0.065m above link 11
        - Closed gripper fingers extend ~0.04m below the hand
        - The whole hand+fingers assembly acts as a flat paddle
        - Approach from 0.12m away to avoid hitting the block on descent
    """
    oid = target_obj if target_obj is not None else obj_id
    actual_pos = get_object_pos(oid)
    if actual_pos is None:
        print("ERROR: No object to push")
        return

    # Direction vectors (world frame)
    dir_map = {
        "forward": [1, 0],
        "back":    [-1, 0],
        "left":    [0, -1],
        "right":   [0, 1],
    }
    if direction not in dir_map:
        print(f"ERROR: Unknown direction '{direction}'. Use: forward, back, left, right")
        return

    dx, dy = dir_map[direction]
    approach_offset = 0.12  # clearance from object center

    print(f"Pushing object {direction} by {distance:.2f}m")
    target_orn = p.getQuaternionFromEuler(PERP_DOWN_ORN)

    # 1. Close gripper first (forms a solid paddle)
    close_gripper(force=GRIPPER_FORCE_OPEN, max_steps=60, min_warmup=0, settle_threshold=5)

    # 2. Move above the approach point (opposite side of push direction)
    approach_hover = [
        actual_pos[0] - dx * approach_offset,
        actual_pos[1] - dy * approach_offset,
        actual_pos[2] + 0.15
    ]
    move_ee_smooth(approach_hover, target_orn, steps=400)

    # 3. Descend to contact height
    # Target link 11 at object center Z. The hand body and closed
    # fingers extend above this, so the solid parts of the gripper
    # will be at the object's height and can push it.
    contact_pos = [
        actual_pos[0] - dx * approach_offset,
        actual_pos[1] - dy * approach_offset,
        actual_pos[2]
    ]
    move_ee_smooth(contact_pos, target_orn, steps=300)
    settle(60)

    # 4. Translate through the object in the push direction
    push_end = [
        contact_pos[0] + dx * (approach_offset + distance),
        contact_pos[1] + dy * (approach_offset + distance),
        contact_pos[2]
    ]
    move_ee_smooth(push_end, target_orn, steps=600)

    # 5. Retreat upward
    retreat_pos = [push_end[0], push_end[1], push_end[2] + 0.15]
    move_ee_smooth(retreat_pos, target_orn, steps=200)

    print(f"Push complete. Object should have moved {direction} by ~{distance:.2f}m")