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
_num_joints = 12       # set in init(); used to size null-space IK arrays
_step_callback = None  # optional per-step hook: fn(step_idx) — set externally for logging
_motion_phase = ""     # human-readable label for the current motion segment

# ── Null-space IK: bias the solver toward this "elbow-up" rest pose ──────────
# Joint indices 0-6 = arm, 7-8 = fixed links, 9-10 = fingers, 11 = fixed.
# J3 (panda_joint4) = -2.356 keeps the elbow strongly raised above the table.
# J5 (panda_joint6) = 1.571 keeps the wrist clear of the robot body.
# Padding to 12 covers every joint in franka_panda/panda.urdf.
_FRANKA_REST   = [0, -0.785, 0, -2.356, 0, 1.571, 0.785, 0, 0, 0.04, 0.04, 0]

# Per-direction base-joint (joint 0) bias.  Rotating the base toward the
# push side keeps the forearm (link5) elevated and clear of the table.
#   left  (+Y approach from right, arm must swing left)  → base +0.5 rad
#   right (-Y approach from left, arm must swing right)  → base -0.5 rad
_DIR_BASE_BIAS = {"forward": 0.0, "left": 0.5, "right": -0.5}
_FRANKA_LOWER  = [-2.90, -1.76, -2.90, -3.07, -2.90, -0.02, -2.90,
                  0, 0, 0.0,  0.0,  0]
_FRANKA_UPPER  = [ 2.90,  1.76,  2.90, -0.07,  2.90,  3.75,  2.90,
                  0, 0, 0.04, 0.04,  0]
_FRANKA_RANGES = [u - l for u, l in zip(_FRANKA_UPPER, _FRANKA_LOWER)]

# Tuned constants
GRIPPER_OPEN = 0.04          # max finger opening (meters)
GRIPPER_CLOSED = 0.0         # fully closed position
GRIPPER_NEAR_CLOSED = 0.005  # near-closed paddle for push (5mm per finger)
GRIP_FORCE = 5.0             # gentle grip — won't squeeze out small objects
GRIPPER_FORCE_OPEN = 18.0    # force for opening fingers
ARM_FORCE = 300              # joint motor force for arm motion
IK_ITERATIONS = 500          # high iteration IK for precision
IK_THRESHOLD = 1e-5          # IK residual threshold
PERP_DOWN_ORN = [math.pi, 0, 0]          # claw pointing straight down
SIDE_PUSH_ORN = [math.pi, 0, math.pi / 2]  # PERP_DOWN + 90° yaw: wide palm face sideways


def init(r, obj=None):
    """Initialize the skills library with a robot and optional tracked object.

    Args:
        r: PyBullet body ID of the Franka Panda robot
        obj: PyBullet body ID of the object to manipulate (optional)
    """
    global robot, obj_id, _num_joints
    robot = r
    obj_id = obj
    _num_joints = p.getNumJoints(r)  # used to size null-space IK arrays


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

        # Null-space IK: provide joint limits + rest pose so the solver
        # prefers the "elbow-up" configuration, keeping all arm links well
        # above the table surface and away from manipulated objects.
        # The rest pose (J3=-2.356) biases the elbow strongly upward.
        n = _num_joints
        jointPoses = p.calculateInverseKinematics(
            robot, endEffectorIndex, interp_pos,
            targetOrientation=target_orn,
            lowerLimits=_FRANKA_LOWER[:n],
            upperLimits=_FRANKA_UPPER[:n],
            jointRanges=_FRANKA_RANGES[:n],
            restPoses=_FRANKA_REST[:n],
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
        if _step_callback is not None:
            _step_callback(i)
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
    """Push/slide an object along a surface using the hand as a flat paddle.

    The wrist is rotated so the gripper tool axis points in the push direction
    (horizontal orientation) with fingers opening horizontally (parallel to the
    table surface) for maximum table clearance. The nearly-closed fingers form
    a flat face that contacts the block squarely at its center height. The
    entire push stroke stays at a constant Z, parallel to the table.

    "back" (-X) is intentionally unsupported. Pulling an object toward the
    robot base requires a pick-and-place, not a push.

    Args:
        target_obj: PyBullet body ID. If None, uses tracked obj_id.
        direction: "forward" (+X), "left" (-Y), "right" (+Y)
        distance: how far to push in meters (default 0.1m)

    Direction mapping (world frame):
        forward = +X (away from robot base)
        left    = -Y
        right   = +Y

    Design notes:
        - Wrist oriented so tool Z faces the push direction AND fingers open
          horizontally (parallel to table) in all cases.  All three share
          the same pitch (π/2) — only the yaw differs:
            forward: [0, π/2,    0] — tool Z→+X, fingers→±Y
            left:    [0, π/2, -π/2] — tool Z→-Y, fingers→±X
            right:   [0, π/2, +π/2] — tool Z→+Y, fingers→±X
        - Navigation to the approach point uses PERP_DOWN (elbow-up, same
          as grasp/flip) to avoid table collisions — wrist rotates to push
          orientation only once safely parked above the contact column.
        - Gripper locked at GRIPPER_NEAR_CLOSED (5mm) — solid paddle face
        - Constant Z throughout the push stroke keeps motion table-parallel
    """
    oid = target_obj if target_obj is not None else obj_id
    actual_pos = get_object_pos(oid)
    if actual_pos is None:
        print("ERROR: No object to push")
        return

    # Direction vectors + horizontal wrist orientations.
    # Each orientation has tool Z pointing in the push direction AND fingers
    # (tool Y axis) horizontal (±X or ±Y world) so neither finger dips toward
    # the table surface.
    #
    # All three share the same wrist pitch (π/2 tilts tool Z horizontal),
    # differing only by a yaw that aims the tool at the push direction.
    # Fingers (tool Y) stay horizontal for all three — no table clearance issues.
    #   forward: yaw=0      tool Z → +X
    #   left:    yaw=-π/2   tool Z → -Y
    #   right:   yaw=+π/2   tool Z → +Y
    dir_map = {
        "forward": ([1,  0], [0, math.pi / 2,  0            ]),
        "left":    ([0, -1], [0, math.pi / 2, -math.pi / 2  ]),
        "right":   ([0,  1], [0, math.pi / 2,  math.pi / 2  ]),
    }
    if direction not in dir_map:
        print(f"ERROR: Unknown direction '{direction}'. "
              f"Supported: forward, left, right.  "
              f"(Use pick+place to move an object backward.)")
        return

    dx, dy = dir_map[direction][0]
    target_orn = p.getQuaternionFromEuler(dir_map[direction][1])
    perp_down_orn = p.getQuaternionFromEuler(PERP_DOWN_ORN)
    approach_offset = 0.15

    print(f"Pushing object {direction} by {distance:.2f}m")

    # Strategy per direction:
    #
    # left / right — vertical-claw approach:
    #   Claw descends from above beside the block (PERP_DOWN), then rotates 90°
    #   at hover height (SIDE_PUSH_ORN) so the wide palm face contacts the side
    #   of the block.  Keeps arm in elbow-up posture throughout; zero table
    #   collisions observed in testing.
    #
    # forward — horizontal-wrist approach:
    #   A vertical approach causes the fingers to sink into the table as the arm
    #   fully extends in X.  Instead, the wrist rotates to aim the tool-Z axis
    #   along +X (horizontal), so the fingertip face contacts the block squarely
    #   like a pool cue.  Approach is from PERP_DOWN; wrist rotates at hover.
    side_push = direction in ("left", "right")
    approach_offset = 0.06 if side_push else 0.15

    side_push_orn = p.getQuaternionFromEuler(SIDE_PUSH_ORN)
    push_orn = side_push_orn if side_push else target_orn

    # 1. Lock gripper in near-closed position — solid flat paddle.
    global _motion_phase
    _motion_phase = "gripper_close"
    for _ in range(80):
        p.setJointMotorControl2(robot, 9,  p.POSITION_CONTROL,
                                targetPosition=GRIPPER_NEAR_CLOSED,
                                force=GRIPPER_FORCE_OPEN)
        p.setJointMotorControl2(robot, 10, p.POSITION_CONTROL,
                                targetPosition=GRIPPER_NEAR_CLOSED,
                                force=GRIPPER_FORCE_OPEN)
        p.stepSimulation()
        time.sleep(1. / 240.)

    safe_x = actual_pos[0] - dx * approach_offset
    safe_y = actual_pos[1] - dy * approach_offset

    # 2a. Navigate to clearance height with PERP_DOWN (elbow-up, same as grasp).
    _motion_phase = "safe_lift"
    clearance_z = actual_pos[2] + 0.35
    move_ee_smooth([safe_x, safe_y, clearance_z], perp_down_orn, steps=480)

    # 2b. Drop to hover height — still PERP_DOWN.
    _motion_phase = "hover"
    hover_pos = [safe_x, safe_y, actual_pos[2] + 0.15]
    move_ee_smooth(hover_pos, perp_down_orn, steps=240)

    # 2c. Rotate wrist to push orientation in-place at hover height.
    #     Side: PERP_DOWN → SIDE_PUSH_ORN (90° yaw, wide palm face sideways).
    #     Forward: PERP_DOWN → horizontal target_orn (tool Z → +X).
    _motion_phase = "wrist_rotate"
    move_ee_smooth(hover_pos, push_orn, steps=240)

    # 3. Descend straight down to block center Z.
    _motion_phase = "descend"
    contact_pos = [safe_x, safe_y, actual_pos[2]]
    move_ee_smooth(contact_pos, push_orn, steps=300)
    settle(60)

    # 4. Translate in push direction at constant Z (table-parallel stroke).
    _motion_phase = "push_stroke"
    push_end = [
        contact_pos[0] + dx * (approach_offset + distance),
        contact_pos[1] + dy * (approach_offset + distance),
        contact_pos[2],
    ]
    move_ee_smooth(push_end, push_orn, steps=800)

    # 5. Retreat upward.
    _motion_phase = "retreat"
    retreat_pos = [push_end[0], push_end[1], push_end[2] + 0.15]
    move_ee_smooth(retreat_pos, push_orn, steps=200)

    print(f"Push complete. Object should have moved {direction} by ~{distance:.2f}m")