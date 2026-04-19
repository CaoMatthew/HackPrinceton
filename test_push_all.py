"""
Test: Push All Directions
=========================
Tests the push skill in all 4 directions (forward, back, left, right).
Each sub-test re-settles a fresh block and measures actual displacement.

Run: python test_push_all.py
"""
import pybullet as p
import pybullet_data
import time
import actions

# -------------------------------------------------------
# HELPERS
# -------------------------------------------------------
BLOCK_START = [0.75, 0, 0.35]
PUSH_DIST   = 0.15
PASS_THRESH = 0.08   # block must move at least this far to count as a pass


def load_scene():
    """Reset sim, load robot + table + block. Returns (robot_id, block_id)."""
    p.resetSimulation()
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.loadURDF("plane.urdf")
    robot = p.loadURDF("franka_panda/panda.urdf", useFixedBase=True)
    p.loadURDF("table/table.urdf", [0.75, 0, 0], globalScaling=0.5)
    block = p.loadURDF("cube_small.urdf", BLOCK_START)

    p.setGravity(0, 0, -9.8)
    for _ in range(240):
        p.stepSimulation()
        time.sleep(1. / 240.)

    return robot, block


def run_push_test(direction):
    """Run a single push test in `direction`. Returns (passed, displacement)."""
    print(f"\n{'='*50}")
    print(f"  TEST: push direction='{direction}'")
    print(f"{'='*50}")

    robot, block = load_scene()
    start_pos, _ = p.getBasePositionAndOrientation(block)
    print(f"  Block settled at: {[f'{x:.4f}' for x in start_pos]}")

    p.resetDebugVisualizerCamera(
        cameraDistance=1.5, cameraYaw=50, cameraPitch=-35,
        cameraTargetPosition=[0.75, 0, 0.3]
    )

    actions.init(robot, block)
    actions.push(direction=direction, distance=PUSH_DIST)

    end_pos, _ = p.getBasePositionAndOrientation(block)

    dx = end_pos[0] - start_pos[0]
    dy = end_pos[1] - start_pos[1]

    disp_map = {
        "forward": dx,
        "back":   -dx,
        "left":   -dy,
        "right":   dy,
    }
    disp = disp_map[direction]
    passed = disp >= PASS_THRESH

    print(f"\n  Start : {[f'{x:.4f}' for x in start_pos]}")
    print(f"  End   : {[f'{x:.4f}' for x in end_pos]}")
    print(f"  dX={dx:+.4f}  dY={dy:+.4f}  => displacement along push axis: {disp:+.4f}m")
    print(f"  {'[PASS]' if passed else '[FAIL]'}  (threshold {PASS_THRESH}m)")

    return passed, disp


# -------------------------------------------------------
# MAIN
# -------------------------------------------------------
p.connect(p.GUI)

directions = ["forward", "left", "right"]
results = {}

for d in directions:
    ok, disp = run_push_test(d)
    results[d] = {"passed": ok, "displacement": disp}
    # Brief pause between tests so you can see the GUI reset
    time.sleep(0.5)

# -------------------------------------------------------
# SUMMARY
# -------------------------------------------------------
print(f"\n{'='*50}")
print("  PUSH TEST SUMMARY")
print(f"{'='*50}")
all_pass = True
for d, r in results.items():
    status = "PASS" if r["passed"] else "FAIL"
    all_pass = all_pass and r["passed"]
    print(f"  {d:<8}  disp={r['displacement']:+.4f}m  [{status}]")

print(f"\n  Overall: {'ALL PASS' if all_pass else 'SOME TESTS FAILED'}")
print(f"{'='*50}")

p.setRealTimeSimulation(1)
input("\nPress Enter to exit...")
p.disconnect()
