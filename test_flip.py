"""
Test: Pick and Flip
===================
Proven working test for the move_to -> grasp -> lift -> flip sequence.
Block on a 0.5x scaled table at [0.7, 0, 0].

Run: python test_flip.py
"""
import pybullet as p
import pybullet_data
import time
import actions

# --- Setup ---
p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.loadURDF("plane.urdf")
robot = p.loadURDF("franka_panda/panda.urdf", useFixedBase=True)

# --- Scene ---
table = p.loadURDF("table/table.urdf", [0.7, 0, 0], globalScaling=0.5)
block = p.loadURDF("cube_small.urdf", [0.7, 0, 0.35])

# --- Gravity + settle ---
p.setGravity(0, 0, -9.8)
for _ in range(240):
    p.stepSimulation()
    time.sleep(1./240.)

actual_pos, _ = p.getBasePositionAndOrientation(block)
print(f"Block settled at: {[f'{x:.4f}' for x in actual_pos]}")

# --- Camera ---
p.resetDebugVisualizerCamera(
    cameraDistance=1.5, cameraYaw=50, cameraPitch=-35,
    cameraTargetPosition=[0.5, 0, 0]
)

# --- Run test ---
actions.init(robot, block)
actions.move_to()
actions.grasp()
actions.lift(0.4)
actions.flip()

print("\n✓ Flip test complete")

# Keep sim alive
p.setRealTimeSimulation(1)
input("\nPress Enter to exit...")
p.disconnect()
