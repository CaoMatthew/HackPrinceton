"""
Test: Push Forward
==================
Tests the push skill: arm navigates beside the block and pushes it forward (+X).
Same table setup as the flip test.

Run: python test_push.py
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
table = p.loadURDF("table/table.urdf", [0.5, 0, 0], globalScaling=0.5)
block = p.loadURDF("cube_small.urdf", [0.4, 0, 0.35])

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
    cameraTargetPosition=[0.4, 0, 0.3]
)

# --- Run test ---
actions.init(robot, block)
actions.push(direction="forward", distance=0.15)

# Check where block ended up
final_pos, _ = p.getBasePositionAndOrientation(block)
print(f"\nBlock started at:  {[f'{x:.4f}' for x in actual_pos]}")
print(f"Block ended at:    {[f'{x:.4f}' for x in final_pos]}")
print(f"Displacement X:    {final_pos[0] - actual_pos[0]:.4f}m")

print("\n[OK] Push test complete")

# Keep sim alive
p.setRealTimeSimulation(1)
input("\nPress Enter to exit...")
p.disconnect()
