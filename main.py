import pybullet as p
import pybullet_data
import time

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

cube = p.loadURDF("cube_small.urdf", [0.5, 0, 0.1])
for i in range(200):
    p.setJointMotorControl2(robot, 2, p.POSITION_CONTROL, targetPosition=1.0)
    p.stepSimulation()
    time.sleep(1/240)

for _ in range(10000):
    p.stepSimulation()
    time.sleep(1/240)