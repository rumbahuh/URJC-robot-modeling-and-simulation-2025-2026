import pybullet as p
import pybullet_data
import time

physicsClient = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.8)

planeId = p.loadURDF("plane.urdf")

euler_angles = [0, 0, 1.57] # Inicia con rotación en eje z
startOrientation = p.getQuaternionFromEuler(euler_angles)
startPosition = [0, 0, 1]

robotId = p.loadURDF("husky/husky.urdf", startPosition, startOrientation)

try:
    while True:
        p.stepSimulation()
        time.sleep(1./240.)
		
except KeyboardInterrupt:
      pass

p.disconnect()