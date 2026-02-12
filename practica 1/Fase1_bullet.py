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

huskyId = p.loadURDF("husky/husky.urdf", startPosition, startOrientation)

euler_angles = [0, 0, 0]
startOrientation = p.getQuaternionFromEuler(euler_angles)
startPosition = [0, 10, 1]

rampId = p.loadURDF("ramp.urdf", startPosition, startOrientation)

startOrientation = p.getQuaternionFromEuler(euler_angles)
startPosition = [0, 21, 1]

finishLineId = p.loadURDF("finish_line.urdf", startPosition, startOrientation)

startOrientation = p.getQuaternionFromEuler(euler_angles)
startPosition = [-1.5, 17, 1]

barId = p.loadURDF("bar.urdf", startPosition, startOrientation)

hor_id = p.addUserDebugParameter("horizontalMotor", -1.57, 1.57, 0)

try:
    while True:
        p.stepSimulation()
        time.sleep(1./240.)
		
        hor_value = p.readUserDebugParameter(hor_id)
        
        p.setJointMotorControl2(barId,0, p.VELOCITY_CONTROL, targetVelocity=hor_value)

except KeyboardInterrupt:
      pass

p.disconnect()