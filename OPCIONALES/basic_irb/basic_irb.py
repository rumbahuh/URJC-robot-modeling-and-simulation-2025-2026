import pybullet as p
import pybullet_data
import time

physicsClient = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.8)

planeId = p.loadURDF("plane.urdf")

euler_angles = [0, 0, 0]
startOrientation = p.getQuaternionFromEuler(euler_angles)
startPosition = [0, 0, 1]

robotId = p.loadURDF("basic_irb.urdf", startPosition, startOrientation)

hor_id = p.addUserDebugParameter("horizontalMotor", -1.57, 1.57, 0)
ver_id = p.addUserDebugParameter("verticalMotor", -0.14, 0.14, 0)

try:
    while True:
        p.stepSimulation()
        time.sleep(1./240.)

        hor_value = p.readUserDebugParameter(hor_id)
        ver_value = p.readUserDebugParameter(ver_id)
        
        p.setJointMotorControl2(robotId,1, p.VELOCITY_CONTROL, targetVelocity=hor_value)
        p.setJointMotorControl2(robotId,0, p.VELOCITY_CONTROL, targetVelocity=ver_value)
        
except KeyboardInterrupt:
      pass

p.disconnect()