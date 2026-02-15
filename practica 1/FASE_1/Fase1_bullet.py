import pybullet as p
import pybullet_data
import time

physicsClient = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.8)

planeId = p.loadURDF("plane.urdf")

euler_angles = [0, 0, 1.57] # Initiates figure rotated 90 degrees on Z axis
startOrientation = p.getQuaternionFromEuler(euler_angles)
startPosition = [0, 0, 1]

huskyId = p.loadURDF("husky/husky.urdf", startPosition, startOrientation)

euler_angles = [0, 0, 0]
startOrientation = p.getQuaternionFromEuler(euler_angles)
startPosition = [0, 10, 1]

rampId = p.loadURDF("ramp.urdf", startPosition, startOrientation)

startOrientation = p.getQuaternionFromEuler(euler_angles)
startPosition = [0, 20, 1]

finishLineId = p.loadURDF("finish_line.urdf", startPosition, startOrientation)

startOrientation = p.getQuaternionFromEuler(euler_angles)
startPosition = [-1.5, 17, 1]

barId = p.loadURDF("bar.urdf", startPosition, startOrientation)

husky_wheel_joints = [2, 3, 4, 5]

try:
    while True:
        p.stepSimulation()
        time.sleep(1./240.)
		
        speed = 12
        torque = 20
        p.setJointMotorControlArray(huskyId,
                                    husky_wheel_joints,
                                    p.VELOCITY_CONTROL,
                                    targetVelocities=[speed,speed,speed,speed],
                                    forces=[torque,torque,torque,torque])

except KeyboardInterrupt:
      pass

p.disconnect()