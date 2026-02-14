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
startPosition = [0, 20, 1]

finishLineId = p.loadURDF("finish_line.urdf", startPosition, startOrientation)

startOrientation = p.getQuaternionFromEuler(euler_angles)
startPosition = [-1.5, 17, 1]

barId = p.loadURDF("bar.urdf", startPosition, startOrientation)

hor_id = p.addUserDebugParameter("horizontalMotor", -1.57, 1.57, 0)
speedId = p.addUserDebugParameter("HUSKY_speed", 0, 40, 5)

########
#numJoints = p.getNumJoints(huskyId)
#print("NumJoints: " + str(numJoints))

#for j in range (numJoints):
#    print("%d - %s" % (p.getJointInfo(huskyId,j)[0], p.getJointInfo(huskyId,j)[1].decode("utf-8")))

# NumJoints: 10
# 0 - chassis_joint
# 1 - imu_joint
# 2 - front_left_wheel
# 3 - front_right_wheel
# 4 - rear_left_wheel
# 5 - rear_right_wheel
# 6 - top_plate
# 7 - user_rail
# 8 - front_bumper
# 9 - rear_bumper

#####
husky_wheel_joints = [2,3,4,5]

try:
    while True:
        p.stepSimulation()
        time.sleep(1./240.)
		
        hor_value = p.readUserDebugParameter(hor_id)
        speed = p.readUserDebugParameter(speedId)
        
        p.setJointMotorControl2(barId,0, p.VELOCITY_CONTROL, targetVelocity=hor_value)
        p.setJointMotorControlArray(huskyId,
                                    husky_wheel_joints,
                                    p.VELOCITY_CONTROL,
                                    targetVelocities=[speed,speed,speed,speed])


except KeyboardInterrupt:
      pass

p.disconnect()