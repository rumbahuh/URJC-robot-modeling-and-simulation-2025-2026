import pybullet as p
import pybullet_data
import time
from sequence import run_sequence

FRONT = [1.63, 1.57]
DELTA = 0.01
Y = 1

BRAKE_STEPS = 120
BRAKE_Y_THRESHOLD = 1.3   # start braking when rover reaches Y=3.5

physicsClient = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.8)
planeId = p.loadURDF("plane_transparent.urdf")

startPos = [0, 0, 1]
startOrientation = p.getQuaternionFromEuler([0, 0, -3.15])
robotId = p.loadURDF("../rover/urdf/rover.urdf", startPos, startOrientation)

startPosCube = [0, 4, 0]
startOrientationCube = p.getQuaternionFromEuler([0, 0, -3.15])
cubeId = p.loadURDF("../cubo/urdf/cubo.urdf", startPosCube, startOrientationCube)

scara_rot_joints = [4, 5]
rover_wheel_joints = [13, 15, 18, 20]

stable_count = 0
stable_threshold = 50
arm_stable = False
sequence_done = False
brake_counter = 0
state = "DRIVE"
RUN_SPEED = 25
RUN_TORQUE = 50
RUN_BRAKE_Y_THRESHOLD = 8.0
RUN_BRAKE_STEPS = 180

try:
    while True:
        p.stepSimulation()
        time.sleep(1. / 240.)

        if state == "DRIVE":
            posicion_robot, _ = p.getBasePositionAndOrientation(robotId)

            if posicion_robot[Y] < BRAKE_Y_THRESHOLD:
                speed, torque = 10, 20
            else:
                speed, torque = 0, 0

            p.setJointMotorControlArray(robotId, rover_wheel_joints,
                p.VELOCITY_CONTROL,
                targetVelocities=[speed] * 4,
                forces=[torque] * 4)

            if posicion_robot[Y] >= BRAKE_Y_THRESHOLD:
                state = "BRAKE"
                brake_counter = 0
                print("Switching to BRAKE")

        elif state == "BRAKE":
            p.setJointMotorControlArray(robotId, rover_wheel_joints,
                p.VELOCITY_CONTROL,
                targetVelocities=[0, 0, 0, 0],
                forces=[100, 100, 100, 100])

            brake_counter += 1
            if brake_counter >= BRAKE_STEPS:
                state = "SCARA_SEQ"
                print("Switching to SCARA_SEQ")

        elif state == "SCARA_SEQ":
            if not arm_stable:
                p.setJointMotorControlArray(
                    robotId, scara_rot_joints, p.POSITION_CONTROL,
                    targetPositions=FRONT, forces=[50, 50]
                )

                current_q2 = p.getJointState(robotId, 4)[0]
                current_q3 = p.getJointState(robotId, 5)[0]
                vel_q2     = p.getJointState(robotId, 4)[1]
                vel_q3     = p.getJointState(robotId, 5)[1]

                at_position = (abs(current_q2 - 1.63) < DELTA and
                               abs(current_q3 - 1.57) < DELTA)
                is_still    = (abs(vel_q2) < DELTA and
                               abs(vel_q3) < DELTA)

                if at_position and is_still:
                    stable_count += 1
                else:
                    stable_count = 0

                if stable_count >= stable_threshold:
                    arm_stable = True

            if arm_stable and not sequence_done:
                run_sequence(robotId, cubeId, 0)
                state = "RUN"
                sequence_done = True
        
        elif state == "RUN":
            posicion_robot, _ = p.getBasePositionAndOrientation(robotId)

            if posicion_robot[Y] < RUN_BRAKE_Y_THRESHOLD:
                speed, torque = RUN_SPEED, RUN_TORQUE
            else:
                speed, torque = 0, 0

            p.setJointMotorControlArray(robotId, rover_wheel_joints,
                p.VELOCITY_CONTROL,
                targetVelocities=[speed] * 4,
                forces=[torque] * 4)

            if posicion_robot[Y] >= RUN_BRAKE_Y_THRESHOLD:
                state = "RUN_BRAKE"
                brake_counter = 0
                print("Switching to RUN_BRAKE")

        elif state == "RUN_BRAKE":
            p.setJointMotorControlArray(robotId, rover_wheel_joints,
                p.VELOCITY_CONTROL,
                targetVelocities=[0, 0, 0, 0],
                forces=[150, 150, 150, 150])

            brake_counter += 1
            if brake_counter >= RUN_BRAKE_STEPS:
                state = "DONE"
                print("Done!")

        elif state == "DONE":
            pass

except KeyboardInterrupt:
    pass

p.disconnect()