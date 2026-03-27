import pybullet as p
import pybullet_data
import time
from sequence import run_sequence

FRONT = [1.57, 1.57]
DELTA = 0.01

physicsClient = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.8)
planeId = p.loadURDF("plane_transparent.urdf")

startPos = [0, 0, 1]
startOrientation = p.getQuaternionFromEuler([0, 0, -3.15])
robotId = p.loadURDF("../rover/urdf/rover.urdf", startPos, startOrientation)

startPosCube = [0.1, 2.1, 1]
startOrientationCube = p.getQuaternionFromEuler([0, 0, -3.15])
cubeId = p.loadURDF("../cubo/urdf/cubo.urdf", startPosCube, startOrientationCube)

scara_rot_joints = [7, 8]

stable_count = 0
stable_threshold = 50
arm_stable = False
sequence_done = False

try:
    while True:
        p.stepSimulation()
        time.sleep(1./240.)

        if not arm_stable:
            # Command arm to target
            p.setJointMotorControlArray(
                robotId, scara_rot_joints, p.POSITION_CONTROL,
                targetPositions=FRONT, forces=[50, 50]
            )

            current_q2 = p.getJointState(robotId, 7)[0]
            current_q3 = p.getJointState(robotId, 8)[0]
            vel_q2     = p.getJointState(robotId, 7)[1]
            vel_q3     = p.getJointState(robotId, 8)[1]

            at_position = (abs(current_q2 - 1.57) < DELTA and
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
            sequence_done = True

except KeyboardInterrupt:
    pass

p.disconnect()