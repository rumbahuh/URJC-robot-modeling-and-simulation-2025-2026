import pybullet as p
import pybullet_data
import time
from practica2.pybullet.good_tray.sequence import run_sequence

FRONT           = [1.63, 1.57]
SCARA_JOINTS    = [4, 5]
WHEEL_JOINTS    = [13, 15, 18, 20]
BRAKE_Y         = 1.3
BRAKE_STEPS     = 120
RUN_BRAKE_Y     = 8.0
RUN_BRAKE_STEPS = 180

p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.8)
p.loadURDF("plane_transparent.urdf")
robotId = p.loadURDF("../rover/urdf/rover.urdf", [0,0,1], p.getQuaternionFromEuler([0,0,-3.15]))
cubeId  = p.loadURDF("../cubo/urdf/cubo.urdf",  [0,4,0], p.getQuaternionFromEuler([0,0,-3.15]))

def set_wheels(speed, torque):
    p.setJointMotorControlArray(robotId, WHEEL_JOINTS,
        p.VELOCITY_CONTROL,
        targetVelocities=[speed] * 4,
        forces=[torque] * 4)

def robot_y():
    return p.getBasePositionAndOrientation(robotId)[0][1]

def joint(j):
    return p.getJointState(robotId, j)[0:2]  # pos, vel

state   = "DRIVE"
counter = 0
stable  = 0

try:
    while True:
        p.stepSimulation()
        time.sleep(0.005)

        if state == "DRIVE":
            set_wheels(10, 20) if robot_y() < BRAKE_Y else set_wheels(0, 0)
            if robot_y() >= BRAKE_Y:
                state, counter = "BRAKE", 0

        elif state == "BRAKE":
            set_wheels(0, 100)
            counter += 1
            if counter >= BRAKE_STEPS:
                state = "SCARA_SEQ"

        elif state == "SCARA_SEQ":
            p.setJointMotorControlArray(robotId, SCARA_JOINTS,
                p.POSITION_CONTROL, targetPositions=FRONT, forces=[50, 50])

            q2, v2 = joint(4)
            q3, v3 = joint(5)
            on_target = (abs(q2 - FRONT[0]) < 0.01 and abs(q3 - FRONT[1]) < 0.01
                         and abs(v2) < 0.01 and abs(v3) < 0.01)
            stable = stable + 1 if on_target else 0

            if stable >= 50:
                run_sequence(robotId, cubeId, 0)
                state, counter = "RUN", 0

        elif state == "RUN":
            set_wheels(25, 50) if robot_y() < RUN_BRAKE_Y else set_wheels(0, 0)
            if robot_y() >= RUN_BRAKE_Y:
                state, counter = "RUN_BRAKE", 0

        elif state == "RUN_BRAKE":
            set_wheels(0, 150)
            counter += 1
            if counter >= RUN_BRAKE_STEPS:
                state = "DONE"
                print("Done!")

except KeyboardInterrupt:
    pass

p.disconnect()