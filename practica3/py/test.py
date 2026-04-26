import pybullet as p
import pybullet_data
import time
import math
import csv

#for i in range(p.getNumJoints(robotId)):
#   print(i, p.getJointInfo(robotId, i)[1])
SCARA_ARM_JOINTS = [4, 5]
SCARA_ROT_PRIS = 6
LINK_PRIS = 7
PINZA_LEFT = 8
PINZA_RIGHT = 9

WHEEL_JOINTS = [13, 15, 18, 20]

BRAKE_Y = 1.3

NUMERO_JOINTS = 4

CLOSE_POSE = [0.48, 0.48]
OPEN_POSE = [0.0, 0.0]

HOME = [0, 0, 0]
CUBE = [-2, 3.6, 1.9]

CUBE_REST_PLACES = [[1.8, -0.9, 0.32]]

MAX_FORCE = 500

dt = 0.01


def log():
    """
    Compares time with the last logged time. If
    the abs of it is higer or equal to dt, it keeps
    track of parcial costs on the SCARA arm joints.

    Writes it on the csv and updates the logged time.
    """

    global last_log
    now = time.time()

    if now - last_log >= dt:
        F1 = p.getJointState(robotId, SCARA_ARM_JOINTS[0])[3]
        F2 = p.getJointState(robotId, SCARA_ARM_JOINTS[1])[3]
        F3 = p.getJointState(robotId, SCARA_ROT_PRIS)[3]
        gParcial = F1 + F2 + F3

        with open('Fase3_rebeca_castilla.csv', 'a', newline='') as csvfile:
            writer = csv.writer(csvfile)
            writer.writerow([round(now - t, 4), NUMERO_JOINTS, gParcial])

        last_log = now


def set_wheels(speed, torque):
    """
    Set speed and force on all wheel joints.
    """

    p.setJointMotorControlArray(robotId,
                                WHEEL_JOINTS,
                                p.VELOCITY_CONTROL,
                                targetVelocities=[speed, speed, speed, speed],
                                forces=[torque, torque, torque, torque])


def robot_y():
    """
    Returns orientation and position of y axis.
    """

    return p.getBasePositionAndOrientation(robotId)[0][1]


def joint(j):
    """
    Returns joint state.
    """

    return p.getJointState(robotId, j)[0:2]


def move_to_cube():
    """
    If the y axis position of the robot is
    less than the maximum position to stop moving,
    it sets positive speed, else, it sets null speed.

    If the y axis position is higer than the maximum
    position to stop moving, it needs to fix inertial
    issues. We change state to BRAKE.
    """

    global state

    if robot_y() < BRAKE_Y:
        set_wheels(10, 20)
    else:
        set_wheels(0, 0)
        state = "BRAKE"


def stop_rover():
    """
    Sets force to the wheels and null speed.
    """

    global state, counter, t

    set_wheels(0, 100)

    state = "SCARA_SEQ"
    t = time.time()


def calculate_ik(coor, l1=1, l2=1):
    """
    See citation .ris for more details. Inverse kinematics in Appendix B.
    """

    tx = coor[0]
    ty = coor[1]
    tz = coor[2]

    num = tx**2 + ty**2 - l1**2 - l2**2
    denom = 2 * l1 * l2
    cos2 = (num / denom)
    q2 = math.atan2(abs((1 - cos2**2))**(1 / 2), cos2)

    k1 = l1 + l2 * cos2
    k2 = l2 * math.sin(q2)
    q1 = math.atan2(ty, tx) - math.atan2(k2, k1)

    r21 = 0
    r11 = 1
    q3 = math.atan2(r21, r11) - q1 - q2

    d4 = tz

    return [q1, q2, q3, d4]


def move_joint(robotId, joint_n, target, speed):
    """
    Sets speed and force to specific joint.

    If the distance between desired pos and current
    is less than 0.01, it stops the loop.
    """

    for _ in range(3000):
        pos, vel = joint(joint_n)
        if abs(pos - target) < dt:
            break

        p.setJointMotorControl2(robotId,
                                joint_n,
                                p.POSITION_CONTROL,
                                targetPosition=target,
                                force=50,
                                maxVelocity=speed)

        p.stepSimulation()
        log()


def pinza(robotId, target, speed=0.1, ignore_contact=False):
    """
    For a specific number of iterations it checks the contact points
    on the gripper.

    If contact happens on both sides, it breaks the iteration,
    otherwise it sets speed and force alongside the prismatic
    of the gripper.
    """

    for _ in range(1000):
        left = p.getContactPoints(bodyA=robotId, linkIndexA=PINZA_LEFT)
        right = p.getContactPoints(bodyA=robotId, linkIndexA=PINZA_RIGHT)
        if not ignore_contact and left and right:
            break

        p.setJointMotorControl2(robotId,
                                PINZA_LEFT,
                                p.POSITION_CONTROL,
                                targetPosition=target[0],
                                force=MAX_FORCE,
                                maxVelocity=speed)
        p.setJointMotorControl2(robotId,
                                PINZA_RIGHT,
                                p.POSITION_CONTROL,
                                targetPosition=target[1],
                                force=MAX_FORCE,
                                maxVelocity=speed)

        p.stepSimulation()
        log()


def move_arm(robotId, target, speed=0.5):
    """
    Abstraction for moving the two arms of SCARA robot.
    """

    move_joint(robotId, SCARA_ARM_JOINTS[0], target[0], speed)
    move_joint(robotId, SCARA_ARM_JOINTS[1], target[1], speed)


def twist_pinza(robotId, target, speed=0.3):
    """
    Abstraction to move the rotation joint on SCARA robot.
    """

    move_joint(robotId, SCARA_ROT_PRIS, target, speed)


def move_to(coor, speed=0.2):
    """
    Abstraction to move SCARA TCP to PX, PY, PZ coordinates
    using inverse kinematics and abstractions of movement.
    """

    q1, q2, q3, d4 = calculate_ik(coor)
    move_arm(robotId, [q1, q2], speed=speed)
    twist_pinza(robotId, q3, speed=speed / 2)
    move_joint(robotId, LINK_PRIS, d4, speed=speed)


def pick_n_place():
    """
    Abstraction to create trayectory and execute said one.

    As if it finishes, changes STATE to "DONE".
    """

    global state

    move_to(CUBE)
    pinza(robotId, CLOSE_POSE, speed=0.3)

    move_joint(robotId, LINK_PRIS, 0, speed=0.2)

    move_to(CUBE_REST_PLACES[0])
    pinza(robotId, OPEN_POSE, speed=0.3, ignore_contact=True)

    move_joint(robotId, LINK_PRIS, 0, speed=0.2)

    move_to(HOME)

    state = "DONE"


physicsClient = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.8)

p.loadURDF("plane_transparent.urdf")
robotId = p.loadURDF("../rover/urdf/rover.urdf", [0, 0, 1],
                     p.getQuaternionFromEuler([0, 0, -3.14]))
cubeId = p.loadURDF("../cubo/urdf/cubo.urdf", [0, 4, 0],
                    p.getQuaternionFromEuler([0, 0, -3.14]))

with open('Fase3_rebeca_castilla.csv', 'w', newline='') as csvfile:
    writer = csv.writer(csvfile)
    writer.writerow(['Tiempo', 'NumeroJoints', 'G_parcial'])

state = "DRIVE"
t = 0.0
last_log = time.time()
try:
    while True:
        p.stepSimulation()
        time.sleep(0.005)

        if state == "DRIVE":
            move_to_cube()
        elif state == "BRAKE":
            stop_rover()
        elif state == "SCARA_SEQ":
            pick_n_place()
        elif state == "DONE":
            break

except KeyboardInterrupt:
    pass