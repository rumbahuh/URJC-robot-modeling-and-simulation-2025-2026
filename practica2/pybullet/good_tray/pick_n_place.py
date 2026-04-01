import pybullet as p

MAX_FORCE    = 100
LINK_SCARA_1 = 4
LINK_SCARA_2 = 5
LINK_PRIS    = 7
PINZA_LEFT   = 8
PINZA_RIGHT  = 9
LINK_TWIST   = 6

def move_joint(robotId, joint, target, speed):
    for _ in range(2000):
        pos, vel = p.getJointState(robotId, joint)[0:2]
        if abs(pos - target) < 0.01 and abs(vel) < 0.05:
            break
        p.setJointMotorControl2(robotId, joint, p.POSITION_CONTROL,
            targetPosition=target, force=50, maxVelocity=speed)
        p.stepSimulation()

def move_prismatic(robotId, target, speed=0.4, ignore_body=None):
    for _ in range(1000):
        contacts = p.getContactPoints(bodyA=robotId, linkIndexA=LINK_PRIS)
        for c in contacts:
            if c[2] == 0 or c[2] == ignore_body:
                continue
            print("Contacto detectado, deteniendo")
            return
        p.setJointMotorControl2(robotId, LINK_PRIS, p.POSITION_CONTROL,
            targetPosition=target, force=MAX_FORCE, maxVelocity=speed)
        p.stepSimulation()

def pinza(robotId, target, speed=0.1, ignore_contact=False):
    for _ in range(1000):
        left  = p.getContactPoints(bodyA=robotId, linkIndexA=PINZA_LEFT)
        right = p.getContactPoints(bodyA=robotId, linkIndexA=PINZA_RIGHT)
        if not ignore_contact and left and right:
            print("Contacto detectado, deteniendo pinza")
            break
        p.setJointMotorControl2(robotId, PINZA_LEFT, p.POSITION_CONTROL,
            targetPosition=target[0], force=MAX_FORCE, maxVelocity=speed)
        p.setJointMotorControl2(robotId, PINZA_RIGHT, p.POSITION_CONTROL,
            targetPosition=target[1], force=MAX_FORCE, maxVelocity=speed)
        p.stepSimulation()

def move_arm(robotId, target, speed=0.5):
    move_joint(robotId, LINK_SCARA_1, target[0], speed)
    move_joint(robotId, LINK_SCARA_2, target[1], speed)

def twist_pinza(robotId, target, speed=0.3):
    move_joint(robotId, LINK_TWIST, target, speed)