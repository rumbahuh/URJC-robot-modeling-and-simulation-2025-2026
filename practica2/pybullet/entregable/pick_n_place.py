import pybullet as p
import time

MAX_FORCE = 100

LINK_SCARA_1 = 4
LINK_SCARA_2 = 5
LINK_PRIS = 7
PINZA_LEFT = 8
PINZA_RIGHT = 9

def move_prismatic(robotId, target, speed=0.4, ee_link=LINK_PRIS, ignore_body=None):
    for _ in range(1000):
        contacts = p.getContactPoints(bodyA=robotId, linkIndexA=ee_link)
        
        real_contacts = []
        for c in contacts:
            if ignore_body is not None and c[2] == ignore_body:
                continue
            if c[2] == 0:
                continue
            real_contacts.append(c)
        
        if real_contacts:
            print("**Contacto detectado, deteniendo")
            break
        
        p.setJointMotorControl2(
            robotId, ee_link, p.POSITION_CONTROL,
            targetPosition=target, force=MAX_FORCE, maxVelocity=speed
        )
        p.stepSimulation()
        time.sleep(1./240.)

def pinza(robotId, target, speed=0.1, ee_link=[PINZA_LEFT, PINZA_RIGHT], ignore_contact=False):
    for _ in range(1000):
        if not ignore_contact:
            contacts_11 = p.getContactPoints(bodyA=robotId, linkIndexA=ee_link[0])
            contacts_12 = p.getContactPoints(bodyA=robotId, linkIndexA=ee_link[1])
            if contacts_11 and contacts_12:
                print("**Contacto detectado, deteniendo pinza")
                break
        
        p.setJointMotorControl2(
            robotId, ee_link[0], p.POSITION_CONTROL,
            targetPosition=target[0], force=MAX_FORCE, maxVelocity=speed
        )
        p.setJointMotorControl2(
            robotId, ee_link[1], p.POSITION_CONTROL,
            targetPosition=target[1], force=MAX_FORCE, maxVelocity=speed
        )
        p.stepSimulation()
        time.sleep(1./240.)

def move_arm(robotId, target, speed=1, ignore_body=None):
    for _ in range(2000):
        current_q2, vel_q2 = p.getJointState(robotId, LINK_SCARA_1)[0:2]
        current_q3, vel_q3 = p.getJointState(robotId, LINK_SCARA_2)[0:2]

        at_position = (abs(current_q2 - target[0]) < 0.01 and
                       abs(current_q3 - target[1]) < 0.01 and
                       abs(vel_q2) < 0.01 and
                       abs(vel_q3) < 0.01)
        if at_position:
            break

        p.setJointMotorControl2(
            robotId, 4, p.POSITION_CONTROL,
            targetPosition=target[0], force=50, maxVelocity=speed
        )
        p.setJointMotorControl2(
            robotId, 5, p.POSITION_CONTROL,
            targetPosition=target[1], force=50, maxVelocity=speed
        )
        p.stepSimulation()
        time.sleep(1./240.)

def twist_pinza(robotId, target, speed=0.3):
    for _ in range(2000):
        current_q4, vel_q4 = p.getJointState(robotId, 6)[0:2]

        at_position = (abs(current_q4 - target) < 0.01 and
                       abs(vel_q4) < 0.01)
        if at_position:
            break

        p.setJointMotorControl2(
            robotId, 6, p.POSITION_CONTROL,
            targetPosition=target, force=50, maxVelocity=speed
        )
        p.stepSimulation()
        time.sleep(1./240.)