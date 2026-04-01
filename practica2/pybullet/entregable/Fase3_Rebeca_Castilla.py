import pybullet as p
import pybullet_data
import time
import csv

q2_q3_FRONT      = [1.63, 1.57]
SCARA_ARM_JOINTS = [4, 5]
SCARA_ROT_PRIS = 6
LINK_PRIS    = 7
PINZA_LEFT   = 8
PINZA_RIGHT  = 9

WHEEL_JOINTS     = [13, 15, 18, 20]

BRAKE_Y          = 1.3
BRAKE_STEPS      = 120
RUN_BRAKE_Y      = 8.0
RUN_BRAKE_STEPS  = 180

NUMERO_JOINTS = 2

CLOSE_POSE = [0.48, 0.48]
OPEN_POSE = [0.0, 0.0]

ARM_TO_CUBE = 2
ARM_HOME = 0
CUBE_TO_CHASSIS = 1.4

CUBE_REST_PLACES = [[-0.5, 0.0], [-1, 0.0], [-1.4, 0.0]]

MAX_FORCE    = 100

counter = 0

def set_wheels(speed, torque):
    p.setJointMotorControlArray(robotId, WHEEL_JOINTS,
        p.VELOCITY_CONTROL,
        targetVelocities=[speed, speed, speed, speed],
        forces=[torque, torque, torque, torque])

def robot_y():
    return p.getBasePositionAndOrientation(robotId)[0][1]

def joint(j):
    return p.getJointState(robotId, j)[0:2]  # pos, vel

def move_to_cube():
	global state, counter

	if robot_y() < BRAKE_Y:
		set_wheels(10, 20)
	else:
		set_wheels(0, 0)
	
	if robot_y() >= BRAKE_Y:
		state, counter = "BRAKE", 0

def stop_rover():
	global state, counter

	set_wheels(0, 100)

	counter = counter + 1
	if counter >= BRAKE_STEPS:
		state = "SCARA_SEQ"

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
    move_joint(robotId, SCARA_ARM_JOINTS[0], target[0], speed)
    move_joint(robotId, SCARA_ARM_JOINTS[1], target[1], speed)

def twist_pinza(robotId, target, speed=0.3):
    move_joint(robotId, SCARA_ROT_PRIS, target, speed)

def run_sequence(robotId, cubeId, index):
    move_prismatic(robotId, ARM_TO_CUBE, speed=0.4)
    pinza(robotId, CLOSE_POSE, speed=0.7)
    move_prismatic(robotId, ARM_HOME, speed=0.4, ignore_body=cubeId)
    move_arm(robotId, CUBE_REST_PLACES[index])
    twist_pinza(robotId, 0.7, speed=0.3) # Usar cin inversa para sacar los grados concretos
    move_prismatic(robotId, CUBE_TO_CHASSIS, speed=0.4, ignore_body=cubeId)
    pinza(robotId, OPEN_POSE, speed=0.7, ignore_contact=True)
    move_prismatic(robotId, ARM_HOME, speed=0.4, ignore_body=cubeId)

def pick_n_place():
	global state, stable

	p.setJointMotorControlArray(robotId, SCARA_ARM_JOINTS,
                p.POSITION_CONTROL, targetPositions=q2_q3_FRONT, forces=[50, 50])

	q2, v2 = joint(SCARA_ARM_JOINTS[0])
	q3, v3 = joint(SCARA_ARM_JOINTS[1])
	on_target = (abs(q2 - q2_q3_FRONT[0]) < 0.01 and
			     abs(q3 - q2_q3_FRONT[1]) < 0.01 and
				 abs(v2) < 0.01 and abs(v3) < 0.01)
	
	if on_target:
		stable = stable + 1
	else:
		stable = 0

	if stable >= 50:
		run_sequence(robotId, cubeId, 0)
		state = "DONE"

physicsClient = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.8)

p.loadURDF("plane_transparent.urdf")
robotId = p.loadURDF("../rover/urdf/rover.urdf", [0,0,1], p.getQuaternionFromEuler([0,0,-3.15]))
cubeId  = p.loadURDF("../cubo/urdf/cubo.urdf",  [0,4,0], p.getQuaternionFromEuler([0,0,-3.15]))

with open('practica2.csv', 'w', newline='') as csvfile:
    writer = csv.writer(csvfile)
    writer.writerow(['Tiempo', 'NumeroJoints', 'G_parcial'])

state   = "DRIVE"

dt = 0.01
t = time.time()
try:
	while True:
		p.stepSimulation()
		time.sleep(0.005) # specification

		if state == "DRIVE":
			move_to_cube()
		elif state == "BRAKE":
			stop_rover()
		elif state == "SCARA_SEQ":
			pick_n_place()
			
		t = time.time() - t
		if (t == dt):
			F1 = p.getJointState(SCARA_ARM_JOINTS[0])[2]
			F3 = p.getJointState(SCARA_ROT_PRIS)[2] # we get forces of two joints
			gParcial = F1 + F3 # add them
			with open('Fase2_data.csv', 'a', newline='') as csvfile:
				writer = csv.writer(csvfile)
				writer.writerow([t, NUMERO_JOINTS, gParcial])

except KeyboardInterrupt:
	pass