import pybullet as p
import pybullet_data
import time
import math
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

NUMERO_JOINTS = 4

CLOSE_POSE = [0.48, 0.48]
OPEN_POSE = [0.0, 0.0]

ARM_TO_CUBE = 2
ARM_HOME = 0
CUBE_TO_CHASSIS = 1.4

# Measured in blender by eye and cursor
HOME = [0, 0, 0]

CUBE = [-2, 3.6, 1.9]
AUX_CUBE = [-2, 3.6, 0]

CUBE_REST_PLACES = [[1.8, -0.9, 0.32]]

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

def calculate_ik(coor, l1=1, l2=1):
	"""
	See citation .ris for more details. Inverse kinematics in Appendix B.
	"""
	tx = coor[0]
	ty = coor[1]
	tz = coor[2]
	
    # Sacamos q2 usando cos(q2)
	num = tx**2 + ty**2 - l1**2 - l2**2
	denom = 2 * l1 * l2
	cos2 = (num / denom)
	q2 = math.atan2(abs((1 - cos2**2))**(1/2), cos2)
    
    # Sacamos q1
	k1 = l1 + l2*cos2
	k2 = l2*math.sin(q2)
	q1 = math.atan2(ty, tx) - math.atan2(k2, k1)
    
    # Sacamos q3
	r21 = 0 # sin(0)
	r11 = 1 # cos(0)
	q3 = math.atan2(r21, r11) - q1 - q2
	
    # La distancia prismática
	d4 = tz
	
	return [q1, q2, q3, d4]

def move_joint(robotId, joint, target, speed):
    for _ in range(3000):
        pos, vel = p.getJointState(robotId, joint)[0:2]
        if abs(pos - target) < 0.01 and abs(vel) < 0.05:
            break

        p.setJointMotorControl2(robotId, joint, p.POSITION_CONTROL,
            targetPosition=target, force=50, maxVelocity=speed)
        p.stepSimulation()

def pinza(robotId, target, speed=0.1, ignore_contact=False):
    for _ in range(1000):
        left  = p.getContactPoints(bodyA=robotId, linkIndexA=PINZA_LEFT)
        right = p.getContactPoints(bodyA=robotId, linkIndexA=PINZA_RIGHT)
        if not ignore_contact and left and right:
            #print("Contacto detectado, deteniendo pinza")
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

def pick_n_place():
	global state

	q1, q2, q3, d4 = calculate_ik(CUBE)
	move_arm(robotId, [q1, q2])
	time.sleep(0.3)
	twist_pinza(robotId, q3, speed=0.3)
	move_joint(robotId, LINK_PRIS, d4, speed=0.4)
	time.sleep(0.3)
	pinza(robotId, CLOSE_POSE, speed=0.7)
	time.sleep(0.3)
	
	q1, q2, q3, d4 = calculate_ik(AUX_CUBE)
	move_joint(robotId, LINK_PRIS, d4, speed=0.4)
	time.sleep(0.3)
	
	q1, q2, q3, d4 = calculate_ik(CUBE_REST_PLACES[0])
	move_arm(robotId, [q1, q2])
	twist_pinza(robotId, q3, speed=0.3)
	time.sleep(0.3)
	move_joint(robotId, LINK_PRIS, d4, speed=0.4)
	pinza(robotId, OPEN_POSE, speed=0.7, ignore_contact=True)
	time.sleep(0.3)
	
	move_joint(robotId, LINK_PRIS, 0, speed=0.4)
	time.sleep(0.3)
	
	q1, q2, q3, d4 = calculate_ik(HOME)
	move_joint(robotId, LINK_PRIS, d4, speed=0.4)
	move_arm(robotId, [q1, q2])
	twist_pinza(robotId, q3, speed=0.3)
	time.sleep(0.3)
	
	state = "DONE"

physicsClient = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.8)

p.loadURDF("plane_transparent.urdf")
robotId = p.loadURDF("../rover/urdf/rover.urdf", [0,0,1], p.getQuaternionFromEuler([0,0,-3.14]))
cubeId  = p.loadURDF("../cubo/urdf/cubo.urdf",  [0,4,0], p.getQuaternionFromEuler([0,0,-3.14]))

with open('Fase3_rebeca_castilla.csv', 'w', newline='') as csvfile:
    writer = csv.writer(csvfile)
    writer.writerow(['Tiempo', 'NumeroJoints', 'G_parcial'])

state   = "DRIVE"

dt = 0.01
t = 0.0
last_log = time.time()
try:
	while True:
		p.stepSimulation()
		time.sleep(0.005) # specification

		if state == "DRIVE":
			move_to_cube()
		elif state == "BRAKE":
			stop_rover()
		elif state == "SCARA_SEQ":
			t += 0.005
			pick_n_place()
			
		now = time.time()
		if (now - last_log >= dt) and state == "SCARA_SEQ":
			F1 = p.getJointState(robotId, SCARA_ARM_JOINTS[0])[3]
			F2 = p.getJointState(robotId, SCARA_ARM_JOINTS[1])[3]
			F3 = p.getJointState(robotId, SCARA_ROT_PRIS)[3]
			gParcial = F1 + F2 + F3
			
			with open('Fase3_rebeca_castilla.csv', 'a', newline='') as csvfile:
				writer = csv.writer(csvfile)
				writer.writerow([t, NUMERO_JOINTS, gParcial])
			last_log = now

except KeyboardInterrupt:
	pass