import pybullet as p
import pybullet_data
import time
import csv

Y = 1

physicsClient = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.8)

planeId = p.loadURDF("plane.urdf")

euler_angles = [0, 0, 1.57] # Initiates figure rotated 90 degrees on Z axis
startOrientation = p.getQuaternionFromEuler(euler_angles)
startPosition = [0, 0, 1]

huskyId = p.loadURDF("husky/husky.urdf", startPosition, startOrientation)
posicion_robot = p.getBasePositionAndOrientation(huskyId, physicsClient)

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
log_y_next = 0.01

with open('Fase2_data.csv', 'w', newline='') as csvfile:
    writer = csv.writer(csvfile)
    writer.writerow(['tiempo', 'y', 'velocidad_ruedas', 'fuerza_ruedas'])

try:
	while True:
		p.setRealTimeSimulation(1)

		posicion_robot, orientation_robot = p.getBasePositionAndOrientation(huskyId, physicsClient)
			
		if (posicion_robot[Y] < 20):
			speed = 12 # angular
			torque = 20
		else:
			speed = 0
			torque = 0
		
		p.setJointMotorControlArray(huskyId,
									husky_wheel_joints,
									p.VELOCITY_CONTROL,
									targetVelocities=[speed,speed,speed,speed],
									forces=[torque,torque,torque,torque])
		
		# Specification for analysis
		if (posicion_robot[Y] >= log_y_next):
			tiempo = time.time()
			y = posicion_robot[Y]
			velocidad_ruedas = speed * 0.165
			fuerza_ruedas =  torque / 0.165

			with open('Fase2_data.csv', 'a', newline='') as csvfile:
				writer = csv.writer(csvfile)
				writer.writerow([tiempo, y, velocidad_ruedas, fuerza_ruedas])
			
			if(log_y_next >= 20):
				break
			log_y_next += 0.01

except KeyboardInterrupt:
	pass

p.disconnect()