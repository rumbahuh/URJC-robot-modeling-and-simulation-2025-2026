import pybullet as p
import pybullet_data
import time
import csv

Y = 1
RADIUS_WHEELS = 0.165
TARGET_VELOCITY = 2.0
KP = 1
KI = 0.001
KD = 0.08
MAX_TORQUE = 500.0
INTEGRAL_LIMIT = 5.0

physicsClient = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.8)

planeId = p.loadURDF("plane.urdf")

euler_angles = [0, 0, 1.57] # Initiates figure rotated 90 degrees on Z axis
startOrientation = p.getQuaternionFromEuler(euler_angles)
huskyId = p.loadURDF("husky/husky.urdf", [0, 0, 1], startOrientation)

euler_angles = [0, 0, 0]
startOrientation = p.getQuaternionFromEuler(euler_angles)
startPosition = [0, 10, 1]

rampId = p.loadURDF("ramp.urdf", startPosition, startOrientation)

startOrientation = p.getQuaternionFromEuler(euler_angles)
startPosition = [0, 20, 1]

finishLineId = p.loadURDF("finish_line.urdf", startPosition, startOrientation)

startOrientation = p.getQuaternionFromEuler(euler_angles)
startPosition = [-1.5, 17, 0.1]

barId = p.loadURDF("bar.urdf", startPosition, startOrientation)

husky_wheel_joints = [2, 3, 4, 5]
for wheel in husky_wheel_joints:
    p.changeDynamics(huskyId, wheel,
                     lateralFriction=0.93,
                     spinningFriction=0.005,
                     rollingFriction=0.003)

log_y_next = 0.01
integral = 0.0
prev_error = 0.0
prev_time = time.time()

with open('Fase4.csv', 'w', newline='') as csvfile:
    writer = csv.writer(csvfile)
    writer.writerow(['tiempo', 'y', 'vy', 'velocidad_ruedas', 'fuerza_ruedas', 'error'])

try:
    p.setRealTimeSimulation(1)
    while True:
        posicion_robot, orientation_robot = p.getBasePositionAndOrientation(huskyId)
        linear_velocity, angular_velocity = p.getBaseVelocity(huskyId)
        vy = linear_velocity[Y]
        y = posicion_robot[Y]

        if y < 20:
            # Compute error and dt
            error = TARGET_VELOCITY - vy
            now = time.time()
            dt = now - prev_time
            if dt <= 0 or dt > 0.5:
                dt = 0.01 # I try to make it constant
            prev_time = now

            # Simple PID configuration
            P_out = KP * error

            integral += error * dt
            if integral > INTEGRAL_LIMIT:
                integral = INTEGRAL_LIMIT
            elif integral < -INTEGRAL_LIMIT:
                integral = -INTEGRAL_LIMIT # Bound integral
            I_out = KI * integral

            D_out = KD * (error - prev_error) / dt

            prev_error = error

            pid_correction = P_out + I_out + D_out
            if pid_correction > 8.0:
                pid_correction = 8.0
            elif pid_correction < -6.0:
                pid_correction = -6.0 # Bound correction

            # Wheel speed (angular)
            velocidad_ruedas = (TARGET_VELOCITY + pid_correction) / RADIUS_WHEELS
            fuerzas_ruedas = MAX_TORQUE

        else:
            velocidad_ruedas = 0.0
            fuerzas_ruedas = 50.0

        p.setJointMotorControlArray(
            huskyId, husky_wheel_joints,
            p.VELOCITY_CONTROL,
            targetVelocities=[velocidad_ruedas] * 4,
            forces=[fuerzas_ruedas] * 4
        )

        # 7. Guardar en CSV cada centímetro
        if y >= log_y_next:
            with open('Fase4.csv', 'a', newline='') as csvfile:
                writer = csv.writer(csvfile)
                velocidad_ruedas = velocidad_ruedas * RADIUS_WHEELS # I want to compare lineal with 3.3 later
                writer.writerow([time.time(), y, vy, velocidad_ruedas, fuerzas_ruedas, TARGET_VELOCITY - vy])
            if log_y_next >= 20:
                print("Llegada a meta con exito.")
                break
            log_y_next += 0.01

except KeyboardInterrupt:
    pass

p.disconnect()