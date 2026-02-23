import pybullet as p
import pybullet_data
import time
import csv

Y = 1
RADIUS_WHEELS = 0.165
TARGET_VELOCITY = 1.78      # Consigna interna (compensa el bias de PyBullet)
TARGET_VELOCITY_REAL = 2.0  # Para calcular error en CSV
KP = 3
KI = 0.3
KD = 0.08
MAX_TORQUE = 500.0
MAX_CORRECTION_UP = 8.0
MAX_CORRECTION_DOWN = 6.0
INTEGRAL_LIMIT = 5.0
ALPHA = 0.15

physicsClient = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.8)
planeId = p.loadURDF("plane.urdf")

euler_angles = [0, 0, 1.57]
startOrientation = p.getQuaternionFromEuler(euler_angles)
huskyId = p.loadURDF("husky/husky.urdf", [0, 0, 1], startOrientation)

euler_angles = [0, 0, 0]
startOrientation = p.getQuaternionFromEuler(euler_angles)
rampId = p.loadURDF("ramp.urdf", [0, 10, 1], startOrientation)
finishLineId = p.loadURDF("finish_line.urdf", [0, 20, 1], startOrientation)
barId = p.loadURDF("bar.urdf", [-1.5, 17, 0.01], startOrientation)

husky_wheel_joints = [2, 3, 4, 5]
for wheel in husky_wheel_joints:
    p.changeDynamics(
        huskyId, wheel,
        lateralFriction=0.93,
        spinningFriction=0.005,
        rollingFriction=0.003
    )

log_y_next = 0.01
integral = 0.0
prev_error = 0.0
prev_time = time.time()
vy_filtered = 0.0

with open('Fase4_data.csv', 'w', newline='') as csvfile:
    writer = csv.writer(csvfile)
    writer.writerow(['tiempo', 'y', 'vy', 'velocidad_ruedas', 'fuerza_ruedas', 'error'])

try:
    p.setRealTimeSimulation(1)
    while True:
        posicion_robot, _ = p.getBasePositionAndOrientation(huskyId)
        linear_velocity, _ = p.getBaseVelocity(huskyId)
        vy = linear_velocity[Y]
        y = posicion_robot[Y]

        vy_filtered = ALPHA * vy + (1 - ALPHA) * vy_filtered

        if y < 20:
            error = TARGET_VELOCITY - vy_filtered

            now = time.time()
            dt = now - prev_time
            if dt <= 0 or dt > 0.5:
                dt = 0.01
            prev_time = now

            P_out = KP * error
            integral += error * dt
            integral = max(-INTEGRAL_LIMIT, min(INTEGRAL_LIMIT, integral))
            I_out = KI * integral
            D_out = KD * (error - prev_error) / dt
            prev_error = error

            pid_correction = P_out + I_out + D_out
            pid_correction = max(-MAX_CORRECTION_DOWN, min(MAX_CORRECTION_UP, pid_correction))
            velocidad_ruedas = (TARGET_VELOCITY + pid_correction) / RADIUS_WHEELS
            fuerzas_ruedas = MAX_TORQUE
        else:
            velocidad_ruedas = 0.0
            fuerzas_ruedas = 50.0

        p.setJointMotorControlArray(
            huskyId,
            husky_wheel_joints,
            p.VELOCITY_CONTROL,
            targetVelocities=[velocidad_ruedas] * 4,
            forces=[fuerzas_ruedas] * 4
        )

        if y >= log_y_next:
            with open('Fase4_data.csv', 'a', newline='') as csvfile:
                writer = csv.writer(csvfile)
                # Error siempre respecto a 2.0 real
                writer.writerow([time.time(), y, vy, velocidad_ruedas, fuerzas_ruedas, TARGET_VELOCITY_REAL - vy])
            if log_y_next >= 20:
                print("Llegada a meta con exito.")
                break
            log_y_next += 0.01

except KeyboardInterrupt:
    pass

p.disconnect()