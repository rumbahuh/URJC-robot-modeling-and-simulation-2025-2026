import matplotlib.pyplot as plt
from rosbags.rosbag2 import Reader
from rosbags.typesys import Stores, get_typestore

typestore = get_typestore(Stores.ROS2_HUMBLE)

bag_path = 'rosbag_kachau'

# Datos a recoger
wheel_times = []
wheel_positions = {}  # nombre -> lista de posiciones
wheel_efforts = {}    # nombre -> lista de esfuerzos

imu_times = []
accel_x = []
accel_y = []
accel_z = []

# Tengo que revisar los topics.. bruh
with Reader(bag_path) as reader:
    connections = {c.topic: c for c in reader.connections}

    for connection, timestamp, rawdata in reader.messages():
        t = timestamp * 1e-9  # nanosegundos a segundos

        # Joint states -> ruedas y gasto
        if connection.topic == '/joint_states':
            msg = typestore.deserialize_cdr(rawdata, connection.msgtype)
            wheel_times.append(t)
            for i, name in enumerate(msg.name):
                if name not in wheel_positions:
                    wheel_positions[name] = []
                    wheel_efforts[name] = []
                wheel_positions[name].append(msg.position[i])
                wheel_efforts[name].append(msg.effort[i])

        # IMU -> aceleración
        if connection.topic == '/imu':
            msg = typestore.deserialize_cdr(rawdata, connection.msgtype)
            imu_times.append(t)
            accel_x.append(msg.linear_acceleration.x)
            accel_y.append(msg.linear_acceleration.y)
            accel_z.append(msg.linear_acceleration.z)

# Normalizamos los tiempos
if wheel_times:
    t0 = wheel_times[0]
    wheel_times = [t - t0 for t in wheel_times]

if imu_times:
    t0 = imu_times[0]
    imu_times = [t - t0 for t in imu_times]

# --- Gráfica 1: Posición de ruedas vs tiempo
plt.figure(figsize=(12, 6))
for name, positions in wheel_positions.items():
    if 'front' in name.lower() or 'back' in name.lower() or 'wheel' in name.lower():
        plt.plot(wheel_times, positions, linewidth=2, label=name)
plt.title('Posición de las ruedas vs Tiempo')
plt.xlabel('Tiempo (s)')
plt.ylabel('Posición (rad)')
plt.legend()
plt.grid(True, alpha=0.3)
plt.tight_layout()
plt.savefig('ruedas_posicion.png')
plt.show()

# --- Gráfica 2: Aceleración vs tiempo
plt.figure(figsize=(12, 6))
plt.plot(imu_times, accel_x, linewidth=2, label='Aceleración X', color='red')
plt.plot(imu_times, accel_y, linewidth=2, label='Aceleración Y', color='green')
plt.plot(imu_times, accel_z, linewidth=2, label='Aceleración Z', color='blue')
plt.title('Aceleración vs Tiempo')
plt.xlabel('Tiempo (s)')
plt.ylabel('Aceleración (m/s²)')
plt.legend()
plt.grid(True, alpha=0.3)
plt.tight_layout()
plt.savefig('aceleracion.png')
plt.show()

# --- Gráfica 3: Gasto vs tiempo
plt.figure(figsize=(12, 6))
for name, efforts in wheel_efforts.items():
    if 'front' in name.lower() or 'back' in name.lower() or 'wheel' in name.lower():
        plt.plot(wheel_times, efforts, linewidth=2, label=name)
plt.title('Gasto (Esfuerzo) vs Tiempo')
plt.xlabel('Tiempo (s)')
plt.ylabel('Esfuerzo (N·m)')
plt.legend()
plt.grid(True, alpha=0.3)
plt.tight_layout()
plt.savefig('gasto.png')
plt.show()
