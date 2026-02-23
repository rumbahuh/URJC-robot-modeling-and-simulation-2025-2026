import pandas
import matplotlib.pyplot as plt
import numpy

TARGET_VELOCITY = 2.0

# Load CSV files
husky_3_1 = pandas.read_csv("practica 1/FASE_3/3.1_vel&force/Fase3_1_data.csv")
husky_3_2 = pandas.read_csv("practica 1/FASE_3/3.2_dynamics/Fase3_2_data.csv")

posicion_1, velocidad_1 = husky_3_1["y"], husky_3_1["vy"]
posicion_2, velocidad_2 = husky_3_2["y"], husky_3_2["vy"]

plt.figure(figsize=(10,6))
plt.plot(posicion_1, velocidad_1, color='blue', linewidth=2, marker='o', markersize=2, markevery=50, label='3.1 - Vel & Force')
plt.plot(posicion_2, velocidad_2, color='red', linewidth=2, marker='o', markersize=2, markevery=50, label='3.2 - Fricción')

plt.grid(True, alpha=0.3)
plt.title("Fase 3 - v&force / fric")
plt.xlabel("Robot position (m)")
plt.ylabel("Robot Linear Speed (m/s)")
plt.legend(fontsize=9)
plt.tight_layout()
plt.legend()
plt.show()