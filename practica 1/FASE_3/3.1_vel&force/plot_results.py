import pandas
import matplotlib.pyplot as plt
import numpy

TARGET_VELOCITY = 2.0

# Load CSV files
husky = pandas.read_csv("practica 1/FASE_3/3.1_vel&force/Fase3_1_data.csv")

posicion = husky["y"]
velocidad = husky["vy"]
error = husky["error"]

# Standard deviations
posicion_std = numpy.std(posicion, axis=0)
velocidad_std = numpy.std(velocidad, axis=0)
error_std = numpy.std(velocidad, axis=0)

plt.figure(figsize=(10,6))
plt.plot(posicion, velocidad, color='blue', linewidth=2, 
                      marker='o', markersize=2, markevery=50,
                      label=f'Velocidad m/s)')

plt.grid(True, alpha=0.3)
plt.title("Fase 3 - v&force")
plt.xlabel("Robot position (m)")
plt.ylabel("Robot Linear Speed (m/s)")
plt.legend(fontsize=9)
plt.tight_layout()
plt.legend()
plt.show()