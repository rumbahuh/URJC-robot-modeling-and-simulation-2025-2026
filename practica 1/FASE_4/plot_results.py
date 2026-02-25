import pandas
import matplotlib.pyplot as plt
import numpy

TARGET_VELOCITY = 2.0

# Cargar CSVs
husky_3_3 = pandas.read_csv("practica 1/FASE_3/3.3_inertial/Fase3.csv")
husky_4   = pandas.read_csv("practica 1/FASE_4/Fase4.csv")

posicion_3, velocidad_3 = husky_3_3["y"], husky_3_3["vy"]
posicion_4, velocidad_4 = husky_4["y"],   husky_4["vy"]

# Metricas
mae_3  = numpy.mean(numpy.abs(TARGET_VELOCITY - velocidad_3))
mae_4  = numpy.mean(numpy.abs(TARGET_VELOCITY - velocidad_4))
rmse_3 = numpy.sqrt(numpy.mean((TARGET_VELOCITY - velocidad_3) ** 2))
rmse_4 = numpy.sqrt(numpy.mean((TARGET_VELOCITY - velocidad_4) ** 2))
mse_3  = numpy.mean((TARGET_VELOCITY - velocidad_3) ** 2)
mse_4  = numpy.mean((TARGET_VELOCITY - velocidad_4) ** 2)

plt.figure(figsize=(12, 6))
plt.plot(posicion_3, velocidad_3, color='green',  linewidth=2, marker='o', markersize=2, markevery=50,
         label=f'3.3 - Fric+Inercia  (MAE: {mae_3:.3f} | RMSE: {rmse_3:.3f} | MSE: {mse_3:.3f})')
plt.plot(posicion_4, velocidad_4, color='blue', linewidth=2, marker='o', markersize=2, markevery=50,
         label=f'Fase 4 - PID        (MAE: {mae_4:.3f} | RMSE: {rmse_4:.3f} | MSE: {mse_4:.3f})')
plt.axhline(y=TARGET_VELOCITY, color='red', linestyle='--', linewidth=1, alpha=0.5, label='Objetivo 2.0 m/s')
plt.grid(True, alpha=0.3)
plt.title("Fase 4 - Controlador PID vs Escenario 3.3")
plt.xlabel("Robot position (m)")
plt.ylabel("Robot Linear Speed (m/s)")
plt.legend(fontsize=8)
plt.tight_layout()
plt.savefig("Fase4.pdf")
plt.show()