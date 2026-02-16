import pandas
import matplotlib
import matplotlib.pyplot
import numpy

# Load CSV files
husky_3_1 = pandas.read_csv("practica 1/FASE_3/3.1_vel&force/Fase3_1_data.csv")
husky_3_2 = pandas.read_csv("practica 1/FASE_3/3.2_dynamics/Fase3_2_data.csv")
husky_3_3 = pandas.read_csv("practica 1/FASE_3/3.3_inertial/Fase3_3_data.csv")

posicion_1 = husky_3_1["y"]
velocidad_1 = husky_3_1["vy"]

posicion_2 = husky_3_2["y"]
velocidad_2 = husky_3_2["vy"]

posicion_3 = husky_3_3["y"]
velocidad_3 = husky_3_3["vy"]

matplotlib.pyplot.figure(figsize=(10,6))
matplotlib.pyplot.ylim(-0.1, 2.2)

matplotlib.pyplot.plot(posicion_1, velocidad_1, color='blue', linewidth=2, 
                      marker='o', markersize=2, markevery=50)
matplotlib.pyplot.plot(posicion_2, velocidad_2, color='red', linewidth=2, 
                      marker='o', markersize=2, markevery=50)
matplotlib.pyplot.plot(posicion_3, velocidad_3, color='green', linewidth=2, 
                      marker='o', markersize=2, markevery=50)
                      
matplotlib.pyplot.grid(True, alpha=0.3)
matplotlib.pyplot.title("Fase 3")
matplotlib.pyplot.xlabel("Robot position (m)") 
matplotlib.pyplot.ylabel("Robot Linear Speed (m/s)") 
matplotlib.pyplot.legend()
matplotlib.pyplot.show()