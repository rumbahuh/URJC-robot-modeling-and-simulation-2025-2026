import pandas
import matplotlib
import matplotlib.pyplot
import numpy

# Load CSV files
husky = pandas.read_csv("practica 1/FASE_2/Fase2_data.csv")

posicion = husky["y"]
velocidad = husky["velocidad_ruedas"]

# Standard deviations
posicion_std = numpy.std(posicion, axis=0)
velocidad_std = numpy.std(velocidad, axis=0)

matplotlib.pyplot.figure(figsize=(10,6))
matplotlib.pyplot.ylim(-0.1, 2.2)

matplotlib.pyplot.plot(posicion, velocidad, color='blue', linewidth=2, 
                      marker='o', markersize=2, markevery=50,
                      label=f'Velocidad de ruedas (std={velocidad_std:.4f} m/s)')
                      
matplotlib.pyplot.grid(True, alpha=0.3)
matplotlib.pyplot.title("Fase 2 - v/x")
matplotlib.pyplot.xlabel("Robot position (m)") 
matplotlib.pyplot.ylabel("Robot Linear Speed (m/s)") 
matplotlib.pyplot.legend()
matplotlib.pyplot.show()