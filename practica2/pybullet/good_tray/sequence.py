import pybullet as p
import time
from practica2.pybullet.good_tray.pick_n_place import move_prismatic, pinza, move_arm, twist_pinza

CLOSE_POSE = [0.48, 0.48]
OPEN_POSE = [0.0, 0.0]

ARM_TO_CUBE = 2
ARM_HOME = 0
CUBE_TO_CHASSIS = 1.4

CUBE_REST_PLACES = [[-0.5, 0.0], [-1, 0.0], [-1.4, 0.0]]

def run_sequence(robotId, cubeId, index):
    move_prismatic(robotId, ARM_TO_CUBE, speed=0.4)
    pinza(robotId, CLOSE_POSE, speed=0.7)
    move_prismatic(robotId, ARM_HOME, speed=0.4, ignore_body=cubeId)
    move_arm(robotId, CUBE_REST_PLACES[index])
    twist_pinza(robotId, 0.7, speed=0.3) # Usar cin inversa para sacar los grados concretos
    move_prismatic(robotId, CUBE_TO_CHASSIS, speed=0.4, ignore_body=cubeId)
    pinza(robotId, OPEN_POSE, speed=0.7, ignore_contact=True)
    move_prismatic(robotId, ARM_HOME, speed=0.4, ignore_body=cubeId)