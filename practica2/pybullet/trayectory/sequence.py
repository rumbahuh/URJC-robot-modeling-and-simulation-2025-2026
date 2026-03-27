import pybullet as p
import time
from pick_n_place import move_prismatic, pinza, move_arm

CLOSE_POSE = [0.48, 0.48]
OPEN_POSE = [0.0, 0.0]

ARM_TO_CUBE = 2
ARM_HOME = 0
CUBE_TO_CHASSIS = 1.4

CUBE_REST_PLACES = [[-0.2, 0.0], [-1, 0.0], [-1.4, 0.0]]

def run_sequence(robotId, cubeId, index):
    print("Step 1: lowering...")
    move_prismatic(robotId, ARM_TO_CUBE, speed=0.5)
    print("Step 2: closing gripper...")
    pinza(robotId, CLOSE_POSE, speed=0.1)
    print("Step 3: lifting...")
    move_prismatic(robotId, ARM_HOME, speed=0.5, ignore_body=cubeId)
    print("Step 4: rotating back...")
    move_arm(robotId, CUBE_REST_PLACES[index], ignore_body=cubeId)
    #print("Step 5: lowering to place...")
    #move_prismatic(robotId, CUBE_TO_CHASSIS, speed=0.5, ignore_body=cubeId)
    print("Step 6: releasing...")
    pinza(robotId, OPEN_POSE, speed=0.1, ignore_contact=True)
    print("Step 7: lifting empty...")
    move_prismatic(robotId, ARM_HOME, speed=0.5)
    print("Done!")