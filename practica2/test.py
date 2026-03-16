import pybullet as p
import pybullet_data
import argparse
import time

parser = argparse.ArgumentParser(description="URDF viewer example")
parser.add_argument("--urdf", type=str, required=True, help="Ruta al archivo URDF.")
args = parser.parse_args()
urdf_path = args.urdf

physicsClient = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.8)
planeId = p.loadURDF("plane_transparent.urdf")

startPos = [0, 0, 1]
startOrientation = p.getQuaternionFromEuler([0, 0, -3.15])
robotId = p.loadURDF(urdf_path, startPos, startOrientation)

try:
    while True:
        p.stepSimulation()
        time.sleep(1./240.)
except KeyboardInterrupt:
    pass

p.disconnect()

# To run it: (bullet_env) rumbahuh@stokibot:~/Desktop/URJC/RoboticSoftware_Tercero/MODELADO/URJC-robot-modeling-and-simulation-2025-2026/practica2$ python test.py --urdf ../BLENDER/class_chassis/test/urdf/test.urdf