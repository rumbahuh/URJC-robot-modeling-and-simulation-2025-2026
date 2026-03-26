import pybullet as p
import pybullet_data
import argparse
import time

# Constantes prismático
PRIS_UP   = 0.6    # arriba
PRIS_DOWN = -0.7   # bajar hasta 0.7 m aproximadamente
MAX_FORCE = 100

def move_prismatic(target, speed=0.4, ee_link=10):
    for _ in range(1000):
        # Detectar contacto
        contacts = p.getContactPoints(bodyA=robotId, linkIndexA=ee_link)
        if contacts:
            print("⚠️ Contacto detectado, deteniendo descenso")
            break
        
        # Mover prismático
        p.setJointMotorControl2(
            robotId,
            ee_link,
            p.POSITION_CONTROL,
            targetPosition=target,
            force=MAX_FORCE,
            maxVelocity=speed
        )
        p.stepSimulation()
        time.sleep(1./240.)

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

#startPosCube = [0, 4, 1]
#startPosCube = [0, -1, 4]
startPosCube = [0.1, 2.1, 1]
startOrientationCube = p.getQuaternionFromEuler([0, 0, -3.15])
cubeId = p.loadURDF("/home/rumbahuh/Desktop/URJC/RoboticSoftware_Tercero/MODELADO/URJC-robot-modeling-and-simulation-2025-2026/practica2/pybullet/cubo/urdf/cubo.urdf", startPosCube, startOrientationCube)

"""
for i in range(p.getNumJoints(robotId)):
    print(p.getJointInfo(robotId, i)[0:2]) # Prints (ID, Name)

(0, b'axisB_link_joint')
(1, b'armBackL_link_joint')
(2, b'backL_link_joint')
(3, b'armBackR_link_joint')
(4, b'backR_link_joint')
(5, b'base_link_joint')
(6, b'link1_link_joint')
(7, b'link2_link_joint')
(8, b'link3_link_joint')
(9, b'rot_link4_link_joint')
(10, b'pris_link4_link_joint')
(11, b'axisF_link_joint')
(12, b'armFrontL_link_joint')
(13, b'frontL_link_joint')
(14, b'armFrontR_link_joint')
(15, b'frontR_link_joint')
"""

myrover_wheel_joints = [2, 4, 13, 15]
myrover_arm_joint = [10]

stable_count = 0        # counts consecutive steps where joint is at target
stable_threshold = 50    # number of consecutive steps needed
prism_moved = False

try:
    while True:
        p.stepSimulation()
        time.sleep(1./240.)

        # Move link2 and link3
        p.setJointMotorControlArray(
            robotId,
            [7, 8],
            p.POSITION_CONTROL,
            targetPositions=[1.57, 1.57],
            forces=[50, 50]
        )

        if not prism_moved:
            # Check BOTH joints, not just link2
            current_q2 = p.getJointState(robotId, 7)[0]
            current_q3 = p.getJointState(robotId, 8)[0]

            # Also check velocity — position alone isn't enough
            vel_q2 = p.getJointState(robotId, 7)[1]
            vel_q3 = p.getJointState(robotId, 8)[1]

            at_position = (abs(current_q2 - 1.57) < 0.01 and
                           abs(current_q3 - 1.57) < 0.01)
            is_still     = (abs(vel_q2) < 0.01 and
                            abs(vel_q3) < 0.01)

            if at_position and is_still:
                stable_count += 1
            else:
                stable_count = 0  # reset if drifting

            if stable_count >= stable_threshold:
                move_prismatic(1.8, speed=0.5)
                prism_moved = True
        
except KeyboardInterrupt:
    pass

p.disconnect()

# To run it: (bullet_env) rumbahuh@stokibot:~/Desktop/URJC/RoboticSoftware_Tercero/MODELADO/URJC-robot-modeling-and-simulation-2025-2026/practica2$ python test.py --urdf ../BLENDER/class_chassis/test/urdf/test.urdf