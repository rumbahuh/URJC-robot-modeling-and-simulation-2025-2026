import pybullet as p
import pybullet_data
import argparse
import time

# Constantes prismático
PRIS_UP   = 0.6    # arriba
PRIS_DOWN = -0.7   # bajar hasta 0.7 m aproximadamente
MAX_FORCE = 100

def move_prismatic(target, speed=0.4, ee_link=10, ignore_body=None):
    for _ in range(1000):
        contacts = p.getContactPoints(bodyA=robotId, linkIndexA=ee_link)
        
        # Filtrar contactos: ignorar el cuerpo que queremos transportar
        real_contacts = []
        for c in contacts:
            if ignore_body is not None and c[2] == ignore_body:
                continue  # skip contact with the cube
            real_contacts.append(c)
        
        if real_contacts:
            print("⚠️ Contacto detectado, deteniendo")
            break
        
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

def pinza(target, speed=0.1, ee_link=[11, 12]):
    for _ in range(1000):
        contacts_11 = p.getContactPoints(bodyA=robotId, linkIndexA=11)
        contacts_12 = p.getContactPoints(bodyA=robotId, linkIndexA=12)
        
        if contacts_11 and contacts_12:
            print("⚠️ Contacto detectado, deteniendo pinza")
            break
        
        p.setJointMotorControl2(
            robotId,
            ee_link[0],
            p.POSITION_CONTROL,
            targetPosition=target[0],
            force=MAX_FORCE,
            maxVelocity=speed
        )
        p.setJointMotorControl2(
            robotId,
            ee_link[1],
            p.POSITION_CONTROL,
            targetPosition=target[1],
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
print("\n── Joints del robot ──────────────────────────────")
for i in range(p.getNumJoints(robotId)):
    info = p.getJointInfo(robotId, i)
    print(f"  [{i}] {info[1].decode():40s}  tipo={info[2]}")
print("─────────────────────────────────────────────────\n")

── Joints del robot ──────────────────────────────
  [0] axisB_link_joint                          tipo=4
  [1] armBackL_link_joint                       tipo=4
  [2] backL_link_joint                          tipo=0
  [3] armBackR_link_joint                       tipo=4
  [4] backR_link_joint                          tipo=0
  [5] base_link_joint                           tipo=4
  [6] link1_link_joint                          tipo=4
  [7] link2_link_joint                          tipo=0
  [8] link3_link_joint                          tipo=0
  [9] rot_link4_link_joint                      tipo=0
  [10] pris_link4_link_joint                     tipo=1
  [11] pris_link4.001_link_joint                 tipo=1
  [12] pris_link4.002_link_joint                 tipo=1
  [13] axisF_link_joint                          tipo=4
  [14] armFrontL_link_joint                      tipo=4
  [15] frontL_link_joint                         tipo=0
  [16] armFrontR_link_joint                      tipo=4
  [17] frontR_link_joint                         tipo=0
─────────────────────────────────────────────────
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
                move_prismatic(2, speed=0.5)                        # bajar
                pinza([0.48, 0.48], speed=0.1)                      # cerrar pinza
                move_prismatic(0, speed=0.5, ignore_body=cubeId)    # subir ignorando cubo
                # --- llegar al destino ---
                pinza([0.0, 0.0], speed=0.1)                        # abrir pinza para soltar
                prism_moved = True
        
except KeyboardInterrupt:
    pass

p.disconnect()

# To run it: (bullet_env) rumbahuh@stokibot:~/Desktop/URJC/RoboticSoftware_Tercero/MODELADO/URJC-robot-modeling-and-simulation-2025-2026/practica2$ python test.py --urdf ../BLENDER/class_chassis/test/urdf/test.urdf