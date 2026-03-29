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

#startPosCube = [0, 4, 1]
startPosCube = [0, -1, 4]
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

try:
    while True:
        p.stepSimulation()
        time.sleep(1./240.)

        speed = 12
        torque = 20
        
        p.setJointMotorControlArray(robotId,
                                    myrover_wheel_joints,
                                    p.VELOCITY_CONTROL,
                                    targetVelocities=[speed,speed,speed,speed],
                                    forces=[torque,torque,torque,torque])
        
        
        """
        p.setJointMotorControlArray(robotId,
                                    myrover_arm_joint,
                                    p.VELOCITY_CONTROL,
                                    targetVelocities=[speed],
                                    forces=[torque])
        """

except KeyboardInterrupt:
    pass

p.disconnect()

# To run it: (bullet_env) rumbahuh@stokibot:~/Desktop/URJC/RoboticSoftware_Tercero/MODELADO/URJC-robot-modeling-and-simulation-2025-2026/practica2$ python test.py --urdf ../BLENDER/class_chassis/test/urdf/test.urdf