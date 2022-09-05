############################################################
##          Written By: Mohammad Hossein Amini            ##
##                 Date:  Mon, 09/05/2022                 ##
############################################################


##  Description: Simple simulation of some robots moving with obstacles


import pybullet as p
import time
import pybullet_data
import numpy as np

def move(q, angle=0., v=5):
    """Moves the q robot in a given angle with a given velocity

    Args:
        q (int): Robot integer identifier
        angle (float, optional): Direction angle in radian. Defaults to 0..
        v (int, optional): Velocity. Defaults to 5.
    """
    p.setJointMotorControl2(
        q, 2, p.VELOCITY_CONTROL, targetVelocity=v * (1 + np.sin(angle))
    )
    p.setJointMotorControl2(
        q, 3, p.VELOCITY_CONTROL, targetVelocity=v * (1 - np.sin(angle))
    )
    p.setJointMotorControl2(
        q, 4, p.VELOCITY_CONTROL, targetVelocity=v * (1 + np.sin(angle))
    )
    p.setJointMotorControl2(
        q, 5, p.VELOCITY_CONTROL, targetVelocity=v * (1 - np.sin(angle))
    )

N_bot = 3  ##  Number of huskies
time.sleep(2)
##  Simulator
physicsClient = p.connect(p.GUI)  # or p.DIRECT for non-graphical version
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -10)
planeId = p.loadURDF("plane.urdf")

husky = [p.loadURDF(
    "husky/husky.urdf", [0, (i - N_bot/2) * 1.5, 0], p.getQuaternionFromEuler([0,0,0]), useFixedBase=False
) for i in range(N_bot)
]

obstacle_pos = [[1, 0, 0], [-2, -1, 0]]
traybox = [p.loadURDF(
    "tray/traybox.urdf", obstacle_pos[i], p.getQuaternionFromEuler([0,0,0]), useFixedBase=False
) for i in range(len(obstacle_pos))
]

nJ = p.getNumJoints(husky[0])
print("No of joints: ", p.getNumJoints(husky[0]))
for i in range(nJ):
    print("Joint info: ", p.getJointInfo(husky[0], i))
print("No of constraints: ", p.getNumConstraints())

for i in range(5000000):
    if i > 100:
        [move(husky[i], np.pi / (i+1) * np.cos(np.pi * (i+1)), np.random.randint(4,7)) for i in range(N_bot)]
    p.stepSimulation()
    time.sleep(1.0 / 240.0)

p.disconnect()
