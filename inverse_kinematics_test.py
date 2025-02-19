import pybullet as p
import pybullet_data
import time
physicsClient = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
pandaUid = p.loadURDF("franka_panda/panda.urdf", useFixedBase=True)
p.setGravity(0, 0, -9.81)
p.setTimeStep(1.0 / 240.0)
num_joints = p.getNumJoints(pandaUid)
print(f"Number of joints: {num_joints}")
end_effector_link_index = 11
target_position = [0.4, 0.5, 0.6]
target_orientation = p.getQuaternionFromEuler([0, 0, 0])
joint_angles = p.calculateInverseKinematics(
    pandaUid,
    end_effector_link_index,
    target_position,
    targetOrientation=target_orientation,
)
print("joint_angles",joint_angles)
for joint_index in range(len(joint_angles)):
    p.setJointMotorControl2(
        bodyUniqueId=pandaUid,
        jointIndex=joint_index,
        controlMode=p.POSITION_CONTROL,
        targetPosition=joint_angles[joint_index],
    )
for _ in range(1000):
    p.stepSimulation()
    time.sleep(1.0 / 240.0)
p.disconnect()
