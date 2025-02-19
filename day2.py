import pybullet as p
import pybullet_data
import time
physicsClient = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.81)
planeId = p.loadURDF("plane.urdf")
robot_urdf_path = "franka_panda/panda.urdf"
robot_start_pos = [0, 0, 0]
robot_start_orientation = p.getQuaternionFromEuler([0, 0, 0])
robot_id = p.loadURDF(robot_urdf_path, robot_start_pos, robot_start_orientation, useFixedBase=True)
num_joints = p.getNumJoints(robot_id)
print(f"Number of joints: {num_joints}")
for joint_index in range(num_joints):
    joint_info = p.getJointInfo(robot_id, joint_index)
    print(f"Joint {joint_index}: {joint_info}")
target_positions = [0, -0.5, 0, -2, 0, 1.5, 0]
for i in range(1000):
    for joint_index in range(len(target_positions)):
        p.setJointMotorControl2(
            bodyUniqueId=robot_id,
            jointIndex=joint_index,
            controlMode=p.POSITION_CONTROL,
            targetPosition=target_positions[joint_index],
        )
    p.stepSimulation()
    time.sleep(1 / 240)
end_effector_state = p.getLinkState(robot_id, 11)
print("End effector position:", end_effector_state[0])
p.disconnect()