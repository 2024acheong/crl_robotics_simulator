import pybullet as p
import pybullet_data
import time

# Connect to PyBullet
physicsClient = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())

# Load the plane
p.loadURDF("plane.urdf")

# Load the Panda robot
pandaUid = p.loadURDF("franka_panda/panda.urdf", useFixedBase=True)
p.setGravity(0, 0, -9.81)
p.setTimeStep(1.0 / 240.0)

# End-effector and gripper info
end_effector_link_index = 11  # Franka's end-effector link
gripper_left = 9
gripper_right = 10

# Define cube properties
cube_start_pos = [0.4, 0.0, 0.0]  # On table surface
cube_start_orientation = p.getQuaternionFromEuler([0, 0, 0])

# Create a small cube
cube_collision_shape = p.createCollisionShape(p.GEOM_BOX, halfExtents=[0.05, 0.05, 0.05])
cube_visual_shape = p.createVisualShape(p.GEOM_BOX, halfExtents=[0.05, 0.05, 0.05], rgbaColor=[1, 0, 0, 1])
cube_uid = p.createMultiBody(
    baseMass=1, 
    baseCollisionShapeIndex=cube_collision_shape,
    baseVisualShapeIndex=cube_visual_shape,
    basePosition=cube_start_pos,
    baseOrientation=cube_start_orientation
)

# Define target positions (above the cube, then to another location)
target_positions = [
    [0.4, 0.0, 0.75],  # Move above the cube
    [0.4, 0.0, 0.0],  # Lower to the cube
    [0.2, -0.3, 0.7],  # Move to a new location
]

target_orientation = p.getQuaternionFromEuler([0, 0, 0])

def move_arm_to_target(target_position, steps=150):
    """ Moves the arm to a given target position """
    joint_angles = p.calculateInverseKinematics(
        pandaUid,
        end_effector_link_index,
        target_position,
        targetOrientation=target_orientation
    )
    
    for step in range(steps):
        for joint_index in range(len(joint_angles)):
            p.setJointMotorControl2(
                bodyUniqueId=pandaUid,
                jointIndex=joint_index,
                controlMode=p.POSITION_CONTROL,
                targetPosition=joint_angles[joint_index],
            )
        p.stepSimulation()
        time.sleep(1.0 / 240.0)

def close_gripper():
    """ Closes the gripper fingers """
    for step in range(50):
        p.setJointMotorControl2(pandaUid, gripper_left, p.POSITION_CONTROL, targetPosition=0)
        p.setJointMotorControl2(pandaUid, gripper_right, p.POSITION_CONTROL, targetPosition=0)
        p.stepSimulation()
        time.sleep(1.0 / 240.0)

def open_gripper():
    """ Opens the gripper fingers """
    for step in range(50):
        p.setJointMotorControl2(pandaUid, gripper_left, p.POSITION_CONTROL, targetPosition=0.04)
        p.setJointMotorControl2(pandaUid, gripper_right, p.POSITION_CONTROL, targetPosition=0.04)
        p.stepSimulation()
        time.sleep(1.0 / 240.0)

# Move above the cube
print("Moving above the cube...")
move_arm_to_target(target_positions[0])

# Move down to the cube
print("Moving to cube...")
move_arm_to_target(target_positions[1])

# Close the gripper to grasp the cube
print("Closing gripper...")
close_gripper()

# Attach the cube to the end-effector using a constraint
constraint_id = p.createConstraint(
    parentBodyUniqueId=pandaUid,
    parentLinkIndex=end_effector_link_index,
    childBodyUniqueId=cube_uid,
    childLinkIndex=-1,
    jointType=p.JOINT_FIXED,
    jointAxis=[0, 0, 0],
    parentFramePosition=[0, 0, 0.05],  # Offset for cube grasp
    childFramePosition=[0, 0, 0]
)

# Move the cube to a new position
print("Moving cube to second position...")
move_arm_to_target(target_positions[2])

# Open the gripper to release the cube
print("Opening gripper...")
open_gripper()

# Remove the constraint so the cube is free
p.removeConstraint(constraint_id)

# Keep simulation running for visualization
time.sleep(2)

# Disconnect from simulation
p.disconnect()
