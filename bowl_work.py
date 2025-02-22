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
end_effector_link_index = 11
gripper_left = 9
gripper_right = 10

# Define cube properties
cube_start_pos = [0.4, 0.0, 0.05]  # On the ground
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

bowl_start_pos = [0.2, -0.3, 0.05]  # Position on the floor
bowl_orientation = p.getQuaternionFromEuler([0, 0, 1.57])  # Correct rotation to open upward

bowl_collision_shape = p.createCollisionShape(p.GEOM_CYLINDER, radius=0.15, height=0.05)
bowl_visual_shape = p.createVisualShape(p.GEOM_CYLINDER, radius=0.15, length=0.05, rgbaColor=[0.8, 0.5, 0.2, 1])

bowl_id = p.createMultiBody(
    baseMass=0,  # Static object
    baseCollisionShapeIndex=bowl_collision_shape,
    baseVisualShapeIndex=bowl_visual_shape,
    basePosition=bowl_start_pos,
    baseOrientation=bowl_orientation
)


# Define target positions (above cube, pick-up, above bowl, place)
target_positions = [
    [0.4, 0.0, 0.2],  # Move above the cube
    [0.4, 0.0, 0.05],  # Lower to the cube
    [0.2, -0.3, 0.2],  # Move above the bowl
    [0.2, -0.3, 0.07],  # Lower into the bowl
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

# Move the cube above the bowl
print("Moving cube above the bowl...")
move_arm_to_target(target_positions[2])

# Lower the cube into the bowl
print("Lowering cube into the bowl...")
move_arm_to_target(target_positions[3])

# Ensure the constraint is removed first before opening the gripper
print("Releasing the cube...")
p.removeConstraint(constraint_id)  # REMOVE THE CONSTRAINT FIRST

# Re-enable gravity on the cube so it falls into the bowl
p.changeDynamics(cube_uid, -1, mass=1)

# Open the gripper fully after removing constraint
open_gripper()

# Keep simulation running for visualization
time.sleep(2)

# Disconnect from simulation
p.disconnect()
