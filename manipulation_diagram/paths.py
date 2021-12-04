import os
from pydrake.common import FindResourceOrThrow


# IIWA 14 paths
iiwa14_no_collision_path = FindResourceOrThrow(
    "drake/manipulation/models/iiwa_description/urdf/iiwa14_no_collision.urdf")
iiwa14_poly_collision_path = FindResourceOrThrow(
    "drake/manipulation/models/iiwa_description/urdf/iiwa14_polytope_collision.urdf")
iiwa14_primitive_collision_path = FindResourceOrThrow(
    "drake/manipulation/models/iiwa_description/urdf/iiwa14_primitive_collision.urdf")
iiwa14_sphere_collision_path = FindResourceOrThrow(
    "drake/manipulation/models/iiwa_description/urdf/iiwa14_spheres_dense_collision.urdf")

# IIWA 7 paths
iiwa_7_no_collision_path = FindResourceOrThrow(
    "drake/manipulation/models/iiwa_description/iiwa7/iiwa7_no_collision.sdf")
iiwa_7_box_collision_path = FindResourceOrThrow(
    "drake/manipulation/models/iiwa_description/iiwa7/iiwa7_with_box_collision.sdf")

# Schunk wsg 50 path
# schunk_path = FindResourceOrThrow(
#       "drake/manipulation/models/wsg_50_description/sdf/schunk_wsg_50.sdf")
#schunk_path = os.path.join(os.getcwd(), "models/schunk_gripper/schunk_wsg_50_no_tip.sdf")
schunk_path = FindResourceOrThrow(
    "drake/manipulation/models/wsg_50_description/sdf/schunk_wsg_50_no_tip.sdf")

# Franka paths
franka_arm_path = FindResourceOrThrow(
    "drake/manipulation/models/franka_description/urdf/panda_arm.urdf")
franka_hand_path = FindResourceOrThrow(
    "drake/manipulation/models/franka_description/urdf/hand.urdf")
franka_combined_path = FindResourceOrThrow(
    "drake/manipulation/models/franka_description/urdf/panda_arm_hand.urdf")
