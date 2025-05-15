from geometry_msgs.msg import Quaternion
import math
import tf.transformations
import tf
import numpy as np

def normalize_angle(ang):
    while ang > math.pi:
        ang -= 2.0 * math.pi
    while ang < -math.pi:
        ang += 2.0 * math.pi
    return ang
    
def rotate_quaternion_around_z(quaternion, degrees):
    radians = math.radians(degrees)
    rotation_quaternion = tf.transformations.quaternion_from_euler(0, 0, radians)
    original_quaternion = [quaternion.x, quaternion.y, quaternion.z, quaternion.w]
    new_quaternion = tf.transformations.quaternion_multiply(rotation_quaternion, original_quaternion)
    return Quaternion(new_quaternion[0], new_quaternion[1], new_quaternion[2], new_quaternion[3])
    
def get_yaw_from_quaternion(quaternion):
    euler = tf.transformations.euler_from_quaternion([quaternion.x, quaternion.y, quaternion.z, quaternion.w])
    yaw = euler[2]
    return yaw

def vectors_are_same_direction(vector_a, vector_b):
    if np.all(vector_a == 0) or np.all(vector_b == 0):
        return False
    norm_a = vector_a / np.linalg.norm(vector_a)
    norm_b = vector_b / np.linalg.norm(vector_b)
    result = np.all(abs(norm_a - norm_b) < 1e-5)
    return result

def cut_vector_by_vector_old(velocity_vector, repulsion_vector, koef):
    repulsion_unit = repulsion_vector / np.linalg.norm(repulsion_vector)
    projection = np.dot(velocity_vector, repulsion_unit) * repulsion_unit
    same_direction = vectors_are_same_direction(repulsion_vector.astype(np.float64), projection.astype(np.float64))
    if not same_direction:
        new_velocity = velocity_vector - projection * koef
    elif same_direction and koef > 1:
        new_velocity = velocity_vector + projection * (koef - 1)
    elif same_direction and koef <= 1:
        new_velocity = velocity_vector
    return new_velocity.astype(np.float64)

def cut_vector_by_vector(velocity_vector, repulsion_vector, koef):
    max_speed = 1.0
    repulsion_unit = repulsion_vector / np.linalg.norm(repulsion_vector)
    projection = np.dot(velocity_vector, repulsion_unit) * repulsion_unit
    current_projection_speed = np.linalg.norm(projection)
    if current_projection_speed == 0 : current_projection_speed = 1e-5
    same_direction = vectors_are_same_direction(repulsion_vector.astype(np.float64), projection.astype(np.float64))
    if not same_direction and koef <= 1:
        if current_projection_speed > max_speed * (1 - koef):
            new_velocity = velocity_vector - projection + projection / current_projection_speed * (max_speed * (1-koef))
        else: new_velocity = velocity_vector
    elif not same_direction and koef > 1:
        new_velocity = velocity_vector - projection + projection / current_projection_speed * (max_speed * (1-koef))
    elif same_direction and koef > 1:
        new_velocity = velocity_vector + projection * (koef - 1)
    elif same_direction and koef <= 1:
        new_velocity = velocity_vector
    return new_velocity.astype(np.float64)