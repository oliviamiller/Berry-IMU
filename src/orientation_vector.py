import ctypes

from ctypes import Structure, POINTER, c_double
from viam.proto.common import Orientation
from viam.logging import getLogger

lib = ctypes.cdll.LoadLibrary("./libviam_rust_utils.so")

class OrientationVector(Structure): 
    pass

orientation_vector_array = c_double * 4

class Quaternion(Structure):
    pass


lib.free_orientation_vector_memory.argtypes = (POINTER(OrientationVector),)
lib.orientation_vector_from_quaternion.argtypes = (POINTER(Quaternion),)
lib.orientation_vector_from_quaternion.restype = POINTER(OrientationVector)
lib.orientation_vector_get_components.argtypes = (POINTER(OrientationVector),)
lib.orientation_vector_get_components.restype = POINTER(orientation_vector_array)
lib.free_quaternion_memory.argtypes = (POINTER(Quaternion),)
lib.quaternion_from_euler_angles.argtypes = (c_double, c_double, c_double)
lib.quaternion_from_euler_angles.restype = POINTER(Quaternion)



def euler_angles_to_orientation_vector(roll, pitch , yaw):
    quaternion = lib.quaternion_from_euler_angles(roll, pitch, yaw)
    orientation_vector = lib.orientation_vector_from_quaternion(quaternion)
    o_x, o_y, o_z, theta = lib.orientation_vector_get_components(orientation_vector).contents
    lib.free_quaternion_memory(quaternion)
    lib.free_orientation_vector_memory(orientation_vector)
    return Orientation(o_x = o_x, o_y = o_y, o_z=o_z, theta=theta) 






    





