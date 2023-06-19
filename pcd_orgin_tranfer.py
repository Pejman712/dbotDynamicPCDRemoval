import numpy as np
import open3d as o3d
from pytransform3d import rotations, transformations
from tf.transformations import euler_from_quaternion
from scipy.spatial.transform import Rotation


def transformation_matrix(p1, euler_angles1, p2, euler_angles2):
    """
    """
    R1 = Rotation.from_euler('xyz', euler_angles1)
    R2 = Rotation.from_euler('xyz', euler_angles2)
    
    T = np.eye(4)  # Identity matrix
    
    T[:3, :3] = R2.as_matrix() @ R1.as_matrix().T
    T[:3, 3] = p2 - R2.as_matrix() @ R1.as_matrix().T @ p1

    return T
    
#Target Orinetaiton 
orientation_list = [-0.02807, -0.027795, 0.66422, 0.7465]    

#Target Cordinate
x,y,z = 9.628, 10.303, 0    


pcd = o3d.io.read_point_cloud("filename.pcd")
center= pcd.get_center()
print(center)
# Example usage
p1 = np.array([0,0,0])
euler_angles1 = np.array([0, 0, 0])
p2 = np.array([0,0,0])

roll, pitch, yaw = euler_from_quaternion(orientation_list)
euler_angles2 = np.array([-roll, -pitch, -yaw])
transform_matrix = transformation_matrix(p1, euler_angles1, p2, euler_angles2)
pcd.transform(transform_matrix)

desired_cordinate = [-x, -y, -z]
pcd.translate(desired_cordinate)
# Save the transformed point cloud to a new PCD file
o3d.io.write_point_cloud("filenamewithnewcordinates.pcd", pcd)


