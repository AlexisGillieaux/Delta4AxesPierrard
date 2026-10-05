import numpy as np
from scipy.spatial.transform import Rotation as R

class RigidTransform:
    # helper class to manipulate 3D rigid poses.
    # initialize with the following functions (e.g. RigidTransform.from_homogeneous(mat).P())
    
    @staticmethod
    def from_homogeneous(hom_mat, seq='xyz', degrees=True):
        return RigidTransform(hom_mat=hom_mat, seq=seq, degrees=degrees)

    
    @staticmethod
    def from_pose(pose, seq='xyz', degrees=True):
        """
        Create a RigidTransform from Euler angles representation.
        
        Args:
            pose: Array of shape (6,) or (N,6) containing [x,y,z,rx,ry,rz]
            seq: Sequence of rotation axes, e.g., 'xyz' for roll-pitch-yaw or 'XYZ' for Euler angles
            degrees: Whether rotation angles are in degrees (True) or radians (False)
            
        Returns:
            A new RigidTransform instance
        """
        return RigidTransform(pose=pose, seq=seq, degrees=degrees)
    
    @staticmethod
    def from_pose_quat(pose_quat, seq='xyz', degrees=True):
        return RigidTransform(pose_quat=pose_quat, seq=seq, degrees=degrees)
    
    @staticmethod
    def from_pose_rotvec(pose_rv, seq='xyz', degrees=True):
        return RigidTransform(pose_rv=pose_rv, seq=seq, degrees=degrees)
    
    @staticmethod
    def identity(dim, degrees=True):
        if dim==1:
            return RigidTransform(hom_mat=np.identity(4))
        else:
            return RigidTransform(hom_mat=np.stack([np.identity(4) for i in range(dim)], axis=0))
            
    def __init__(self, hom_mat=None, pose=None, pose_quat=None, pose_rv=None, 
                 seq='xyz', degrees=True) -> None:
        self.translation = None
        self.rotation = None
        self.degrees = degrees
        self.reduced = False
        self.N = 0
        self.seq = seq  # Store the rotation sequence
        
        self.hom_mat = None
        self.pose = None
        self.pose_quat = None
        self.pose_rv = None
        
        if hom_mat is not None:
            if not isinstance(hom_mat, np.ndarray):
                hom_mat = np.asarray(hom_mat)
            if len(hom_mat.shape)==2:
                self.reduced = True
                hom_mat = np.expand_dims(hom_mat, axis=0)
            self.rotation = R.from_matrix(hom_mat[:,:3,:3])
            self.translation = hom_mat[:,:3,3]  # Nx3
            self.hom_mat = hom_mat
            
        elif pose is not None:
            if not isinstance(pose, np.ndarray):
                pose = np.asarray(pose)
            if len(pose.shape)==1:
                self.reduced = True
                pose = np.expand_dims(pose, axis=0)
            self.rotation = R.from_euler(seq, pose[:,3:], degrees=self.degrees)
            self.translation = pose[:,:3]  # Nx3
            self.pose = pose
            
        elif pose_quat is not None:
            if not isinstance(pose_quat, np.ndarray):
                pose_quat = np.asarray(pose_quat)
            if len(pose_quat.shape)==1:
                self.reduced = True
                pose_quat = np.expand_dims(pose_quat, axis=0)
            self.rotation = R.from_quat(pose_quat[:,3:])
            self.translation = pose_quat[:,:3]  # Nx3
            self.pose_quat = pose_quat
            
        elif pose_rv is not None:
            if not isinstance(pose_rv, np.ndarray):
                pose_rv = np.asarray(pose_rv)
            if len(pose_rv.shape)==1:
                self.reduced = True
                pose_rv = np.expand_dims(pose_rv, axis=0)
            self.rotation = R.from_rotvec(pose_rv[:,3:], degrees=self.degrees)
            self.translation = pose_rv[:,:3]  # Nx3
            self.pose_rv = pose_rv
            
        if self.translation is not None:    
            self.N = self.translation.shape[0]
        
    def set_degrees(self, deg=True):
        """
        Change the angle representation between degrees and radians.
        
        Args:
            deg: True for degrees, False for radians
        """
        if self.degrees != deg:
            self.degrees = deg
            # Clear cached representations that depend on angle units
            self.pose = None
            self.pose_rv = None
    
    def set_seq(self, seq='xyz'):
        """
        Change the Euler angle sequence.
        
        Args:
            seq: Sequence of rotation axes, e.g., 'xyz' for roll-pitch-yaw or 'XYZ' for Euler angles
        """
        if self.seq != seq:
            self.seq = seq
            # Clear cached pose representation
            self.pose = None
    
    def as_hom_mat(self):
        if self.hom_mat is None:
            self.hom_mat = np.zeros((self.N, 4, 4))
            
            self.hom_mat[:,:3,:3] = self.rotation.as_matrix()
            self.hom_mat[:,:3,3] = self.translation
            self.hom_mat[:,3,3] = np.ones((self.N,))
        
        if self.reduced:
            return self.hom_mat.reshape(4, 4)
        else:
            return self.hom_mat
    
    def as_pose(self, seq=None):
        """
        Get pose as translation + Euler angles.
        
        Args:
            seq: Optional sequence of rotation axes. If None, uses the stored sequence.
            
        Returns:
            Array of shape (6,) or (N,6) containing [x,y,z,rx,ry,rz]
        """
        if seq is None:
            seq = self.seq
            
        # If we're requesting a different sequence than cached, or no cache exists
        if self.pose is None or seq != self.seq:
            pose = np.zeros((self.N, 6))
            pose[:,:3] = self.translation
            pose[:,3:] = self.rotation.as_euler(seq, degrees=self.degrees)
            
            # Only update the cache if using the default sequence
            if seq == self.seq:
                self.pose = pose
            else:
                # Return the calculated pose without caching
                if self.reduced:
                    return pose.reshape(6,)
                else:
                    return pose
        
        if self.reduced:
            return self.pose.reshape(6,)
        else:
            return self.pose
    
    def as_quaternion(self):
        if self.pose_quat is None:
            self.pose_quat = np.zeros((self.N, 7))
            
            self.pose_quat[:,3:] = self.rotation.as_quat()
            self.pose_quat[:,:3] = self.translation
        if self.reduced:
            return self.pose_quat.reshape(7,)
        else:       
            return self.pose_quat

    def as_rot_vec(self):
        if self.pose_rv is None:
            self.pose_rv = np.zeros((self.N, 6))
            
            self.pose_rv[:,3:] = self.rotation.as_rotvec(degrees=self.degrees)
            self.pose_rv[:,:3] = self.translation
        
        if self.reduced:
            return self.pose_rv.reshape(6,)      
        else:
            return self.pose_rv
        
    def transform_points(self, points):
        """
        Transform 3D points using this rigid transformation.
        
        Args:
            points: Array of shape (N,3) containing points to transform
            
        Returns:
            Transformed points as array of shape (N,3)
        """
        # Convert to homogeneous coordinates
        if points.shape[-1] == 3:
            homogeneous = np.ones((points.shape[0], 4))
            homogeneous[:, :3] = points
        else:
            homogeneous = points
            
        # Apply transformation
        transformed = self.as_hom_mat() @ homogeneous.T
        
        # Convert back to 3D
        return (transformed[:3, :] / transformed[3, :]).T
    
    def copy(self):
        """Create a deep copy of this transform."""
        result = RigidTransform(hom_mat=self.as_hom_mat().copy(), seq=self.seq, degrees=self.degrees)
        return result

    def __invert__(self):
        res = self.as_hom_mat().copy()
        if self.reduced:
            res.shape = (1, 4, 4)
        res[:,:3,:3] = np.transpose(res[:,:3,:3], (0, 2, 1))
        res[:,:3,[3]] = -res[:,:3,:3] @ res[:,:3,[3]]
        if self.reduced:
            res.shape = (4, 4)
        return RigidTransform.from_homogeneous(res, seq=self.seq, degrees=self.degrees)

    def __matmul__(self, other):
        """composes 2 transformations. it will do that by first converting to 
        homogeneous matrices and multiplying them. since Pose objects can contain 
        a list of transforms, the resulting poses will follow the order
        [A,B,C]@[U,V] = [A@U,A@V,B@U,B@V,C@U,C@V]
        
        Args:
            other (RigidTransform): pose to right-multiply
        """
        if not isinstance(other, RigidTransform):
            raise TypeError("Can only compose with another RigidTransform")
        if not(self.reduced or other.reduced):
            # both instances are collection of transforms, first reshape and multiply
            result = ((self.as_hom_mat().reshape(-1, 1, 4, 4)) @ (other.as_hom_mat().reshape(1, -1, 4, 4))).reshape(-1, 4, 4)
        else:
            result = self.as_hom_mat() @ other.as_hom_mat()
        
        return RigidTransform.from_homogeneous(result, seq=self.seq, degrees=self.degrees)
    
    # Add support for string representation
    def __repr__(self):
        return f"RigidTransform(translation={self.translation}, rotation={self.rotation}, seq='{self.seq}')"

if __name__ == '__main__':
    # Example using the new unified pose interface
    trans1 = RigidTransform.from_pose(np.array([[0.1,0.0,0.0,0,0,0],[0.0,0.1,0.0,0,0,0],[0.0,0.0,0.1,0,0,0]]), seq='xyz')
    trans2 = RigidTransform.from_pose(np.array([0.0,0.0,0.0,90,0,0]), seq='xyz')
    
    trans_comp = trans2 @ trans1
    print(trans_comp.as_pose())  # Default is 'xyz' (RPY)
    print(trans_comp.as_pose(seq='XYZ'))  # Get as Euler angles
    
    quaternion_poses = trans1.as_quaternion()
    trans1.set_degrees(False)
    rotation_vector_poses_rad = trans1.as_rot_vec()
    trans1.set_degrees(True)
    euler_poses = trans1.as_pose()
    
    hom_points = np.array([
        [1,2,4],
        [2,4,8],
        [3,6,12],
        [1,1,1],
    ])
    # Example of RigidTransform inversion
    inverted_trans1 = ~trans1
    print("Inverted Trans1 as Pose (Euler):", inverted_trans1.as_pose())
    print("Inverted Trans1 as Homogeneous Matrix:\n", inverted_trans1.as_hom_mat())

    transformed_points = trans1.as_hom_mat() @ hom_points
    print("Trans1 as Pose (Euler):", trans1.as_pose())
    print("Trans1 as Quaternion:", quaternion_poses)
    print("Trans1 as Rotation Vector (Radians):", rotation_vector_poses_rad)
    print("Trans1 as Pose (Degrees):", euler_poses)
    print("Trans1 as Homogeneous Matrix:\n", trans1.as_hom_mat())

    print("Trans2 as Pose (Euler):", trans2.as_pose())
    print("Trans2 as Homogeneous Matrix:\n", trans2.as_hom_mat())

    print("Transformed Points:\n", transformed_points)
