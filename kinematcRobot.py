from ultities import *
import numpy as np

class Robot:
    def __init__(self):
        self.DH_a = [0,0, 105, 98.3, 145]
        self.DH_d = [62.8, 0, 0, 0,0]
        self.DH_alpha = [0, -np.pi/2, 0, 0,0]
        self.dof = len(self.DH_a)-1
        self.joint_angles = np.zeros(self.dof) # Initialize joint angles to zero
        self.Tmatrix = self.forward_kinematics() # Initialize transformation matrix
    
    def update_joint_angles(self, angles):
        if len(angles) != self.dof:
            raise ValueError("Number of angles must match the number of degrees of freedom.")
        self.joint_angles = np.array(angles)

    def geometryKinematics(self):
        theta1 = self.joint_angles[0]
        theta2 = -self.joint_angles[1]
        theta3 = -self.joint_angles[2]
        theta4 = -self.joint_angles[3]
        # Calculate the end-effector position using the joint angles and DH parameters
        x = np.cos(theta1)*(self.DH_a[2]*np.cos(theta2)+ self.DH_a[3]*np.cos(theta2+theta3+ self.DH_a[4]*np.cos(theta2+theta3+theta4)))
        y = np.sin(theta1)*(self.DH_a[2]*np.cos(theta2)+ self.DH_a[3]*np.cos(theta2+theta3)+ self.DH_a[4]*np.cos(theta2+theta3+theta4))
        z = self.DH_d[0] + self.DH_a[2]*np.sin(theta2) + self.DH_a[3]*np.sin(theta2+theta3)+ self.DH_a[4]*np.sin(theta2+theta3+theta4)
        return x, y, z

    def forward_kinematics(self):
        # Initialize transformation matrix
        T = np.eye(4)
        for i in range(self.dof+1):
            if i == self.dof:
                theta = 0
            else:
                theta = self.joint_angles[i]
            a = self.DH_a[i]
            d = self.DH_d[i]
            alpha = self.DH_alpha[i]

            # Compute the transformation matrix using DH parameters
            T_i = np.array([[np.cos(theta), -np.sin(theta) , 0, a ],
                            [np.sin(theta)* np.cos(alpha), np.cos(theta) * np.cos(alpha), -np.sin(alpha), -d * np.sin(alpha)],
                            [np.sin(theta)*np.sin(alpha), np.cos(theta)*np.sin(alpha), np.cos(alpha), d*np.cos(alpha)],
                            [0, 0, 0, 1]])
            
            # Update the overall transformation matrix
            T = T @ T_i
        return T

    def inverse_kinematics(self, target_position = [348.3, 0, 62.8]):
        if len(target_position) != 3:
            raise ValueError("Target position must be a 3D coordinate (x, y, z).")
        x, y, z = target_position
        # Calculate the new transformation matrix based on the target position
        T_target = self.Tmatrix.copy()
        T_target[0, 3] = x
        T_target[1, 3] = y
        T_target[2, 3] = z
        # Calculate the angles using inverse kinematics
        initial_matrix = np.array([[1, 0, 0, 105+98.3+145],
                                   [0, 0, 1, 0],
                                   [0, -1, 0, 62.8],
                                   [0, 0, 0, 1]])
        joint_w = [[0,0,1],[0,1,0],[0,1,0],[0,1,0]]
        joint_p =[[0,0,0],[0,0,62.8],[105,0,62.8],[105+98.3,0,62.8]]
        calist = []
        for i in range(self.dof):
            temp1 = -1*np.cross(np.array(joint_w[i]),np.array(joint_p[i]))
            temp2 = np.concatenate((np.array(joint_w[i]),temp1), axis=0)
            calist.append(temp2)
        calistarr = np.transpose(np.array(calist))
        # return FKinSpace(initial_matrix,calistarr, self.joint_angles)
        theta_target = IKinSpace(calistarr,initial_matrix, T_target, self.joint_angles, 0.001, 0.1)
        theta_angles = np.zeros(self.dof-1)
        if theta_target[1] is True:
            theta_rad = theta_target[0]

            for i in range(self.dof-1):
                theta_angles[i] = theta_rad[i]*180/np.pi
       
        # return Blist
        return theta_angles