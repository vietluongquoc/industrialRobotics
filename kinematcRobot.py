from ultities import *
import numpy as np

class Robot:
    def __init__(self):
        self.DH_a = [0,0, 105, 98.3, 145]
        self.DH_d = [62.8, 0, 0, 0,0]
        self.DH_alpha = [0, -np.pi/2, 0, 0,0]
        self.dof = len(self.DH_a)-1
        self.joint_angles = np.array([np.pi/2,3*np.pi/4,np.pi/4,np.pi/2]) # Initialize joint angles to zero[90, 145, 45, 90]
        self.Tmatrix = self.forward_kinematics() # Initialize transformation matrix
    
    def degree_to_rad(self, angle):
        # Convert angle from degrees to radians
        return np.radians(angle)
    def rad_to_degree(self, angle):
        # Convert angle from radians to degrees
        return np.degrees(angle)
    
    def update_joint_angles(self, angles):
        
        if len(angles) != self.dof:
            raise ValueError("Number of angles must match the number of degrees of freedom.")
        angleRad = [self.degree_to_rad(angle) for angle in angles]
        self.joint_angles = np.array(angleRad)
        self.Tmatrix = self.forward_kinematics()

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

    def inverse_kinematics(self, T_target):
        if T_target.shape != (4, 4):
            raise ValueError("Target transformation matrix must be 4x4.")
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
        theta_target = IKinSpace(calistarr,initial_matrix,T_target,self.joint_angles , 0.1 , 1)
        theta_angles = []

        if theta_target[1] is True:
            theta_rad = theta_target[0]
            theta_angles = [180*(theta_rad[i])/np.pi for i in range(len(theta_rad))]
        # return Blist
        return [theta_angles, theta_target[1]]

    def joy_control(self, direction, step=5):
        # Calculate the new transformation matrix based on the target position
        T_target = self.Tmatrix.copy()
        
        if direction == 'X+':
            T_target[0, 3] += step
        elif direction == 'X-':
            T_target[0, 3] -= step
        elif direction == 'Z+':
            T_target[2, 3] += step
        elif direction == 'Z-':
            T_target[2, 3] -= step
        # Calculate the angles using inverse kinematics
        
        return self.inverse_kinematics(T_target)