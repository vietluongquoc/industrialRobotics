import numpy as np

class Ultities:
    def __init__(self):
        self.name = "Kinematic Utilities functions"
        self.version = "1.0"
        self.author = "Quoc Viet, Luong"

    def vectoso3(self, omega):
        # Convert a 3D vector to a skew-symmetric matrix (so3 representation)
        # // This function return screw matrix from w vector
        # // input:
        # //  vec = [1, 2, 3]
        # // Output = 
        # // [ 0    -3     2
        # //    3     0    -1
        # //   -2     1     0]
        return np.array([[0, -omega[2], omega[1]],
                         [omega[2], 0, -omega[0]],
                         [-omega[1], omega[0], 0]])
    def vectose3(self, omega):
        # // Get T matrices from w and v
        # // Input: [1; 2; 3; 4; 5; 6];
        # //Output:
        # //    [   0    -3     2     4
        # //         3     0    -1     5
        # //        -2     1     0     6
        # //         0     0     0     1]
        return np.array([[0, -omega[2], omega[1], omega[3]],
                         [omega[2], 0, -omega[0], omega[4]],
                         [-omega[1], omega[0], 0, omega[5]],
                         [0, 0, 0, 1]])
    
    def transTorp(self, T):   
        # //Get w matrices and p vector
        # // Input = [[1, 0, 0, 0]; [0, 0, -1, 0]; [0, 1, 0, 3]; [0, 0, 0, 1]];
        # // Output:  
        # // R = [1     0     0
        # //      0     0    -1
        # //      0     1     0];   
        # //  p =[0,0,3]

        R = np.array([[T[0, 0], T[0, 1], T[0, 2]],
                      [T[1, 0], T[1, 1], T[1, 2]],
                      [T[2, 0], T[2, 1], T[2, 2]]])
        p = np.array([T[0, 3], T[1, 3], T[2, 3]])

        return R, p
    
    def maxtriEXP6(self,se3mat):
        # // Get homogeneous transformation matrices T using exponentials formula
        # // input:
        # // se3mat = [ 0,      0,       0,      0;
        # //             0,      0, -1.5708, 2.3562;
        # //             0, 1.5708,       0, 2.3562;
        # //             0,      0,       0,      0]
        # // # Output:
        # //     T =[  1.0000     0         0         0
        # //             0    0.0000   -1.0000   -0.0000
        # //             0    1.0000    0.0000    3.0000
        # //             0         0         0    1.0000 ]
        se3mat = np.array(se3mat)
        omgtheta = np.array([se3mat[2, 1], se3mat[0, 2], se3mat[1, 0]])
        if np.linalg.norm(omgtheta)<0.0001:
            return np.r_[np.c_[np.eye(3), se3mat[0: 3, 3]], [[0, 0, 0, 1]]]
        else:
            theta = np.linalg.norm(omgtheta)
            omgmat = se3mat[0: 3, 0: 3] / theta
            R = np.eye(3) + np.sin(theta) * omgmat + (1 - np.cos(theta)) * np.dot(omgmat, omgmat)
            p = np.dot(np.eye(3) * theta + (1 - np.cos(theta)) * omgmat + (theta - np.sin(theta)) * np.dot(omgmat, omgmat), se3mat[0: 3, 3]) / np.linalg.norm(omgtheta)
            return np.r_[np.c_[R, p], [[0, 0, 0, 1]]]
            
    def maxtrixLog6(self,Tm):
        # //     T = np.array([[1, 0, 0, 0],[0, 0, -1, 0],[0, 1, 0, 3],[0, 0, 0, 1]])
        # // output (inclue the theta):
        # //     [S] = [1.57  0.  0.  0.  2.36  2.36]
        
        R, p = self.transTorp(Tm)
        acosin = (np.trace(R) - 1) / 2
        
        if acosin >= 1:
            so3mat = np.zeros((3, 3))
            theta = 0.0001
        elif acosin <= -1:
            if np.linalg.norm(R[2,2]) >= 0.0000001:
                omgmat = 1/np.sqrt(2*(1+R[2,2]))*R[0:2,2]
            elif np.linalg.norm(R[1,1]) >= 0.0000001:
                omgmat = 1/np.sqrt(2*(1+R[1,1]))*R[0:2,1]
            else:
                omgmat = 1/np.sqrt(2*(1+R[0,0]))*R[0:2,0]
            so3mat = self.vectoso3(np.pi@omgmat)
            theta = np.pi
        
        else:
            theta = np.arccos(acosin)
            so3mat = theta/(2*np.sin(theta))*(R - np.transpose(R))
        
        Ginverse = np.eye(3) + (-0.5) * so3mat + (1/theta)*(1/theta - 1/(2*(np.tan(theta/2))))*np.dot(so3mat, so3mat)
        vout = Ginverse@p
        return np.array([so3mat[2,1], so3mat[0,2], so3mat[1,0], vout[0], vout[1], vout[2]])
    
    def Adjoint(self, T):
        # // Get Adjoint matrix from T matrix
        # // Input:
        # // T = [[1, 0, 0, 0],
        # //      [0, 0, -1, 0],
        # //      [0, 1, 0, 3],
        # //      [0, 0, 0, 1]]
        # // Output:
        # // [   
        # //     1     0     0     0     0     0
        # //     0     0    -1     0     0     0
        # //     0     1     0     0     0     0
        # //     0     0     3     1     0     0
        # //     3     0     0     0     0    -1
        # //     0     0     0     0     1     0 
        # // ]
        R, p = self.transTorp(T)
        upAdT = np.concatenate((R, np.zeros((3, 3))), axis=1)
        downAdT = np.concatenate((self.vectoso3(R@p), R), axis=1)
        return np.concatenate((upAdT, downAdT), axis=0)
    
    def FKinSpace(self, M, Slist, thetalist):
        # // Get T matrix from M and Slist and thetalist
        # // Input:
        # // M = math.matrix([[-1, 0, 0, 0],[0, 1, 0, 6],[0, 0, -1, 2],[0, 0, 0, 1]])
        # // Slist = math.matrix([[0,0,1,4,0,0],[0,0,0,0,1,0],[0,0,-1,-6,0,-0.1]])
        # // thetalist = math.matrix([math.pi/2, 3,math.pi]);
        # // Output:
        # // [   -0.0000    1.0000         0   -5.0000
        # //     1.0000    0.0000         0    4.0000
        # //     0         0             -1.0000    1.6858
        # //     0         0             0    1.0000 ] 

        T = M.copy()
        for i in range(len(thetalist) - 1, -1, -1):
            T = np.dot(self.maxtriEXP6(self.vectose3(Slist[:, i]*thetalist[i])), T)
        return T
    
    def JacobianBody(self, Blist, thetalist):
        # // Calculate Jacobian matrices in body coordinate
        # // input
        # // Blist = [   [0; 0; 1;   0; 0.2; 0.2], ...
        # //             [1; 0; 0;   2;   0;   3], ...
        # //             [0; 1; 0;   0;   2;   1], ...
        # //             [1; 0; 0; 0.2; 0.3; 0.4]];
        # // thetalist = [0.2; 1.1; 0.1; 1.2];
        # // output
        # //  Jb =[
        # //        -0.0453    0.9950         0    1.0000
        # //         0.7436    0.0930    0.3624         0
        # //        -0.6671    0.0362   -0.9320         0
        # //         2.3259    1.6681    0.5641    0.2000
        # //        -1.4432    2.9456    1.4331    0.3000
        # //        -2.0664    1.8288   -1.5887    0.4000 
        # //     ] 
        Jb = np.array(Blist).copy().astype(float)
        T = np.eye(4)
        for i in range(len(thetalist) - 2, -1, -1):
            T = np.dot(T,self.maxtriEXP6(self.vectose3(np.array(Blist)[:, i + 1] \
                                            * -thetalist[i + 1])))
            Jb[:, i] = np.dot(self.Adjoint(T), np.array(Blist)[:, i])
        return Jb
    
    def screwTrajectory(self,Tstart, Tend, N):
        # // Get screw trajectory from Tstart to Tend
        # // Input:
        # // Tstart = [[1, 0, 0, 0],
        # //           [0, 0, -1, 0],
        # //           [0, 1, 0, 3],
        # //           [0, 0, 0, 1]]
        # // Tend = [[1, 0, 0, 0],
        # //          [0, 0, -1, 0],
        # //          [0, 1, 0, 3],
        # //          [0, 0, 0, 1]]
        # // N = 10
        # // Output:
        # // [[1.00000000e+00 -2.22044605e-16 -2.22044605e-16 -2.22044605e-16]
        # // [-2.22044605e-16 -2.22044605e-16 -2.22044605e-16 -2.22044605e-16]
        # // [-2.22044605e-16 -2.22044605e-16 -2.22044605e-16 -2.22044605e-16]
        # // [-2.22044605e-16 -2.22044605e-16 -2.22044605e-16 -2.22044605e-16]]
             
        timegap =  0.02
        TF = timegap * (N - 1)
        
        Vb = self.maxtrixLog6(np.dot(np.linalg.inv(Tstart), Tend))
        traj = []
        for i in range(0, N):
            s = 3*np.pow(timegap*i/TF, 2) - 2*np.power(timegap*i/TF, 3)
            Matrixv = self.maxtrixLog6(self.vectose3(Vb * s))
            traj.append(np.dot(Tstart, Matrixv))
        return traj
    
    def IKspace(self, M, Slist, T, theta0):
        # // Get thetalist from M, Slist and T
        # // Input:
        # // M = math.matrix([[-1, 0, 0, 0],[0, 1, 0, 6],[0, 0, -1, 2],[0, 0, 0, 1]])
        # // Slist = math.matrix([[0,0,1,4,0,0],[0,0,0,0,1,0],[0,0,-1,-6,0,-0.1]])
        # // T = [[1.00000000e+00 -2.22044605e-16 -2.22044605e-16 -2.22044605e-16]
        # //       [-2.22044605e-16 -2.22044605e-16 -2.22044605e-16 -2.22044605e-16]
        # //       [-2.22044605e-16 -2.22044605e-16 -2.22044605e-16 -2.22044605e-16]
        # //       [-2.22044605e-16 -2.22044605e-16 -2.22044605e-16 -2.22044605e-16]]
        # // Output:
        # // [   3.1416    3.1416    3.1416    3.1416]
        
        interate = 20
        errVb = 0.0001

        AdjM = self.Adjoint(M)
        Blist = np.dot(np.linalg.inv(AdjM), Slist)

        its = 0
        thetalist  = [theta0]

        while its < interate:
            Tsb = self.FKinSpace(M, Blist, T)
            Vb = self.maxtrixLog6(np.dot(np.linalg.inv(Tsb), T))

            if np.linalg.norm(Vb) >= errVb:
                Jinv = np.linalg.pinv(self.JacobianBody(Blist.T, thetalist[its]))
                tempts = thetalist[its] + np.dot(Jinv, Vb)
                editTheta = np.sin(tempts)
                thetalist.append(editTheta)
                isSolve = False
            else:
                isSolve = True
                break
            its = its + 1

        return isSolve, thetalist[its-1]