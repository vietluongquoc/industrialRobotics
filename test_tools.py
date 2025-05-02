import tools as tools
import numpy as np

tools = tools.Ultities()

# vector = np.array([1, 2, 3])
# skew_matrix = tools.vectoso3(vector)
# print("Skew-symmetric matrix from vector:")
# print(skew_matrix)

# vector = np.array([1, 2, 3, 4, 5, 6])
# se_matrix = tools.vectose3(vector)
# print("SE(3) matrix from vector:")
# print(se_matrix)

# se3mat = np.array([[1, 0, 0, 0],
#                    [0, 0, -1.5708, 2.3562],
#                    [0, 1.5708, 0, 2.3562],
#                    [0, 0, 0, 1]])

# T = tools.maxtriEXP6(se3mat)
# print("Homogeneous transformation matrix from se3mat:")
# print(T)

    #  //     T = np.array([[1, 0, 0, 0],[0, 0, -1, 0],[0, 1, 0, 3],[0, 0, 0, 1]])
    #     // output (inclue the theta):
    #     //     [S] = [1.57  0.  0.  0.  2.36  2.36]

# Tm = np.array([[1, 0, 0, 0],
#                 [0, 0, -1, 0],
#                 [0, 1, 0, 3],
#                 [0, 0, 0, 1]])
# se3mat = tools.Adjoint(Tm)
# print("se3mat from Tm:")
# print(se3mat)

# // M = math.matrix([[-1, 0, 0, 0],[0, 1, 0, 6],[0, 0, -1, 2],[0, 0, 0, 1]])
#         // Slist = math.matrix([[0,0,1,4,0,0],[0,0,0,0,1,0],[0,0,-1,-6,0,-0.1]])
#         // thetalist = math.matrix([math.pi/2, 3,math.pi]);
#         // Output:
#         // [   -0.0000    1.0000         0   -5.0000
#         //     1.0000    0.0000         0    4.0000
#         //     0         0             -1.0000    1.6858
#         //     0         0             0    1.0000 ] 

# M = np.array([[-1, 0, 0, 0],[0, 1, 0, 6],[0, 0, -1, 2],[0, 0, 0, 1]])
# Slist = np.array([[0, 0, 1, 4, 0, 0], [0, 0, 0, 0, 1, 0], [0, 0, -1, -6, 0, -0.1]])
# thetalist = np.array([np.pi/2, 3, np.pi])
# T = tools.FKinSpace(M, Slist.T, thetalist)
# print("T matrix from M, Slist, and thetalist:")
# print(T)

# Blist = np.array([[0, 0; 1;   0; 0.2; 0.2],[1; 0; 0;   2;   0;   3],[0; 1; 0;   0;   2;   1],[1; 0; 0; 0.2; 0.3; 0.4]])
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
Blist = np.array([[0, 0, 1, 0, 0.2, 0.2], [1, 0, 0, 2, 0, 3], [0, 1, 0, 0, 2, 1], [1, 0, 0, 0.2, 0.3, 0.4]])
thetalist = np.array([0.2, 1.1, 0.1, 1.2])
Jb = tools.JacobianBody(Blist.T, thetalist)
print("Jacobian matrix in body coordinates:")
print(Jb)