import numpy as np
import tools as tools

class moRobot:
    def __init__(self, initial_raw_current):
        super().__init__()
        self.d = 62.8
        self.a1 = 105
        self.a2 = 90
        self.a3 = 70
        self.joins = [{'name': 'join1', 'w1': np.array([0,0,-1]), 'q1': np.array([0,0,0])},
                      {'name': 'join2', 'w1': np.array([0,-1,0]), 'q1': np.array([0,0,self.d])},
                      {'name': 'join3', 'w1': np.array([0,1,0]), 'q1': np.array([self.a1,0,self.d])},
                      {'name': 'join4', 'w1': np.array([0,1,0]), 'q1': np.array([self.a1+self.a2,0,self.d])}]
        self.calibration = np.array([0,0,0,0])
        self.Slist
        self.initial_matrix = np.array([[1, 0, 0, self.a1+self.a2+self.a3],
                                   [0, 0, 1, 0],
                                   [0, -1, 0, self.d],
                                   [0, 0, 0, 1]])
        self.tools = tools.Ultities()
        self.calSlist()
        self.raw_current = self.raw2rad(initial_raw_current)
        self.cur_T
        self.update_cur_T()

    def raw2rad(self, raw_current):
        # Convert raw current to radians
        # // Input: [0, 0, 0, 0]
        # // Output: [0.0000, 0.0000, 0.0000, 0.0000]
        return np.array([raw_current[0] * np.pi / 180,
                         raw_current[1] * np.pi / 180,
                         raw_current[2] * np.pi / 180,
                         raw_current[3] * np.pi / 180])
    
    def rad2raw(self, rad_current):
        # Convert radians to raw current
        # // Input: [0.0000, 0.0000, 0.0000, 0.0000]
        # // Output: [0, 0, 0, 0]
        return [rad_current[0] * 180 / np.pi, rad_current[1] * 180 / np.pi, rad_current[2] * 180 / np.pi, rad_current[3] * 180 / np.pi]
    
    
    def calSlist(self):
        self.Slist = np.zeros((6, len(self.joins)))
        for i in range(len(self.joins)):
            self.Slist[0:3, i] = self.joins[i]['w1']
            self.Slist[3:6, i] = np.cross(-self.joins[i]['w1'], self.joins[i]['q1'])
        return self.Slist

    def manForwardKi(self,thetalist):
        return tools.FKinSpace(self.initial_matrix, self.Slist, thetalist)
    
    def manInverseKi(self, T):
        isSolve, thetalist = tools.FKinSpace(self.initial_matrix, self.Slist,T)
        if isSolve:
            return self.raw2rad(thetalist)
    
    def update_cur_T(self, thetalist=None):
        if thetalist is None:
            thetalist = self.raw2rad(self.raw_current)
        self.cur_T = self.tools.FKinSpace(self.initial_matrix, self.Slist, thetalist)
        return self.cur_T
    
    def update_raw_current(self, raw_current):
        self.raw_current = raw_current
        self.update_cur_T()
        return self.cur_T
    
    def joy_control(self, direction, step):
        # Calculate the new transformation matrix based on the target position
        T_target = self.cur_T.copy()
        
        if direction == 'X+':
            T_target[0, 3] += step
        elif direction == 'X-':
            T_target[0, 3] -= step
        elif direction == 'Z+':
            T_target[2, 3] += step
        elif direction == 'Z-':
            T_target[2, 3] -= step
        # Calculate the angles using inverse kinematics
        

        return self.manInverseKi(T_target)
    
