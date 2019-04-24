import numpy as np
import time
import nengo

np.set_printoptions(suppress=True)

class NEURAL_PD():
    """
    The base functions for all controllers

    Parameters
    ----------
    robot_config : class instance
        contains all relevant information about the arm
        such as: number of joints, number of links, mass information etc.
    """

    #By default, do nothing. 
    def __init__(self, kp=0, kd=0):
        #TODO
        self.kp = kp
        self.kd = kd
        self.prev_time = time.time()

    def print_diagnostics(self,q, dq, target, d_target, dt):
        print("dt: " + str(dt) )
        print("Target: " + str(target))
        print("Target change: " + str(d_target))
        print("Actual: " + str(q))
        print("Actual changes: " + str(dq))

    def generate_neural(self, q, dq, target, d_target):
        #normalize u

        #normalize scaling factors along with u (mind the resolution though)




        #scale u

        return u

    def generate_simple(self, q, dq, target, d_target):
        """
        Generate the torques to apply to robot joints

        Parameters
        ----------
        q : float numpy.array
            joint angles [radians]
        dq : float numpy.array
            the current joint velocities [radians/second]

        """
        # print(q)

        p = np.fix(target - q)
        proportional = p*self.kp
        
        dt = time.time() - self.prev_time

        # self.print_diagnostics(q, dq, target, d_target, dt)
        self.prev_time = time.time()

        d = (d_target - dq)/dt

        d[np.abs(d) < 1] = 0 #ignore changes in error less than half a degree. 
        derivative = d*self.kd

        u = proportional + derivative

        u[1] = 0;
        # u[2] = 0;
        # u[0] = 0;

        #Do nothing if val is 0. 
        for i in range(len(u)):
            if u[i] > 0 and abs(u[i]) < 50:
                u[i] = 50
            elif u[i] < 0 and abs(u[i]) < 50:
                u[i] = -50

        return u
        # return [0,0,0,0]
