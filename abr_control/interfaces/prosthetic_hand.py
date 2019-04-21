import numpy as np
import serial
import serial.tools.list_ports
import time
from abr_control.utils import transformations
from .interface import Interface


# TODO: add ability to load models files so that vrep only has to be open
class PROSTHETIC_HAND(Interface):
    """ An interface for VREP.

    Implements force control using VREP's torque-limiting method.
    Lock-steps the simulation so that it only moves forward one dt
    every time send_forces is called.

    Parameters
    ----------
    robot_config : class instance
        contains all relevant information about the arm
        such as: number of joints, number of links, mass information etc.
    dt : float, optional (Default: 0.001)
        simulation time step in seconds
    """

#Unknown: Does the inherited init automatically get called here?
    # def __init__(self, robot_config, dt=.001):

    #     super(VREP, self).__init__(robot_config)

    #     self.q = np.zeros(self.robot_config.N_JOINTS)  # joint angles
    #     self.dq = np.zeros(self.robot_config.N_JOINTS)  # joint_velocities

    #     # joint target velocities, as part of the torque limiting control
    #     # these need to be super high so that the joints are always moving
    #     # at the maximum allowed torque
    #     self.joint_target_velocities = (np.ones(robot_config.N_JOINTS) *
    #                                     10000.0)

    #     self.dt = dt  # time step
    #     self.count = 0  # keep track of how many times send forces is called
    #     self.misc_handles = {}  # for tracking miscellaneous object handles
    def __init__(self, dt=.001):
        self.dt = dt
        self.ser1 = None; #Serial connection
        self.feedback = {}

    def connect(self):
        """ Connect to the current scene open in VREP

        Finds the VREP references to the joints of the robot.
        Sets the time step for simulation and put into lock-step mode.

        NOTE: the joints and links must follow the naming convention of
        'joint#' and 'link#', starting from 'joint0' and 'link0'

        NOTE: The dt in the VREP physics engine must also be specified
        to be less than the dt used here.
        """

        # close any open connections
        

        # Init connection with arduino
        # The arduinos should already have been inited.
        self.ser1 = serial.Serial(str(serial.tools.list_ports.comports()[0]).split()[0], 9600)



    def disconnect(self):
        """ Stop and reset the simulation. """

        #Probably non-essential
        self.ser1 = None
        print('Disconnected.')


    def send_forces(self, u):
        """ Apply the specified torque to the robot joints

        Apply the specified torque to the robot joints, move the simulation
        one time step forward, and update the position of the hand object.

        Parameters
        ----------
        u : np.array
            the torques to apply to the robot
        """
        ser1.write(str(u).encode())

        #Send the set_speed thing for the arduino

    def get_feedback(self):
        """ Return a dictionary of information needed by the controller.

        Returns the joint angles and joint velocities in [rad] and [rad/sec],
        respectively
        """

        #Get joint angles via serial connection. 
        #format: [q, dq, millis]
        new_vals = self.ser1.readline()
        millis = new_vals.split()[0]
        angle = new_vals.split()[1]

        feedback["dq"] = angle - feedback["q"]
        feedback["q"] = angle
        feedback["millis"] = millis

        return feedback