import numpy as np
import serial
import serial.tools.list_ports
import time

class PROSTHETIC_HAND():
    """ Base class for interfaces

    The purpose of interfaces is to abstract away the API
    and overhead of connection / disconnection etc for each
    of the different systems that can be controlled.

    Parameters
    ----------
    robot_config : class instance
        contains all relevant information about the arm
        such as: number of joints, number of links, mass information etc.
    """

    def __init__(self, dt=0.001, num_motors = 4, u_controllers = 2):
        self.dt = dt
        self.num_motors = num_motors
        self.u_controllers = u_controllers
        self.motors_per_arduino = int(self.num_motors/self.u_controllers)
        self.serial_coms = [None]*self.u_controllers; #Serial connection
        #Feedback is 4D. [thumb, index, middle, wrist]
        self.feedback = {"q":np.zeros(self.num_motors), "dq":np.zeros(self.num_motors)} 
        self.dt = 0 #will need to populate this here or main script? 
        self.cur_vals = []
        self.arduino_state = [0]*self.u_controllers #keep track in script; querying arduino too slow.

    def connect(self):
        """ All initial setup. """
        ports = serial.tools.list_ports.comports()
        #read port; third index. 1 = thumb; 2 = middle finger

        for i in range(len(ports)): 
            cur_port = serial.Serial(str(ports[i]).split()[0], 115200, timeout = 0.01)
            time.sleep(2.5) 
            for j in range(10): #Stabilize serial; clear buffer. 
                temp_val = cur_port.readline()
                # print(temp_val)

            #sometimes read fails.
            if self.safe_read(cur_port, i):
                # print(self.cur_vals)
                if int(self.cur_vals[2]) == 1:
                    self.serial_coms[0] = cur_port
                    cur_port.write("0\n".encode()) #reset arduino SM. 
                    # self.arduino_state[0] = 0
                else:
                    self.serial_coms[1] = cur_port
                    cur_port.write("0\n".encode())
                    # self.arduino_state[0] = 0

            else:
                return False


    def safe_read(self, cur_port, port_index):
        #sends signal to get value for arduino then resets arduino state machine. 
        #return list of the read values if OK
        #else returns false. 

        for i in range(5):
            cur_port.write("1111\n".encode())
            # self.arduino_state[port_index] = 1;

            try: #i use readline; sometimes characters needed for readline function are corrupted
                read_val = (cur_port.readline().decode("utf-8")).split()
                # cur_port.write("0\n".encode())
                if read_val != []:
                    self.cur_vals = read_val
                    # self.arduino_state[port_index] = 0;
                    return True
            except:
                cur_port.write("0\n".encode())
                # self.arduino_state[port_index] = 0;

        return False

    def toggle_arduino_state(self, cur_port, cur_state, port_index):
        if cur_state == 1:
            cur_port.write("0\n".encode()) #send zero command to motors
            self.arduino_state[port_index] = 0;

        elif cur_state == 0:
            cur_port.write("1111\n".encode()) #set to write mode. Note that doing this will mess up serial buffer. 
            self.arduino_state[port_index] = 1;

        else:
            print("unknown state!")


    def disconnect(self):
        """ Any socket closing etc that must be done to properly shut down
        """
        self.serial_coms = []
        print('Disconnected.')

    def send_forces(self, u): #TODO! Limitation: arduino must be in correct state.
    
        """ Applies the set of torques u to the arm. If interfacing to
        a simulation, also moves dynamics forward one time step.

        u : np.array
            An array of joint torques [Nm]
            form: [int, int, int, int]
        """
        # print(self.arduino_state)

        motors = [0]*2
        motors[0] = u[:len(u)//2]
        motors[1] = u[len(u)//2:]
        # print(motors)
        
        for i in range(len(self.serial_coms)):
            # if self.arduino_state[i] == 0:
            #     self.toggle_arduino_state(self.serial_coms[i], 0, i)

            zero_padded = [str(speed).zfill(4) for speed in motors[i]]
            # print(self.arduino_state)
            # print(str(str(zero_padded[0]) + str(zero_padded[1])))
            
            self.serial_coms[i].write(str(str(zero_padded[0]) + str(zero_padded[1]) + "\n").encode())


    def get_feedback(self):
        """ Returns a dictionary of the relevant feedback

        Returns a dictionary of relevant feedback to the
        controller. At very least this contains q, dq.
        """

        vals = [0]*self.u_controllers
        for i in range(len(self.serial_coms)): 

            # if self.arduino_state[i] == 1:
            #     self.toggle_arduino_state(self.serial_coms[i], 1, i)

            if self.safe_read(self.serial_coms[i], i): 
                vals[i] = (self.cur_vals)
            else:
                return False #read failed

        for i in range(len(vals)):
            for j in range(self.motors_per_arduino):
                #TODO: Check if value is valid.
                prev_angle = self.feedback["q"][i*self.motors_per_arduino+j]

                cur_angle = float(vals[i][j])
                if cur_angle > -80 or cur_angle < 300: #account for hyperextension
                    self.feedback["q"][i*self.motors_per_arduino+j] = cur_angle #i=1j=0 
                    self.feedback["dq"][i*self.motors_per_arduino+j] = cur_angle-prev_angle
                else:
                    self.feedback["q"][i*self.motors_per_arduino+j] = 501 #error code. 
                    self.feedback["dq"][i*self.motors_per_arduino+j] = 501
                self.feedback

        return self.feedback
