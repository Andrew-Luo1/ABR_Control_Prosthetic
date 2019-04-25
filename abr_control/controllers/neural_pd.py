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
    def __init__(self, kp=0, kd=0, neural=False, adapt = False, num_motors=4, neuron_model=False, pes_learning_rate=1e-4):
        #TODO
        self.kp = kp
        self.kd = kd
        self.prev_time = time.time()
        self.output = np.zeros(num_motors)
        self.adapt = adapt
        self.pes_learning_rate = pes_learning_rate
        self.neuron_model = neuron_model

        if neural == True: 
            model = nengo.Network(label="Adaptive Controller")
            tau_rc =  0.02 #TODO: Check if this is in ms or s.
            tau_ref = 0.002
            if self.neuron_model == "RELU":
                cur_model = nengo.RectifiedLinear()
            elif self.neuron_model == "LIF":
                cur_model = nengo.LIF(tau_rc=tau_rc, tau_ref=tau_ref) #lif model object.
            elif self.neuron_model == "LIFRate":
                cur_model = nengo.LIFRate(tau_rc=tau_rc, tau_ref=tau_ref) #lif model object.

            def output_func(t, x):
                self.output = np.copy(x)

            def input_func_q(t, x):
                return self.q
            def input_func_dq(t, x):
                return self.dq
            def input_func_target(t, x):
                return self.target
            def input_func_d_target(t, x):
                return self.d_target

            with model:
                
                output = nengo.Node(output_func, size_in=num_motors, size_out=0)
                input_q = nengo.Node(input_func_q, size_in=num_motors, size_out=num_motors)
                input_dq = nengo.Node(input_func_dq, size_in=num_motors, size_out=num_motors)
                input_target = nengo.Node(input_func_target, size_in=num_motors, size_out=num_motors)
                input_d_target = nengo.Node(input_func_d_target, size_in=num_motors, size_out=num_motors)

                pes_learning_rate = 1e-4
                
                #Adaptive component
                if self.adapt:
                    adapt_ens = nengo.Ensemble(
                            n_neurons=1000, dimensions=num_motors,
                            radius=1.5,
                            neuron_type=cur_model)

                    learn_conn = nengo.Connection(
                            adapt_ens,
                            output,
                            learning_rule_type=nengo.PES(pes_learning_rate))

                for i in range(num_motors): #list of dictionaries
                    # Create the neuronal ensemble; 1k per layer? 
                #     A = nengo.Ensemble(1000, dimensions=1, radius=1.5) #This is tester
                    inverter = nengo.Ensemble(500, dimensions=2, radius=1.5, neuron_type = cur_model)
                    proportional = nengo.Ensemble(500, dimensions=1, radius=1.5, neuron_type = cur_model) 
                    derivative = nengo.Ensemble(500, dimensions=1, radius=1.5, neuron_type = cur_model)
                    control_signal = nengo.Ensemble(500, dimensions=1, radius=1.5, neuron_type = cur_model)
                         
                    
                    # invert terms that will be subtracted. 
                    nengo.Connection(input_q[i], inverter[0], synapse=None, function=lambda x: x*-1)
                    nengo.Connection(input_dq[i], inverter[1], synapse=None, function=lambda x: x*-1)

                    # calculate proportional part
                    nengo.Connection(inverter[0], proportional, synapse=None, function=lambda x:x*kp)
                    nengo.Connection(input_target[i], proportional, synapse=None, function=lambda x:x*kp)

                    # calculate derivative part
                    nengo.Connection(inverter[1], derivative, synapse=None, function=lambda x:(x*kd))
                    nengo.Connection(input_d_target[i], derivative, synapse=None, function=lambda x:(x*kd))

                    # output
                    nengo.Connection(proportional, control_signal)
                    nengo.Connection(derivative, control_signal)
                    nengo.Connection(control_signal, output[i])
                    
                    if self.adapt:
                        # adapt connections
                        nengo.Connection(input_q[i], adapt_ens, function=lambda x: np.zeros(num_motors), synapse=None)
                        nengo.Connection(control_signal, learn_conn.learning_rule[i], transform=-1, synapse=None)
                    
                    # Nodes to access outputs. 
                    # PID_Ens.append({"inverter":inverter, 
                    #                 "proportional":proportional, 
                    #                 "derivative": derivative, #TODO: Remove all except control signal. 
                    #                 "control_signal": control_signal})

                     # control_signal_p = nengo.Probe(PID_Ens[0]["control_signal"], synapse=.01)

            self.sim = nengo.Simulator(model)

            #input node connect up


            #Set up connections

    def print_diagnostics(self,q, dq, target, d_target, dt):
        print("dt: " + str(dt) )
        print("Target: " + str(target))
        print("Target change: " + str(d_target))
        print("Actual: " + str(q))
        print("Actual changes: " + str(dq))


    def generate_neural(self, q, dq, target, d_target):
        #normalize u

        dt = time.time() - self.prev_time
        self.prev_time = time.time()

        #normalize scaling factors along with u (mind the resolution though)
        self.q = q
        self.dq = dq*(1/dt) #Do this part of eqn here or signal to noise ratio too low
        self.target = target
        self.d_target = d_target*(1/dt)


        self.sim.run(time_in_seconds=.001, progress_bar=False)

        return self.output

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

        p=target-q
        # p = np.fix(target - q)
        proportional = p*self.kp
        
        dt = time.time() - self.prev_time

        # self.print_diagnostics(q, dq, target, d_target, dt)
        self.prev_time = time.time()

        d = (d_target - dq)/dt

        d[180*np.abs(d) < 1] = 0 #ignore changes in error less than half a degree. 
        derivative = d*self.kd

        u = proportional + derivative

        # u[1] = 0;
        # u[2] = 0;
        # u[0] = 0;

        #Do nothing if val is 0. 
        # for i in range(len(u)):
        #     if u[i] > 0 and abs(u[i]) < 50:
        #         u[i] = 50
        #     elif u[i] < 0 and abs(u[i]) < 50:
        #         u[i] = -50

        return u
        # return [0,0,0,0]
