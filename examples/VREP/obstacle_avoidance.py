"""
Move the UR5 VREP arm to a target position while avoiding an obstacle.
The simulation ends after 1.5 simulated seconds, and the trajectory
of the end-effector is plotted in 3D.
"""
import numpy as np

from abr_control.arms import ur5 as arm
# from abr_control.arms import jaco2 as arm
from abr_control.controllers import OSC, AvoidObstacles, Damping
from abr_control.interfaces import VREP

# initialize our robot config
robot_config = arm.Config(use_cython=True)
# if using the Jaco 2 arm with the hand attached, use the following instead:
# robot_config = arm.Config(use_cython=True, hand_attached=False)

# damp the movements of the arm
damping = Damping(robot_config, kv=10)
# instantiate the REACH controller with obstacle avoidance
ctrlr = OSC(robot_config, kp=200, vmax=0.5, null_controllers=[damping])
avoid = AvoidObstacles(robot_config)

# create our VREP interface
interface = VREP(robot_config, dt=.005)
interface.connect()

# set up lists for tracking data
ee_track = []
target_track = []
obstacle_track = []


try:
    num_targets = 0
    back_to_start = False

    # get visual position of end point of object
    feedback = interface.get_feedback()
    # set up the values to be used by the Jacobian for the object end effector
    start = robot_config.Tx('EE', q=feedback['q'])

    target_xyz = start + np.array([.2, -.2, 0.0])
    interface.set_xyz(name='target', xyz=target_xyz)

    moving_obstacle = True
    obstacle_xyz = np.array([0.09596, -0.2661, 0.64204])
    interface.set_xyz(name='obstacle', xyz=obstacle_xyz)

    # run ctrl.generate once to load all functions
    zeros = np.zeros(robot_config.N_JOINTS)
    ctrlr.generate(q=zeros, dq=zeros, target_pos=target_xyz)
    robot_config.R('EE', q=zeros)

    print('\nSimulation starting...\n')

    count = 0.0
    obs_count = 0.0
    while count < 1500:
        # get joint angle and velocity feedback
        feedback = interface.get_feedback()
        # calculate the control signal
        u = ctrlr.generate(
            q=feedback['q'],
            dq=feedback['dq'],
            target_pos=target_xyz,
            target_vel=np.zeros(3))

        # get obstacle position from VREP
        obs_x, obs_y, obs_z = interface.get_xyz('obstacle')
        # update avoidance system about obstacle position
        avoid.set_obstacles([[obs_x, obs_y, obs_z, 0.05]])
        if moving_obstacle is True:
            obs_x = .125 + .25 * np.sin(obs_count)
            obs_count += .05
            interface.set_xyz(name='obstacle',
                              xyz=[obs_x, obs_y, obs_z])

        # add in obstacle avoidance control signal
        u += avoid.generate(q=feedback['q'])

        # send forces into VREP, step the sim forward
        interface.send_forces(u)

        # calculate end-effector position
        ee_xyz = robot_config.Tx('EE', q=feedback['q'])
        # track data
        ee_track.append(np.copy(ee_xyz))
        target_track.append(np.copy(target_xyz))
        obstacle_track.append(np.copy([obs_x, obs_y, obs_z]))

        count += 1

finally:
    # stop and reset the VREP simulation
    interface.disconnect()

    print('Simulation terminated...')

    ee_track = np.array(ee_track)
    target_track = np.array(target_track)
    obstacle_track = np.array(obstacle_track)

    if ee_track.shape[0] > 0:
        # plot distance from target and 3D trajectory
        import matplotlib.pyplot as plt
        from mpl_toolkits.mplot3d import axes3d  # pylint: disable=W0611

        fig = plt.figure(figsize=(8,12))
        ax1 = fig.add_subplot(211)
        ax1.set_ylabel('Distance (m)')
        ax1.set_xlabel('Time (ms)')
        ax1.set_title('Distance to target')
        ax1.plot(np.sqrt(np.sum((np.array(target_track) -
                                 np.array(ee_track))**2, axis=1)))

        ax2 = fig.add_subplot(212, projection='3d')
        ax2.set_title('End-Effector Trajectory')
        ax2.plot(ee_track[:, 0], ee_track[:, 1], ee_track[:, 2], label='ee_xyz')
        ax2.scatter(target_track[0, 0], target_track[0, 1], target_track[0, 2],
                    label='target', c='g')
        ax2.plot(obstacle_track[:, 0], obstacle_track[:, 1], target_track[:, 2],
                 label='obstacle', c='r')
        ax2.legend()
        plt.show()
