#!/usr/bin/env python3
# Main file for Final Project in ME449
# RKS

# Project imports
import mile1
import mile2
import mile3
# Python imports
import argparse
import csv
# 3rd-party imports
import modern_robotics as mr
import numpy as np


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="Calculate and output csv for final project animations"
    )
    parser.add_argument('--output',
        help='output csv for animations. Currently only supports milestone 2. Defaults to "output.csv"',
        default="output.csv", type=str)
    parser.add_argument('--test_mile1', help='produce only the outputs for milestone 1',
        default=False, action='store_true')
    parser.add_argument('--test_mile2', help='produce only the outputs for milestone 2',
        default=False, action='store_true')
    parser.add_argument('--test_mile3', help='produce only the outputs for milestone 3',
        default=False, action='store_true')
    args = parser.parse_args()

    # Constants
    NUM_STATES = 12
    NUM_WHEEL = 4
    NUM_JOINT = 5

    ### CONFIGURATION ###
    # Transformation matrix from frame b to base of arm
    T_b0 = np.array([[1., 0., 0., 0.1662],
                     [0., 1., 0., 0.],
                     [0., 0., 1., 0.0026],
                     [0., 0., 0., 1.]])
    # Transformation matrix from arm base to end effector 
    # while robot is in the home position
    M_0e = np.array([[1., 0., 0., 0.033],
                     [0., 1., 0., 0.],
                     [0., 0., 1., 0.6546],
                     [0., 0., 0., 1.]])
    
    # Screw axis of the robot for each joint in zero position
    Blist = np.array([[0., 0., 0., 0., 0.],
                      [0., -1., -1., -1., 0.],
                      [1., 0., 0., 0., 1.],
                      [0., -0.5076, -0.3526, -0.2176, 0.],
                      [0.033, 0., 0., 0., 0.,],
                      [0., 0., 0., 0., 0.]])

    # Init position and orientation of the robot
    T_se_init = np.array([[0., 0., 1., 0.],
                        [0., 1., 0., 0.],
                        [-1., 0., 0., 0.5],
                        [0., 0., 0., 1.]])

    # Test position for feedforward control
    T_se_home = np.array([[-0.309 ,  0.    ,  0.9511,  0.4721],
       [ 0.    ,  1.    ,  0.    ,  0.    ],
       [-0.9511,  0.    , -0.309 ,  0.4601],
       [ 0.    ,  0.    ,  0.    ,  1.    ]])


    # Init position and orientation of the cube
    T_sc_init = np.array([[1., 0., 0., 1.],
                        [0., 1., 0., 0.],
                        [0. ,0. ,1., 0.025],
                        [0. ,0. ,0., 1.]])
    
    # Goal position and orientation of the cube
    T_sc_goal = np.array([[0., 1., 0., 0.],
                        [-1., 0., 0.,-1.],
                        [0., 0., 1., 0.025],
                        [0., 0., 0., 1.]])

    # Transformation matrix representing position of the end-effector
    # relative from the cube to prepare grabbing it
    T_ce_standoff = np.array([[-np.sqrt(2)/2, 0., np.sqrt(2)/2, 0.0085],
                            [0., 1., 0., 0.],
                            [-np.sqrt(2)/2, 0., -np.sqrt(2)/2, .1],
                            [0., 0., 0., 1.]])

    # Transformation matrix represention position of the end-effector
    # relative from the cube to grab it
    T_ce_grasp = np.array([[-np.sqrt(2)/2, 0., np.sqrt(2)/2, 0.0085],
                        [0., 1., 0., 0.],
                        [-np.sqrt(2)/2, 0., -np.sqrt(2)/2, 0.],
                        [0., 0., 0., 1.]])
    

    if not (args.test_mile1 or args.test_mile2 or args.test_mile3):
        ### MAIN INTEGRATED LOOP
        dt = 0.01
        ee_trajs = mile2.TrajectoryGenerator(T_se_init, 
                                             T_sc_init,
                                             T_sc_goal,
                                             T_ce_standoff,
                                             T_ce_grasp,
                                             dt)
        chassis_states = np.zeros(3)
        joint_states = np.array([0.0, -np.pi/20, -np.pi/20, -np.pi/2, 0.0])
        wheel_states = np.zeros(4)
        Integration_Err = np.zeros(6)
        Xerr_log = []
        states_log = []
        states_log.append(np.zeros(13))

        for i in range(len(ee_trajs) - 1):
            # Calculate inv Jacobian
            pinv_Je = mile3.GetPinvJacobian(M_0e, T_b0, Blist, joint_states)
            
            # Calculate current T_se
            T_sb = mile2.GetTsb(chassis_states[0],
                                chassis_states[1],
                                chassis_states[2])
            T_0e = mr.FKinBody(M_0e, Blist, joint_states)
            curr_X = np.around( np.matmul( np.matmul(T_sb, T_b0), T_0e ), 4)
            # desi_X = np.around(ee_trajs[i][0], 3)
            # desi_X_n = np.around(ee_trajs[i+1][0], 3)
            # print("Current X", f"{curr_X}")
            # print("Desired X", f"{ee_trajs[i][0]}")
            # Use FeedbackControl
            velocities, Xerr_sum, Xerr = mile3.FeedbackControl(curr_X, 
                                                               ee_trajs[i][0], 
                                                               ee_trajs[i+1][0],
                                                               1,
                                                               0,
                                                               Integration_Err, 
                                                               pinv_Je,
                                                               dt)
            # print("Velcities", f"{velocities}")
            # Update integration error
            Integration_Err = Xerr_sum
            # Log X_err
            Xerr_log.append(((i+1)*dt, Xerr))

            # Update states
            states = np.hstack((chassis_states, joint_states, wheel_states))
            # print(f"states {states}")
            next_states = mile1.NextState(states, velocities, dt, 30, [20, 20, 20, 20, 20])
            # print(f"next_states {next_states}")
            # Log states
            complete_states = np.hstack((next_states, np.array([ee_trajs[i][1]])))
            states_log.append(complete_states)
            # Update states for next loop
            chassis_states = next_states[0:3]
            joint_states = next_states[3:8]
            wheel_states = next_states[8:]

        
        # Write logs to files
        output_file = args.output
        with open(output_file, 'w', newline='') as csvfile:
            output_writer = csv.writer(csvfile)
            for s in states_log:
                output_writer.writerow(s)
        
        xerr_file = "xerr_log.csv"
        with open(xerr_file, 'w', newline='') as csvfile:
            xerr_writer = csv.writer(csvfile)
            for err in Xerr_log:
                xerr_writer.writerow(err)


    else:
        ### MILESTONE TEST ###
        if args.test_mile1:
            # Make zero motion
            init_states = [0. for x in range(NUM_STATES)]
            wheel_vel = [0. for x in range(NUM_WHEEL)]
            joint_vel = [0. for x in range(NUM_JOINT)]
            # Write to csv
            zero_output_file = "../mile1_tests/zm.csv"
            with open(zero_output_file, 'w', newline='') as csvfile:
                zero_output_writer = csv.writer(csvfile)
                states = init_states
                
                for i in range(100):
                    next_states = mile1.NextState(states, wheel_vel+joint_vel, 0.01, None, None)
                    row = next_states.copy() + [0]
                    zero_output_writer.writerow(row)
                    states = next_states.copy()
            
            # Make forward motion
            init_states = [0 for x in range(NUM_STATES)]
            wheel_vel = [10 for x in range(NUM_WHEEL)]
            joint_vel = [0 for x in range(NUM_JOINT)]
            # Write to csv
            forward_output_file = "../mile1_tests/fm.csv"
            with open(forward_output_file, 'w', newline='') as csvfile:
                forward_output_writer = csv.writer(csvfile)
                states = init_states
                
                for i in range(100):
                    next_states = mile1.NextState(states, wheel_vel+joint_vel, 0.01, None, None)
                    row = next_states.copy() + [0]
                    forward_output_writer.writerow(row)
                    states = next_states.copy()
            
            # Make side motion
            init_states = [0 for x in range(NUM_STATES)]
            wheel_vel = [-10, 10, -10, 10]
            joint_vel = [0 for x in range(NUM_JOINT)]
            # Write to csv
            side_output_file = "../mile1_tests/sm.csv"
            with open(side_output_file, 'w', newline='') as csvfile:
                side_output_writer = csv.writer(csvfile)
                states = init_states
                
                for i in range(100):
                    next_states = mile1.NextState(states, wheel_vel+joint_vel, 0.01, None, None)
                    row = next_states.copy() + [0]
                    side_output_writer.writerow(row)
                    states = next_states.copy()

            # Make counter-clockwise turn
            init_states = [0 for x in range(NUM_STATES)]
            wheel_vel = [-10, 10, 10, -10]
            joint_vel = [0 for x in range(NUM_JOINT)]
            # Write to csv
            turn_output_file = "../mile1_tests/tm.csv"
            with open(turn_output_file, 'w', newline='') as csvfile:
                turn_output_writer = csv.writer(csvfile)
                states = init_states
                
                for i in range(100):
                    next_states = mile1.NextState(states, wheel_vel+joint_vel, 0.01, None, None)
                    row = next_states.copy() + [0]
                    turn_output_writer.writerow(row)
                    states = next_states.copy()


        elif args.test_mile2:
            # Generate trajectory for mile2
            N_list = mile2.TrajectoryGenerator(T_se_init, 
                                            T_sc_init,
                                            T_sc_goal,
                                            T_ce_standoff,
                                            T_ce_grasp,
                                            0.01)
            # Write to csv
            output_file = "../mile2_tests/traj.csv"
            with open(output_file, 'w', newline='') as csvfile:
                output_writer = csv.writer(csvfile)
                for N in N_list:
                    T = N[0]
                    gripper = N[1]
                    row = [T[0][0], 
                        T[0][1],
                        T[0][2],
                        T[1][0], 
                        T[1][1],
                        T[1][2],
                        T[2][0], 
                        T[2][1],
                        T[2][2],
                        T[0][3],
                        T[1][3],
                        T[2][3],
                        gripper]
                    output_writer.writerow(row)
        
        elif args.test_mile3:
            # Set up configuration 
            test_thetalist = np.array([0., 0., 0.2, -1.6, 0.])
            test_X = np.array([[0.170, 0., 0.985, 0.387],
                            [0., 1., 0., 0.],
                            [-0.985, 0., 0.170, 0.570],
                            [0., 0., 0., 1.]])
            test_Xd = np.array([[0., 0., 1., 0.5],
                            [0., 1., 0., 0.],
                            [-1., 0., 0., 0.5],
                            [0., 0., 0., 1.]])
            test_Xdnext = np.array([[0., 0., 1., 0.6],
                            [0., 1., 0., 0.],
                            [-1., 0., 0., 0.3],
                            [0., 0., 0., 1.]])
            test_dt = 0.01
            # Grab psuedo inverse Jacobian
            test_pinv_Je = mile3.GetPinvJacobian(M_0e, T_b0, Blist, test_thetalist)
            # Test feedback Control
            test_vels, _ = mile3.FeedbackControl(test_X, 
                                            test_Xd, 
                                            test_Xdnext, 
                                            1, 
                                            0, 
                                            np.zeros(6), 
                                            test_pinv_Je, 
                                            test_dt)

        else:
            # Do nothing
            pass