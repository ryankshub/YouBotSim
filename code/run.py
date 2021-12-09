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
import numpy as np

if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="Calculate and output csv for final project animations"
    )
    # parser.add_argument('--ani-file',
    #     help='output csv for animations. Currently only supports milestone 2. Defaults to "ani_file.csv"',
    #     default="ani_file.csv", type=str)
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

    # Configuration
    T_b0 = np.array([[1., 0., 0., 0.1662],
                     [0., 1., 0., 0.],
                     [0., 0., 1., 0.0026],
                     [0., 0., 0., 1.]])
    M_0e = np.array([[1., 0., 0., 0.033],
                     [0., 1., 0., 0.],
                     [0., 0., 1., 0.6546],
                     [0., 0., 0., 1.]])
    Blist = np.array([[0., 0., 0., 0., 0.],
                      [0., -1., -1., -1., 0.],
                      [1., 0., 0., 0., 1.],
                      [0., -0.5076, -0.3526, -0.2176, 0.],
                      [0.033, 0., 0., 0., 0.,],
                      [0., 0., 0., 0., 0.]])
    if not (args.test_mile1 or args.test_mile2 or args.test_mile3):
        ### MAIN INTEGRATED LOOP
        print("Let's integrate baby!")
        


    else:
        # Test a particular milestone
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
                    next_states = mile1.NextState(states, joint_vel+wheel_vel, 0.01, None, None)
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
                    next_states = mile1.NextState(states, joint_vel+wheel_vel, 0.01, None, None)
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
                    next_states = mile1.NextState(states, joint_vel+wheel_vel, 0.01, None, None)
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
                    next_states = mile1.NextState(states, joint_vel+wheel_vel, 0.01, None, None)
                    row = next_states.copy() + [0]
                    turn_output_writer.writerow(row)
                    states = next_states.copy()


        elif args.test_mile2:
            # Set up configuration for milestone 2
            T_se_init = np.array([[0., 0., 1., 0.],
                                [0., 1., 0., 0.],
                                [-1., 0., 0., 0.5],
                                [0., 0., 0., 1.]])

            T_sc_init = np.array([[1., 0., 0., 1.],
                                [0., 1., 0., 0.],
                                [0. ,0. ,1., 0.025],
                                [0. ,0. ,0., 1.]])
            
            T_sc_goal = np.array([[0., 1., 0., 0.],
                                [-1., 0., 0.,-1.],
                                [0., 0., 1., 0.025],
                                [0., 0., 0., 1.]])

            T_ce_standoff = np.array([[-np.sqrt(2)/2, 0., np.sqrt(2)/2, 0.0085],
                                    [0., 1., 0., 0.],
                                    [-np.sqrt(2)/2, 0., -np.sqrt(2)/2, .1],
                                    [0., 0., 0., 1.]])

            T_ce_grasp = np.array([[-np.sqrt(2)/2, 0., np.sqrt(2)/2, 0.0085],
                                [0., 1., 0., 0.],
                                [-np.sqrt(2)/2, 0., -np.sqrt(2)/2, 0.],
                                [0., 0., 0., 1.]])

            # Generate trajectory for mile2
            N_list = mile2.TrajectoryGenerator(T_se_init, 
                                            T_sc_init,
                                            T_sc_goal,
                                            T_ce_standoff,
                                            T_ce_grasp)
            # Write to csv
            output_file = "../mile2_test/traj.csv"
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