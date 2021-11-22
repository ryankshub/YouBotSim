#!/usr/bin/env python3
# Main file for Final Project in ME449
# RKS

# Project imports
import mile2
# Python imports
import argparse
import csv
# 3rd-party imports
import numpy as np

if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="Calculate and output csv for final project animations"
    )
    parser.add_argument('--ani-file',
        help='output csv for animations. Currently only supports milestone 2. Defaults to "ani_file.csv"',
        default="ani_file.csv", type=str)
    parser.add_argument('--only_mile1', help='produce only the outputs for milestone 1',
        default=False, action='store_true')
    parser.add_argument('--only_mile2', help='produce only the outputs for milestone 2',
        default=False, action='store_true')
    parser.add_argument('--only_mile3', help='produce only the outputs for milestone 3',
        default=False, action='store_true')
    args = parser.parse_args()

    if args.only_mile1 or args.only_mile3:
        print("Only Milestone 2 is currently supported")
        exit(1)

    if args.only_mile2:
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
        output_file = args.ani_file
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