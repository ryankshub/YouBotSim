#!/usr/bin/env python3
# Code for milestone 2
# RKS

# Project import

# Python import

# 3rd-party
import modern_robotics as mr
import numpy as np


def TrajectoryGenerator(T_se_init, 
                        T_sc_init, 
                        T_sc_goal, 
                        T_ce_standoff, 
                        T_ce_grasp):
    """
    Generates trajectories for grabbing a 5cm^3 volume cube and moving it 
    to the desired location

    Args:
        SE(3) T_se_init:
            The initial pose of the robot's end-effector (in space frame)
        SE(3) T_sc_init:
            The initial pose of the cube (in space frame)
        SE(3) T_sc_goal:
            The desired pose of the cube
        SE(3) T_ce_standoff:
            The pose of the end-effector in the 'standoff' position. 
        SE(3) T_ce_grasp:
            The pose of the end-effector to grab the cube
    """
    # Constants
    grip_action = 65
    time_scale_method = 5
    # Set up checkpoints. 
    T_se_pickset = np.matmul(T_sc_init, T_ce_standoff)
    T_se_grip = np.matmul(T_sc_init, T_ce_grasp)
    T_se_dropset = np.matmul(T_sc_goal, T_ce_standoff)
    T_se_open = np.matmul(T_sc_goal, T_ce_grasp)
    
    #Calculate Trajectories
    N_list1 = [(x, 0) for x in mr.ScrewTrajectory(T_se_init, T_se_pickset, 5, 500, time_scale_method)]
    N_list2 = [(x, 0) for x in mr.CartesianTrajectory(T_se_pickset, T_se_grip, 1, 100, time_scale_method)]
    N_list3 = [(N_list2[-1][0], 1) for i in range(grip_action)]
    N_list4 = [(x, 1) for x in mr.CartesianTrajectory(T_se_grip, T_se_pickset, 1, 100, time_scale_method)]
    N_list5 = [(x, 1) for x in mr.ScrewTrajectory(T_se_pickset, T_se_dropset, 5, 500, time_scale_method)]
    N_list6 = [(x, 1) for x in mr.CartesianTrajectory(T_se_dropset, T_se_open, 1, 100, time_scale_method)]
    N_list7 = [(N_list6[-1][0], 0) for i in range(grip_action)]
    N_list8 = [(x, 0) for x in mr.CartesianTrajectory(T_se_open, T_se_dropset, 1, 100, time_scale_method)]
    
    #Sum Trajectories
    rtn_list = N_list1 + N_list2 + N_list3 + N_list4 + N_list5 + N_list6 + N_list7 + N_list8
    return rtn_list