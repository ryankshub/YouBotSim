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
                        T_ce_grasp,
                        dt):
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
        float dt:
            Time duration
    """
    # Constants
    grip_action = 65
    time_scale_method = 5
    # Set up checkpoints. 
    T_se_pickset = np.matmul(T_sc_init, T_ce_standoff)
    T_se_grip = np.matmul(T_sc_init, T_ce_grasp)
    T_se_dropset = np.matmul(T_sc_goal, T_ce_standoff)
    T_se_open = np.matmul(T_sc_goal, T_ce_grasp)
    # Set up time duration
    screw_duration = 5  # number of seconds a screw motion should take
    screw_samples = np.around(screw_duration/dt)
    turn_duration = 10  # number of seconds a screw motion should take
    turn_samples = np.around(turn_duration/dt)
    cart_duration = 1   # number of seconds a cart motion should take
    cart_samples = np.around(cart_duration/dt)
    
    #Calculate Trajectories
    N_list1 = [(np.around(x,4), 0) for x in mr.ScrewTrajectory(T_se_init, T_se_pickset, screw_duration, screw_samples, time_scale_method)]
    N_list2 = [(np.around(x,4), 0) for x in mr.CartesianTrajectory(T_se_pickset, T_se_grip, cart_duration, cart_samples, time_scale_method)]
    N_list3 = [(N_list2[-1][0], 1) for i in range(grip_action)]
    N_list4 = [(np.around(x,4), 1) for x in mr.CartesianTrajectory(T_se_grip, T_se_pickset, cart_duration, cart_samples, time_scale_method)]
    N_list5 = [(np.around(x,4), 1) for x in mr.ScrewTrajectory(T_se_pickset, T_se_dropset, turn_duration, turn_samples, time_scale_method)]
    N_list6 = [(np.around(x,4), 1) for x in mr.CartesianTrajectory(T_se_dropset, T_se_open, cart_duration, cart_samples, time_scale_method)]
    N_list7 = [(N_list6[-1][0], 0) for i in range(grip_action)]
    N_list8 = [(np.around(x,4), 0) for x in mr.CartesianTrajectory(T_se_open, T_se_dropset, cart_duration, cart_samples, time_scale_method)]
    
    #Sum Trajectories
    rtn_list = N_list1 + N_list2 + N_list3 + N_list4 + N_list5 + N_list6 + N_list7 + N_list8
    return rtn_list


def GetTsb(phi, x, y):
    """
    Return the transformation matrix given the configuration
    q
    """
    T_sb = np.array([[np.cos(phi), -np.sin(phi), 0, x],
                     [np.sin(phi),  np.cos(phi), 0, y],
                     [           0,            0, 1, 0.0963],
                     [           0,            0, 0, 1]])
    return T_sb