#!/usr/bin/env python3
# Code for milestone 1
# RKS

# Project import

# Python import

# 3rd-party
import numpy as np


def NextState(states, velocities, dt, wheel_vel_lim=None, joint_vel_lims=None):
    """
    Generates the next chassis configuration, wheel angles, and joint angles
    given the previous values and the velocities applies for certain time duration

    Args:
        list[12] states:
            The previous chassis configuration x_b, y_b, phi_b (m, m, rad)
            The four wheel angles w1, w2, w3, w4 (rad)
            the five joint angles j1, j2, j3, j4, j5 (rad)
            Arranged in the following order:
            [phi_b, x_b, y_b, j1, j2, j3, j4, j5, w1, w2, w3, w4,]
        list[9] velocities:
            The wheel and joint velocities (rads/s)
            Arranged in the following order:
            [w1_dot, w2_dot, w3_dot, w4_dot, j1_dot, j2_dot, j3_dot, j4_dot, j5_dot,]
        float dt:
            Time duration velocities are applied
        float wheel_vel_lim: default None
            The velocity limit of the wheels (rad/s)
            A value of None means no limit
        list[5] joint_vel_lims:
            The pose of the end-effector to grab the cube (rad/s)
            Arranged as [j1_dot_lim, j2_dot_lim, j3_dot_lim, j4_dot_lim, j5_dot_lim]
            A value of None means no limits
    """
    #Constant
    LENGTH = .235
    WIDTH = .15
    WHEEL_RADIUS = 0.0475

    # Calculate H
    INV_H = np.array([[-1/(LENGTH + WIDTH),  1/(LENGTH + WIDTH),  1/(LENGTH + WIDTH),  -1/(LENGTH + WIDTH)],
                      [  1,  1,  1,  1],
                      [ -1,  1, -1,  1]])
    INV_H = (WHEEL_RADIUS/4)*INV_H

    # Parse states
    # Chassis
    phi_b, x_b, y_b, = states[0:3]
    # Joints
    j1, j2, j3, j4, j5 = states[3:8]
    # Wheels
    w1, w2, w3, w4 = states[8:]
    

    # Parse velocities
    if joint_vel_lims is None:
        j1_dot, j2_dot, j3_dot, j4_dot, j5_dot = velocities[4:]
    else:
        j1_dot, j2_dot, j3_dot, j4_dot, j5_dot = [limiter(v,l) for v,l in zip(velocities[4:], joint_vel_lims)]

    w1_dot, w2_dot, w3_dot, w4_dot = [limiter(v, wheel_vel_lim) for v in velocities[:4]]
    
    # Calculate new values
    # Wheel Angles
    new_w1 = angle_wrap(w1 + w1_dot*dt)
    new_w2 = angle_wrap(w2 + w2_dot*dt)
    new_w3 = angle_wrap(w3 + w3_dot*dt)
    new_w4 = angle_wrap(w4 + w4_dot*dt)
    # Joint Angles
    new_j1 = angle_wrap(j1 + j1_dot*dt)
    new_j2 = angle_wrap(j2 + j2_dot*dt)
    new_j3 = angle_wrap(j3 + j3_dot*dt)
    new_j4 = angle_wrap(j4 + j4_dot*dt)
    new_j5 = angle_wrap(j5 + j5_dot*dt)

    # Chassis
    angle_diff = np.array([[w1_dot*dt],
                           [w2_dot*dt],
                           [w3_dot*dt],
                           [w4_dot*dt]])
    twist_b = np.matmul(INV_H, angle_diff)

    w_b = twist_b[0][0]
    vx_b = twist_b[1][0]
    vy_b = twist_b[2][0]

    if w_b != 0:
        del_phi = w_b
        del_xb = (vx_b*np.sin(w_b) + vy_b*(np.cos(w_b) - 1))/w_b
        del_yb = (vy_b*np.sin(w_b) + vx_b*(1 - np.cos(w_b)))/w_b
    else:
        del_phi = w_b
        del_xb = vx_b
        del_yb = vy_b

    del_qb = np.array([[del_phi],
                    [del_xb],
                    [del_yb]])
    q_k = np.array([[1, 0, 0],
                    [0, np.cos(phi_b), -np.sin(phi_b)],
                    [0, np.sin(phi_b), np.cos(phi_b)]])

    del_qk = np.matmul(q_k, del_qb)

    new_phi_b = phi_b + del_qk[0][0]
    new_x_b = x_b + del_qk[1][0]
    new_y_b = y_b + del_qk[2][0]

    #Return results
    rtn_list = [new_phi_b, new_x_b, new_y_b, new_j1, new_j2, new_j3, new_j4, new_j5, new_w1, new_w2, new_w3, new_w4]
    rtn_list = np.around(rtn_list, 4)
    return rtn_list


def limiter(value, limit):
    """
    Restrict value in range [-limit, limit]

    Args:
        float value:
            Value to limit
        float limit:
            Range boundary; if None or negative, original value is returned

    Rtns:
        float bounded_value
    """
    if limit is None or limit < 0:
        return value
    if value > limit:
        return limit
    elif value < -limit:
        return -limit
    else:
        return value

def angle_wrap(angle):
    """
    Ensure angle is in range or [-pi, pi]
    """
    if angle > np.pi:
        return angle % np.pi
    elif angle < -np.pi:
        return angle % np.pi - np.pi
    else:
        return angle