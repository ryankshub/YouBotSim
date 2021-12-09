#!/usr/bin/env python3
# Code for milestone 3
# RKS

# Project import

# Python import

# 3rd-party
import modern_robotics as mr
import numpy as np


def FeedbackControl(X, X_d, X_dnext, kp, ki, Err, pinv_Je, dt):
    """
    Generates wheel and joint velocity using a Feedback and feedforward control

    Args:
        np.array(4,4) X:
            The current configuration of the end effector: known as T_se
        np.array(4,4) X_d:
            The desired end-effector in the trajectory: known as T_se,d
        np.array(4,4) X_dnext:
            The next desired end-effector in the trajectory: known as T_se,d,next
        float kp:
            Proportional gain
        float ki:
            Integral gain
        float Err:
            Current integral error
        np.array(9,6) pinv_Je:
            psuedo-inverse Jacobian
        float dt:
            Time duration
    """
    ### GET VB SECTION
    # Calculate gain matrix
    KP = kp * np.eye(6)
    KI = ki * np.eye(6)

    # Get Adj[X^(-1)Xd]
    X_inv = mr.TransInv(X)
    T_ed = np.matmul(X_inv, X_d)
    Adj_T_ed= mr.Adjoint(T_ed)

    # Get V_d
    Xd_inv = mr.TransInv(X_d)
    T_ddn = np.matmul(Xd_inv, X_dnext)
    se_T_ddn = mr.MatrixLog6(T_ddn)
    V_d = mr.se3ToVec(se_T_ddn)
    V_d = V_d/dt

    # get Xerr
    se_T_ed = mr.MatrixLog6(T_ed)
    Xerr = mr.se3ToVec(se_T_ed)

    # get updated integral error
    Xerr_sum = Err + (Xerr * dt)
    

    # get Vb
    Vb = np.matmul(Adj_T_ed, V_d.T) + np.matmul(KP,Xerr) + np.matmul(KI,Xerr_sum)
    #Return results
    rtn_list = np.matmul(pinv_Je, Vb)
    return np.around(rtn_list, 4), Xerr_sum


def GetPinvJacobian(M, T_b0, Blist, thetalist):
    """
    Calculate and return the psuedo-inverse Jacabian

    np.array(4,4) M:
        Home configuration of the arm
    np.array(4,4) T_b0:
        Transformation matrix from body frame to base of the arm
    np.array(6,5) Blist:
        Screw-axis relative to the body frame
    np.array(5) thetalist:
        Current values of the joint angles
    """
    # Get arm Jacobian
    J_arm = mr.JacobianBody(Blist, thetalist)

    # Get base Jacobian
    # Get the Adj of T_eb
    T_0e = mr.FKinBody(M, Blist, thetalist)
    inv_T_0e = mr.TransInv(T_0e)
    inv_T_b0 = mr.TransInv(T_b0)
    T_eb = np.matmul(inv_T_0e, inv_T_b0)
    Adj_T_eb = mr.Adjoint(T_eb)

    # Get F i.e. pseudo_inv H
    LENGTH = .235
    WIDTH = .15
    WHEEL_RADIUS = 0.0475
    INV_H = np.array([[-1/(LENGTH + WIDTH),  1/(LENGTH + WIDTH),  1/(LENGTH + WIDTH),  -1/(LENGTH + WIDTH)],
                      [  1,  1,  1,  1],
                      [ -1,  1, -1,  1]])
    INV_H = (WHEEL_RADIUS/4)*INV_H

    zero_array = np.zeros(4)
    F_6 = np.vstack((zero_array,
                     zero_array,
                     INV_H,
                     zero_array))
    J_base = np.matmul(Adj_T_eb, F_6)

    # Get endeffector Jacobian
    Je = np.hstack((J_base, J_arm))
    pinv_Je = np.linalg.pinv(Je, 1e-5)
    return pinv_Je


