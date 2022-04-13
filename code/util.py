#!/usr/bin/env python3
# Code for plotting Xerr
# RKS

# Project import

# Python import
import csv

# 3rd-party
import numpy as np
import matplotlib.pyplot as plt


def PlotXerr(xerr_data, image_file=None):
    """
    Generates a plot of the 6-dimensional xerr_csv_file

    Args:
        np.array(N, 7) xerr_data:
            Datalog of xerr with N entries
        string image_file:
            If not None, location to save the plot
    """
    print("Formatting Data")
    #Format Data
    time_vec = xerr_data[:,0]
    xerr_col1 = xerr_data[:,1]
    xerr_col2 = xerr_data[:,2]
    xerr_col3 = xerr_data[:,3]
    xerr_col4 = xerr_data[:,4]
    xerr_col5 = xerr_data[:,5]
    xerr_col6 = xerr_data[:,6]

    fig, ax = plt.subplots()
    fig.suptitle("YouBot Configuration Error Response")
    ax.set(xlabel="time (seconds)", ylabel="error")
    ax.plot(time_vec, xerr_col1, color='red',label="Xerr_wx")
    ax.plot(time_vec, xerr_col2, color='blue',label="Xerr_wy")
    ax.plot(time_vec, xerr_col3, color='yellow',label="Xerr_wz")
    ax.plot(time_vec, xerr_col4, color='green',label="Xerr_vx")
    ax.plot(time_vec, xerr_col5, color='orange',label="Xerr_vy")
    ax.plot(time_vec, xerr_col6, color='purple',label="Xerr_vz")
    ax.legend()

    print("Plotting")
    if image_file is not None:
        plt.savefig(image_file, bbox_inches='tight')

    plt.show()
