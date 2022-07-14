#!/usr/bin/python
# -*- coding:utf-8 -*-
from nis import cat
import rospy
import numpy as np
import pandas as pd
from matplotlib import pyplot as plt
from std_msgs.msg import Float64MultiArray
pos_record = np.zeros([5000, 4], dtype=float)
vel_record = np.zeros([5000, 4], dtype=float)
torque_record = np.zeros([5000, 4], dtype=float)
pos_count = 0
vel_count = 0
torque_count = 0


def pos_callback(msg):
    global pos_count
    if pos_count <5000:
        pos_record[pos_count][0] = msg.data[1]
        pos_record[pos_count][1] = msg.data[2]
        pos_record[pos_count][2] = msg.data[4]
        pos_record[pos_count][3] = msg.data[5]
        pos_count = pos_count + 1


def vel_callback(msg):
    global vel_count
    if vel_count <5000:
        vel_record[vel_count][0] = msg.data[1]
        vel_record[vel_count][1] = msg.data[2]
        vel_record[vel_count][2] = msg.data[4]
        vel_record[vel_count][3] = msg.data[5]
        vel_count = vel_count + 1


def torque_callback(msg):
    global torque_count
    if torque_count <5000:
        torque_record[torque_count][0] = msg.data[1]
        torque_record[torque_count][1] = msg.data[2]
        torque_record[torque_count][2] = msg.data[4]
        torque_record[torque_count][3] = msg.data[5]
        torque_count = torque_count + 1


if __name__ == '__main__':
    rospy.init_node("plot_node")
    rospy.Subscriber("/pos_feedback", Float64MultiArray, pos_callback)
    rospy.Subscriber("/vel_feedback", Float64MultiArray, vel_callback)
    rospy.Subscriber("/torque_feedback",Float64MultiArray, torque_callback)
    rospy.spin()
    pos_data = pd.DataFrame(pos_record[0:pos_count,:],columns=[1,2,4,5],index=None)
    vel_data = pd.DataFrame(vel_record[0:vel_count,:],columns=[1,2,4,5],index=None)
    torque_data = pd.DataFrame(torque_record[0:torque_count,:],columns=[1,2,4,5],index=None)
    pos_data.to_csv('/home/holmes/ros/qradruped/src/wtr-serial/scripts/pos.csv')
    vel_data.to_csv('/home/holmes/ros/qradruped/src/wtr-serial/scripts/vel.csv')
    torque_data.to_csv('/home/holmes/ros/qradruped/src/wtr-serial/scripts/torque.csv')
    
