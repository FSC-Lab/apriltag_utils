#!/usr/bin/env python

import sys
from timeit import default_timer as timer

import matplotlib.pyplot as plt
import numpy as np
import rospy
from mpl_toolkits.mplot3d import Axes3D
from nav_msgs.msg import Odometry
from optitrack_broadcast.msg import Mocap


class posePlot():
    def __init__(self):

        self.fig = plt.figure()
        self.ax = self.fig.gca(projection='3d')
        self._sub = {"/uav2/px4_command/visualmeasurement": rospy.Subscriber("/uav2/px4_command/visualmeasurement", Odometry, self.msgCallback, queue_size=1),
                     "/mocap/Payload": rospy.Subscriber("/mocap/Payload", Mocap, self.msgCallback, queue_size=1)}
        self.flag_draw = {"odom": False, "mocap": False}
        self.plotted = False
        self.last = timer()
        self.dcm = np.zeros((2, 3, 3))
        plt.show()

    def msgCallback(self, data):

        if isinstance(data, Odometry):
            q = np.array([[data.pose.pose.orientation.w], [data.pose.pose.orientation.x],
                          [data.pose.pose.orientation.y], [data.pose.pose.orientation.z]])
            self.dcm[:][:][0] = self.to_dcm(q).squeeze()
            self.flag_draw["odom"] = True

        elif isinstance(data, Mocap):
            self.dcm[:][:][1] = self.to_dcm(data.quaternion).squeeze()
            self.flag_draw["mocap"] = True

        now = timer()
        if now - self.last >= 0.1:
            self.last = now
            self.plot()

    def plot(self):
        if self.flag_draw["odom"] and self.flag_draw["mocap"]:
            self.ax.cla()
            self.flag_draw["odom"] = False
            self.flag_draw["mocap"] = False

            self.plotframe(self.dcm[:][:][0], lbl=['p_1', 'p_2', 'p_3'])
            self.plotframe(self.dcm[:][:][1], stl=['--c', '--m', '--y'], lbl=[
                           'p_t_1', 'p_t_2', 'p_t_3'])
            self.ax.set_xlim3d(-1, 1)
            self.ax.set_ylim3d(-1, 1)
            self.ax.set_zlim3d(-1, 1)
            plt.legend()
            self.ax.set_xlabel("X-axis")
            self.ax.set_ylabel("Y-axis")
            self.ax.set_zlabel("Z-axis")

            self.fig.canvas.draw()
            self.fig.canvas.flush_events()
            self.plotted = True

    def plotframe(self, dcm, stl=['r', 'g', 'b'], lbl=['x', 'y', 'z']):
        self.ax.plot([0, dcm[0][0]], [0, dcm[1][0]], [
                     0, dcm[2][0]], stl[0], label=lbl[0])
        self.ax.plot([0, dcm[0][1]], [0, dcm[1][1]], [
                     0, dcm[2][1]], stl[1], label=lbl[1])
        self.ax.plot([0, dcm[0][2]], [0, dcm[1][2]], [
                     0, dcm[2][2]], stl[2], label=lbl[2])

    def main(self):
        try:
            rospy.spin()
        except KeyboardInterrupt:
            print("exiting")
            sys.exit()

    @staticmethod
    def to_dcm(quat):
        _2q0q1 = 2 * quat[0] * quat[1]
        _2q0q2 = 2 * quat[0] * quat[2]
        _2q0q3 = 2 * quat[0] * quat[3]
        _2q1q1 = 2 * quat[1] * quat[1]
        _2q1q2 = 2 * quat[1] * quat[2]
        _2q1q3 = 2 * quat[1] * quat[3]
        _2q2q2 = 2 * quat[2] * quat[2]
        _2q2q3 = 2 * quat[2] * quat[3]
        _2q3q3 = 2 * quat[3] * quat[3]

        dcm = np.array([
            [1 - _2q2q2 - _2q3q3, _2q1q2 + _2q0q3, _2q1q3 - _2q0q2],
            [_2q1q2 - _2q0q3, 1 - _2q1q1 - _2q3q3, _2q2q3 + _2q0q1],
            [_2q1q3 + _2q0q2, _2q2q3 - _2q0q1, 1 - _2q1q1 - _2q2q2]
        ])
        return dcm


if __name__ == '__main__':
    rospy.init_node('posePlot')
    plot = posePlot()
    plot.main()
