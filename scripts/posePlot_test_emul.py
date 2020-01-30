#!/usr/bin/env python

import sys
from timeit import default_timer as timer

import matplotlib.pyplot as plt
import numpy as np
import rospy
from mpl_toolkits.mplot3d import Axes3D
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, PoseStamped
from optitrack_broadcast.msg import Mocap


class posePlot():
    def __init__(self):

        self.fig = plt.figure()
        self.ax = self.fig.gca(projection='3d')
        self._topics = {"/uav2/px4_command/visualmeasurement": Odometry,
                        "/mocap/Payload": Mocap,
                        "/mocap/UAV2": Mocap
                        }

        self._sub = {}
        for idx, key in enumerate(self._topics.keys()):
            try:
                self._sub[key] = rospy.Subscriber(
                    key, self._topics[key], self.msgCallback, callback_args=idx, queue_size=1)
            except:
                print("Subscription to {} failed!".format(key))

        self.last = timer()
        self.dcm = np.zeros((len(self._sub), 3, 3))

        # Throttle plotting rate! 10Hz appears to be working fine
        self.update_rate = 10
        plt.show()

    def msgCallback(self, data, args):
        key = args
        if isinstance(data, Odometry):
            q = np.array([[data.pose.pose.orientation.w],
                          [data.pose.pose.orientation.x],
                          [data.pose.pose.orientation.y],
                          [data.pose.pose.orientation.z]])

        elif isinstance(data, Mocap):
            q = data.quaternion

        elif isinstance(data, Pose) or isinstance(data, PoseStamped):
            q = np.array([[data.orientation.w],
                          [data.orientation.x],
                          [data.orientation.y],
                          [data.orientation.z]])

        self.dcm[:][:][key] = self.to_dcm(q).squeeze()
        self.plot()

    def plot(self):
        now = timer()
        if now - self.last >= 1/self.update_rate:
            self.last = now
            self.ax.cla()
            C_p_v = self.dcm[:][:][0]
            if np.count_nonzero(self.dcm[:][:][1]) and np.count_nonzero(self.dcm[:][:][2]):
                C_p_v_t = np.matmul(self.dcm[:][:][1],
                                    self.dcm[:][:][2].transpose())
            self.plotframe(C_p_v, lbl=['p_1', 'p_2', 'p_3'])
            self.plotframe(C_p_v_t, stl=['--c', '--m', '--y'], lbl=[
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
