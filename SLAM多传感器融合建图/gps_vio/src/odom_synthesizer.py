#!/usr/bin/env python
"""
Zhiang Chen
Feb 2021
symthesize a uav odom from realsense
"""

import rospy
from nav_msgs.msg import Odometry
import numpy as np
import tf

class OdomSynthesizer(object):
    def __init__(self):
        trans_vec = (0.1, 0, -0.01)
        trans = tf.transformations.translation_matrix(trans_vec)
        rot = tf.transformations.euler_matrix(0.5, 1.0471975512, -0.5)
        self.T_robot2vio = np.matmul(trans, rot)
        print("Ground truth T_vio2robot: ")
        self.T_vio2robot = np.linalg.inv(self.T_robot2vio)
        print(self.T_vio2robot)

        self.sub_vio = rospy.Subscriber('/vio/odom', Odometry, self.vioCallback, queue_size=1)
        self.pub_vio = rospy.Publisher('/gps/odom', Odometry, queue_size=1)
        print("odom_synthesizer has been initialized.")

    def vioCallback(self, vio_odom):
        x = vio_odom.pose.pose.position.x
        y = vio_odom.pose.pose.position.y
        z = vio_odom.pose.pose.position.z
        pos = (x, y, z)
        x = vio_odom.pose.pose.orientation.x
        y = vio_odom.pose.pose.orientation.y
        z = vio_odom.pose.pose.orientation.z
        w = vio_odom.pose.pose.orientation.w
        qua = (x, y, z, w)
        trans = tf.transformations.translation_matrix(pos)
        rot = tf.transformations.quaternion_matrix(qua)
        p_cc = np.matmul(trans, rot)  # camera's pose in vio coordinates

        p_rr = np.matmul(self.T_vio2robot, np.matmul(p_cc, self.T_robot2vio))  # uav's pose in robot coordinates

        tf_pos = tf.transformations.translation_from_matrix(p_rr)
        tf_qua = tf.transformations.quaternion_from_matrix(p_rr)

        tf_odom = Odometry()
        tf_odom.header = vio_odom.header
        tf_odom.pose.pose.position.x = tf_pos[0]
        tf_odom.pose.pose.position.y = tf_pos[1]
        tf_odom.pose.pose.position.z = tf_pos[2]
        tf_odom.pose.pose.orientation.x = tf_qua[0]
        tf_odom.pose.pose.orientation.y = tf_qua[1]
        tf_odom.pose.pose.orientation.z = tf_qua[2]
        tf_odom.pose.pose.orientation.w = tf_qua[3]
        self.pub_vio.publish(tf_odom)


if __name__ == '__main__':
    rospy.init_node('odom_synthesizer', anonymous=False)
    vio_Transformer = OdomSynthesizer()

    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Node killed!")