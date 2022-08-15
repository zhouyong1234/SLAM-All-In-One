#!/usr/bin/env python
"""
Zhiang Chen
Feb 2021
external calibration of two odometries
"""

import rospy
from nav_msgs.msg import Odometry
import numpy as np
import message_filters
import tf
import random
from geometry_msgs.msg._Pose import Pose
from sklearn.linear_model import RANSACRegressor
from utils import so3_estimation

estimation_state_dict = {'rotation':0, 'translation':1, 'done':2}
data_collecting_list = ['x', 'y', 'z', 'roll', 'pitch', 'yaw']

class Estimator(object):
    def __init__(self):
        # data/samples
        self.P_rr = None  # capital p is pose; little p is position
        self.P_cc = None
        self.samples = []  # samples are constructed by pose or positions or both depending on the estimation algorithm
        self.N = 4

    def get_transforms(self, odom):
        x = odom.pose.pose.position.x
        y = odom.pose.pose.position.y
        z = odom.pose.pose.position.z
        pos = np.asarray((x, y, z, 1))
        x = odom.pose.pose.orientation.x
        y = odom.pose.pose.orientation.y
        z = odom.pose.pose.orientation.z
        w = odom.pose.pose.orientation.w
        qua = (x, y, z, w)
        T = tf.transformations.quaternion_matrix(qua)
        T[:, 3] = pos
        return T

    def calcRotationDiff(self, r1, r2):
        err_matrix = (np.matmul(r1.transpose(),r2) - np.matmul(r1,r2.transpose()))/2.
        x3 = err_matrix[1, 0]
        x2 = err_matrix[0, 2]
        x1 = err_matrix[2, 1]
        #print(x1, x2, x3)
        return abs(x1) + abs(x2) + abs(x3)

    def add_N(self, n):
        self.N = self.N + n

class RotEstimator(Estimator):
    def __init__(self):
        Estimator.__init__(self)
        self.distance_threshold = 0.1
        self.distance_min = 0.1
        self.distance_max = 0.75
        self.tolerance_angle = 15 / 180. * np.pi
        self.N_x = 0
        self.N_y = 0
        self.N_z = 0

    def has_sufficent_sample(self):
        if (self.N_x < self.N) or (self.N_y < self.N) or (self.N_z < self.N):
            return False
        else:
            return True

    def add_sample(self, gps_odom, vio_odom):
        T_gps = self.get_transforms(gps_odom)
        T_vio = self.get_transforms(vio_odom)

        if self.P_cc is None:
            self.P_cc = T_vio
            self.P_rr = T_gps
            return

        # insert key sample
        if not self.insert_sample(T_vio, T_gps):
            return

        if self.N_x < self.N:
            print("Next translation goal: " + str(self.distance_threshold) + " on x direction")
            return
        if self.N_y < self.N:
            print("Next translation goal: " + str(self.distance_threshold) + " on y direction")
            return
        if self.N_z < self.N:
            print("Next translation goal: " + str(self.distance_threshold) + " on z direction")
            return

    def estimate_params(self):
        # get ransac data
        Y = []
        X = []
        delta_vio_pos = np.array([sample[0, :] for sample in self.samples])
        delta_gps_pos = np.array([sample[1, :] for sample in self.samples])
        N = delta_gps_pos.shape[0]
        for i in range(N):
            delta_vio = delta_vio_pos[i]
            delta_gps = delta_gps_pos[i]
            x1 = np.array((delta_vio[0], delta_vio[1], delta_vio[2], 0, 0, 0, 0, 0, 0, 1, 0, 0))
            y1 = delta_gps[0]
            x2 = np.array((0, 0, 0, delta_vio[0], delta_vio[1], delta_vio[2], 0, 0, 0, 0, 1, 0))
            y2 = delta_gps[1]
            x3 = np.array((0, 0, 0, 0, 0, 0, delta_vio[0], delta_vio[1], delta_vio[2], 0, 0, 1))
            y3 = delta_gps[2]
            X.append(x1)
            X.append(x2)
            X.append(x3)
            Y.append(y1)
            Y.append(y2)
            Y.append(y3)

        X = np.asarray(X)
        Y = np.asarray(Y)
        print("equation number: ")
        print("X: " + str(self.N_x))
        print("Y: " + str(self.N_y))
        print("Z: " + str(self.N_z))
        # ransac
        # https://scikit-learn.org/stable/modules/generated/sklearn.linear_model.RANSACRegressor.html
        X = X[:, :-3]
        reg = RANSACRegressor(random_state=0).fit(X, Y)
        params = reg.estimator_.coef_
        inlier_mask = reg.inlier_mask_
        R = params.reshape((3, 3))
        #R = so3_estimation.Rot_Estimate_Frobenius_norm(R)
        R = so3_estimation.Rot_Estimate_SVD_SO3(R)
        return R

    def insert_sample(self, T_vio, T_gps):
        previous_vio_pos = self.P_cc[:3, 3]
        current_vio_pos = T_vio[:3, 3]
        p1_distance = np.abs(current_vio_pos - previous_vio_pos).max()
        if p1_distance < self.distance_threshold:
            return False
        else:
            previous_rot = self.P_cc[:3,:3]
            current_rot = T_vio[:3,:3]
            angle = self.calcRotationDiff(previous_rot, current_rot)
            if angle <= self.tolerance_angle:
                # append sample
                delta_vio_pos = current_vio_pos - previous_vio_pos
                previous_gps_pos = self.P_rr[:3, 3]
                current_gps_pos = T_gps[:3, 3]
                delta_gps_pos = current_gps_pos - previous_gps_pos
                dim_idx = np.abs(delta_vio_pos).argmax()
                if dim_idx == 0:
                    self.N_x += 1
                    print("Got sample on x direction")
                elif dim_idx == 1:
                    self.N_y += 1
                    print("Got sample on y direction")
                elif dim_idx == 2:
                    self.N_z += 1
                    print("Got sample on z direction")
                self.samples.append(np.array((previous_vio_pos, previous_gps_pos)))
                # reset self.distance_threshold
                self.distance_threshold = random.random() * (self.distance_max - self.distance_min) + self.distance_min
                # update current pose
                self.P_cc = T_vio
                self.P_rr = T_gps
                return True
            else:
                return False

class TransEstimator(Estimator):
    def __init__(self):
        Estimator.__init__(self)
        self.rot_min = 15 / 180. * np.pi
        self.rot_max = 70 / 180. * np.pi
        self.rot_angle_threshold = 15 / 180. * np.pi  # self.rot_angle_threshold is bounded by self.rot_min and self.rot_max
        self.N_roll = 0
        self.N_pitch = 0
        self.N_yaw = 0
        self.R = np.eye(3)

    def set_R(self, R):
        self.R = R

    def has_sufficent_sample(self):
        if (self.N_roll < self.N) or (self.N_pitch < self.N) or (self.N_yaw < self.N):
            return False
        else:
            return True

    def add_sample(self, gps_odom, vio_odom):
        T_gps = self.get_transforms(gps_odom)
        T_vio = self.get_transforms(vio_odom)

        if self.P_cc is None:
            self.P_cc = T_vio
            self.P_rr = T_gps
            return

        # insert key sample
        if not self.insert_sample(T_vio, T_gps):
            return

        if self.N_roll < self.N:
            print("Next rotation goal: " + str(self.rot_angle_threshold) + " on roll")
            return
        if self.N_pitch < self.N:
            print("Next rotation goal: " + str(self.rot_angle_threshold) + " on pitch")
            return
        if self.N_yaw < self.N:
            print("Next rotation goal: " + str(self.rot_angle_threshold) + " on yaw")
            return

    def estimate_params(self):
        N = len(self.samples)
        X = []
        Y = []
        for i in range(N):
            T_vio, T_gps = self.samples[i]
            P_rr = T_gps
            p_rr = T_gps[:, 3]
            p_cc = T_vio[:, 3]
            A, r = self.computeTanslationSystemParam(P_rr, p_rr, p_cc)
            for j in range(3):
                X.append(A[j, :])
                Y.append(r[j])
            """
            inv_P_rr = np.linalg.inv(P_rr)
            trans_vec = (0.1, 0, -0.01)
            trans = tf.transformations.translation_matrix(trans_vec)
            rot = tf.transformations.euler_matrix(0.5, 1.0471975512, -0.5)
            T = np.matmul(trans, rot)
            T_gt = np.linalg.inv(T)
            p_rc = np.matmul(T_gt, p_cc)
            trans_vec = T_gt[:3, 3]
            T0 = tf.transformations.translation_matrix(trans_vec)
            p_const = np.matmul(inv_P_rr, p_rc) - np.matmul(inv_P_rr, p_rr)
            """

        X = np.asarray(X)
        Y = np.asarray(Y)
        print("equation number: ")
        print("roll: " + str(self.N_roll))
        print("pitch: " + str(self.N_pitch))
        print("yaw: " + str(self.N_yaw))
        reg = RANSACRegressor(random_state=0).fit(X, Y)
        params = reg.estimator_.coef_
        return params

    def computeTanslationSystemParam(self, P_rr, p_rr, p_cc):
        # check out wiki for the translation system: https://github.com/ZhiangChen/gps_vio/wiki/T265-External-Calibration#2-estimating-translation-matrix
        inv_P_rr = np.linalg.inv(P_rr)
        a11 = P_rr[0, 0]
        a12 = P_rr[0, 1]
        a13 = P_rr[0, 2]
        a21 = P_rr[1, 0]
        a22 = P_rr[1, 1]
        a23 = P_rr[1, 2]
        a31 = P_rr[2, 0]
        a32 = P_rr[2, 1]
        a33 = P_rr[2, 2]
        b1 = P_rr[0, 3]
        b2 = P_rr[1, 3]
        b3 = P_rr[2, 3]
        a11_ = inv_P_rr[0, 0]
        a12_ = inv_P_rr[0, 1]
        a13_ = inv_P_rr[0, 2]
        a21_ = inv_P_rr[1, 0]
        a22_ = inv_P_rr[1, 1]
        a23_ = inv_P_rr[1, 2]
        a31_ = inv_P_rr[2, 0]
        a32_ = inv_P_rr[2, 1]
        a33_ = inv_P_rr[2, 2]
        b1_ = inv_P_rr[0, 3]
        b2_ = inv_P_rr[1, 3]
        b3_ = inv_P_rr[2, 3]
        r11 = self.R[0, 0]
        r12 = self.R[0, 1]
        r13 = self.R[0, 2]
        r21 = self.R[1, 0]
        r22 = self.R[1, 1]
        r23 = self.R[1, 2]
        r31 = self.R[2, 0]
        r32 = self.R[2, 1]
        r33 = self.R[2, 2]
        x_cc = p_cc[0]
        y_cc = p_cc[1]
        z_cc = p_cc[2]
        x_rr = p_rr[0]
        y_rr = p_rr[1]
        z_rr = p_rr[2]

        left_r1 = a11 * b1_ + a12 * b2_ + a13 * b3_ + b1 + x_rr * (a11 * a11_ + a12 * a21_ + a13 * a31_) + y_rr * (
                    a11 * a12_ + a12 * a22_ + a13 * a32_) + z_rr * (a11 * a13_ + a12 * a23_ + a13 * a33_)
        left_r2 = a21 * b1_ + a22 * b2_ + a23 * b3_ + b2 + x_rr * (a11_ * a21 + a21_ * a22 + a23 * a31_) + y_rr * (
                    a12_ * a21 + a22 * a22_ + a23 * a32_) + z_rr * (a13_ * a21 + a22 * a23_ + a23 * a33_)
        left_r3 = a31 * b1_ + a32 * b2_ + a33 * b3_ + b3 + x_rr * (a11_ * a31 + a21_ * a32 + a31_ * a33) + y_rr * (
                    a12_ * a31 + a22_ * a32 + a32_ * a33) + z_rr * (a13_ * a31 + a23_ * a32 + a33 * a33_)
        right_r1 = r11 * x_cc + r12 * y_cc + r13 * z_cc
        right_r2 = r21 * x_cc + r22 * y_cc + r23 * z_cc
        right_r3 = r31 * x_cc + r32 * y_cc + r33 * z_cc

        r1 = right_r1 - left_r1
        r2 = right_r2 - left_r2
        r3 = right_r3 - left_r3

        r = np.array((r1, r2, r3))
        A = np.array([[a11 - 1, a12, a13], [a21, a22 - 1, a23], [a31, a32, a33 - 1]])
        return A, r

    def insert_sample(self, T_vio, T_gps):
        current_vio_rot = T_vio[:3, :3]
        previous_vio_rot = self.P_cc[:3, :3]
        angle = self.calcRotationDiff(previous_vio_rot, current_vio_rot)
        if angle < self.rot_angle_threshold:
            return False
        else:
            # append sample
            previous_vio_euler = tf.transformations.euler_from_matrix(previous_vio_rot)
            previous_vio_euler = np.array(previous_vio_euler)
            current_vio_euler = tf.transformations.euler_from_matrix(current_vio_rot)
            current_vio_euler = np.array(current_vio_euler)
            dim_idx = np.abs(current_vio_euler - previous_vio_euler).argmax()
            if dim_idx == 0:
                self.N_roll += 1
                print("Got sample on roll")
            elif dim_idx == 1:
                self.N_pitch += 1
                print("Got sample on pitch")
            elif dim_idx == 2:
                self.N_yaw += 1
                print("Got sample on yaw")
            self.samples.append((T_vio, T_gps))
            # reset self.rot_angle_threshold
            r = (random.random() + self.rot_min / np.pi) / (1. + self.rot_min / np.pi + self.rot_max / np.pi)
            self.rot_angle_threshold = r * np.pi / 2
            # update current pose
            self.P_cc = T_vio
            self.P_rr = T_gps
            return True


class Calibrator(object):
    """
    odometry external calibration interface
    """
    def __init__(self):
        self.estimated_rot = np.eye(3)
        self.estimated_trans = np.zeros(3)
        self.rot_estimator = RotEstimator()
        self.trans_estimator = TransEstimator()
        self.current_estimation = estimation_state_dict['rotation']
        self.sub_gps = message_filters.Subscriber('/gps/odom', Odometry)
        self.sub_vio = message_filters.Subscriber('/vio/odom', Odometry)
        self.ts = message_filters.ApproximateTimeSynchronizer([self.sub_gps, self.sub_vio], queue_size=10, slop=0.02)
        self.ts.registerCallback(self.callback)
        rospy.loginfo("calibrator has been intialized.")
        raw_input("Set Initial Pose. \nPress Enter to continue...")

    def callback(self, gps_data, vio_data):
        if self.current_estimation == estimation_state_dict['rotation']:
            if not self.rot_estimator.has_sufficent_sample():
                self.rot_estimator.add_sample(gps_data, vio_data)
            else:
                self.estimated_rot = self.rot_estimator.estimate_params()
                self.trans_estimator.set_R(self.estimated_rot)
                self.current_estimation = estimation_state_dict['translation']
                raw_input("Rotation Estimation Done. \nPress Enter to continue...")

        elif self.current_estimation == estimation_state_dict['translation']:
            if not self.trans_estimator.has_sufficent_sample():
                self.trans_estimator.add_sample(gps_data, vio_data)
            else:
                self.estimated_trans = self.trans_estimator.estimate_params()
                # display the estimated transformation params
                estimated_T = np.zeros((4, 4))
                #print(self.estimated_trans)
                #print(self.estimated_rot)
                estimated_T[:3, :3] = self.estimated_rot
                estimated_T[:3, 3] = self.estimated_trans
                print("Estimated transform: ")
                print(estimated_T)
                # prompt option of adding more samples to refine estimation
                s = raw_input("Rotation Estimation Done. \nPress 1 to add one more group of samples\nPress any other to stop...")
                if int(s) == 1:
                    self.rot_estimator.add_N(4)
                    self.trans_estimator.add_N(4)
                    self.current_estimation = estimation_state_dict['translation']
                else:
                    self.current_estimation = estimation_state_dict['done']

if __name__ == '__main__':
    rospy.init_node('odom_calibrator', anonymous=False)
    calib = Calibrator()
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Node killed!")