#!/usr/bin/python
# Filename: orb_evaluate.py
# Author:Turtlezhong 2017.07.28 in DM
import matplotlib.pyplot as plt
import numpy as np


data_truth = np.genfromtxt('data_output.txt', usecols=(0, 1))
data_prediction = np.genfromtxt('data_output.txt', usecols=(3, 4))
data_ekf = np.genfromtxt('data_output.txt', usecols=(6, 7))
data_ukf = np.genfromtxt('data_output.txt', usecols=(9, 10))
# show the data
plt.xlabel('x [m]', size=14)
plt.ylabel('y [m]', size=14)
plt.plot(data_truth[:, 0], data_truth[:, 1], color='r', label='Ground Truth')
plt.plot(data_prediction[:, 0], data_prediction[:, 1], color='b', label='Prediction')
plt.plot(data_ekf[:, 0], data_ekf[:, 1], color='g', label='EKF Estimate')
plt.plot(data_ukf[:, 0], data_ukf[:, 1], color='y', label='UKF Estimate')


plt.legend(loc='upper right')
#
# plt.savefig('pose.png', format='png')
plt.show()

# print 'save image suscessfully'



