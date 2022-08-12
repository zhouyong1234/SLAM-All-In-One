#!/usr/bin/python
# -*- coding: utf-8 -*
import sys
import os
import re
import numpy as np
import matplotlib
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
inputFile = sys.argv[1]
inputFile1 = sys.argv[2]
### read pos
position = []
quaterntions = []
timestamp = []
tx_index = 1
position = np.loadtxt(inputFile, usecols = (tx_index, tx_index + 1, tx_index + 2))

position1 = []
quaterntions1 = []
timestamp1 = []
position1 = np.loadtxt(inputFile1, usecols = (tx_index, tx_index + 1, tx_index + 2))

### plot 3d
fig = plt.figure()
ax = fig.gca(projection='3d')
ax.plot(position[:,0], position[:,1], position[:,2],'b',label='gt')
ax.plot(position1[:,0], position1[:,1], position1[:,2],'r-',label='drck')

ax.legend()
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')
plt.show()