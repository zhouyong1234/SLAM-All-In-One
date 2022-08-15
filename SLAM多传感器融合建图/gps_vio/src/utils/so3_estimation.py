#!/usr/bin/env python
"""
Zhiang Chen
Feb 2021
estimate so3 rotation matrix from a 3x3 projection matrix
"""

import numpy as np

def Rot_Estimate_SVD_SO3(R):
    U, S, V = np.linalg.svd(R)
    return np.matmul(U, V)

def Rot_Estimate_Frobenius_norm(R):
    M = R[:, :2]
    U, S, V = np.linalg.svd(M)
    S_ = np.array([[1, 0], [0, 1], [0, 0]])
    R_ = np.matmul(np.matmul(U, S_), V)
    r1 = R_[:, 0]
    r2 = R_[:, 1]
    r3 = np.cross(r1, r2)
    Rot = np.zeros((3, 3))
    Rot[:, :2] = R_
    Rot[:, 2] = r3
    if np.linalg.det(Rot) > 0:
        return Rot
    else:
        Rot[:, 2] = -r3
        return Rot

def Frobenius_norm(M, R):
    return np.trace(np.matmul((M-R).transpose(), (M-R)))
