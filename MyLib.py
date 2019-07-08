# -*- coding: utf-8 -*-

""" Aluno: Leonardo Santos Paulucio
    Data: 10/07/19
    Trabalho 3 de Visão Computacional
"""

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D


def createLinearMatrixEquation(cameras, points):

    N = len(points)
    A = np.zeros((3*N, 4))
    b = np.zeros((3*N, 1))

    for i, p in enumerate(points):
        K = cameras[i].getIntrinsicMatrix()
        T = cameras[i].getExtrinsicMatrix()[:-1, -1]
        R = cameras[i].getExtrinsicMatrix()[0:3, 0:3]
        i *= 3
        p.append(1)
        m = np.array(p).reshape((3, 1))

        point = np.dot(K, R)
        point = np.linalg.inv(point)
        point = np.dot(point, m)

        # first line
        A[i, 0:3] = np.array([-1, 0, 0])
        A[i, -1] = point[0]
        # second line
        A[i+1, 0:3] = np.array([0, -1, 0])
        A[i+1, -1] = point[1]
        # third line
        A[i+2, 0:3] = np.array([0, 0, -1])
        A[i+2, -1] = point[2]

        t = np.dot(np.linalg.inv(R), T)
        b[i] = t[0]
        b[i+1] = t[1]
        b[i+2] = t[2]

    return A, b


def recoverPosition3D(cameras, points):

    idx = []

    for i, p in enumerate(points):
        if p is not None:
            idx.append(i)

    cameras = [cameras[i] for i in idx]
    points = [points[i] for i in idx]
    A, b = createLinearMatrixEquation(cameras, points)

    return np.dot(np.linalg.pinv(A), b)[:-1, :]


def plotPoints3D(points, title="3D points"):

    plt.figure()
    ax0 = plt.axes(projection='3d')
    ax0.set_title(title)
    ax0.set_xlabel('x-axis')
    ax0.set_ylabel('y-axis')
    ax0.set_zlabel('z-axis')
    ax0.set_xlim([-2, 2])
    ax0.set_ylim([-1, 1])
    ax0.set_zlim([0, 4])

    ax0.plot3D(points[0, :], points[1, :], points[2, :], 'k.')
    ax0.view_init(elev=25, azim=-65)
    plt.show()
