# -*- coding: utf-8 -*-

""" Aluno: Leonardo Santos Paulucio
    Data: 10/07/19
    Trabalho 3 de Visão Computacional
"""

# Standard libraries
import numpy as np
import json


class Camera:
    """ Camera class """

    def __init__(self, filename=None):
        self.intrinsicMatrix = None
        self.extrinsicMatrix = None
        self.resolution = None
        self.error = None
        self.distortion = None
        self.id = None
        self.videoPATH = None

        if filename is not None:
            self.readCalibrationJSON(filename)

    def getError(self):
        return self.error

    def getID(self):
        return self.id

    def getDistortion(self):
        return self.distortion

    def getResolution(self):
        return self.resolution

    def getIntrinsicMatrix(self):
        return self.intrinsicMatrix

    def getExtrinsicMatrix(self):
        return self.extrinsicMatrix

    def getVideoPATH(self):
        return self.videoPATH

    @staticmethod
    def newProjectionMatrix():

        return np.array([[1, 0, 0, 0],
                         [0, 1, 0, 0],
                         [0, 0, 1, 0]])

    def readCalibrationJSON(self, filename):
        # [] are for JSON arrays, which are called list in Python
        # {} are for JSON objects, which are called dict in Python

        with open(filename) as json_file:
            data = json.load(json_file)
            self.resolution = data['resolution']['width'], data['resolution']['height']
            self.error = data['error']
            self.id = data['id']
            self.videoPATH = "videos/camera-0" + self.id + ".mp4"
            distortion = data['distortion']
            shape = distortion['shape']['dims'][0]['size'], distortion['shape']['dims'][1]['size']
            self.distortion = np.array(distortion['doubles']).reshape(shape)
            intrinsic = data['intrinsic']
            shape = intrinsic['shape']['dims'][0]['size'], intrinsic['shape']['dims'][1]['size']
            self.intrinsicMatrix = np.array(intrinsic['doubles']).reshape(shape)
            extrinsic = data['extrinsic']['tf']
            shape = extrinsic['shape']['dims'][0]['size'], extrinsic['shape']['dims'][1]['size']
            self.extrinsicMatrix = np.linalg.inv(np.array(extrinsic['doubles']).reshape(shape))
