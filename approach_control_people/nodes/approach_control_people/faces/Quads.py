#! /usr/bin/env python
# -*- coding: utf-8 -*-
import numpy as np

class Quads(object):

    def __init__(self, points):
        self.__xi = points[0]
        self.__yi = points[1]
        self.__xf = points[2]
        self.__yf = points[3]

    def getArea(self):
        return ((self.__xf - self.__xi) * (self.__yf - self.__yi))

    def getPoints(self):
        return np.array([self.__xi, self.__yi, self.__xf, self.__yf])
