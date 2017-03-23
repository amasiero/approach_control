#!/usr/bin/env python
# -*- coding: utf-8 -*-

from behavior_robot.models import *
import numpy as np

class ULBP:

    def __init__(self,
                 pMap    = None,
                 pKernel = [[128, 64, 32], [1, 0, 16], [2, 4, 8]]):

        self.kernel     = np.array(pKernel)
        self.regions    = pMap.regions if not (pMap is None) else []

        # Limites dos 58 padrões uniformes possíveis, em base decimal.
        self.ubins      = [ -1,   0,   1,   2,   3,   4,   6,   7,   8,  12,
                            14,  15,  16,  24,  28,  30,  31,  32,  48,  56,
                            60,  62,  63,  64,  96, 112, 120, 124, 126, 127,
                           128, 129, 131, 135, 143, 159, 191, 192, 193, 195,
                           199, 207, 223, 224, 225, 227, 231, 239, 240, 241,
                           243, 247, 248, 249, 251, 252, 253, 254, 255, 256]

    def __Uniform__(self, pBinary):
        l = (1,0,0,2,1,0,2,2,1)
        c = (0,0,1,0,1,2,1,2,2)

        return (pBinary[l, c] != pBinary.reshape(-1)).sum() < 4

    def __Windowing__(self, pNeighborhood):
        binary = pNeighborhood >= pNeighborhood[1, 1]

        return (binary * self.kernel).sum() if self.__Uniform__(binary) else -1

    def MakePattern(self, pFace):
        self.pattern = np.ones_like(pFace) * -1

        self.pattern[1:-1, 1:-1] = [[
                    self.__Windowing__( pFace[l - 1:l + 2, c - 1:c + 2])
                    for c in range(1, pFace.shape[1] - 1)]
                    for l in range(1, pFace.shape[0] - 1)]

    def MakeHistogram(self):
        self.histogram =    np.array([ ((1. / r.l.shape[0])
                            * np.histogram(self.pattern[r.l, r.c], bins=self.ubins)[0]).tolist()
                            for r in self.regions ])
