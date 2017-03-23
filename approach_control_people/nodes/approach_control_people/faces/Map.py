#!/usr/bin/env python
# -*- coding: utf-8 -*-

import numpy as np
from approach_control_people.Region import Region

class Map:

    def __init__(self, pName, pSource = None, pCluster = None):
        self.name       = pName

        self.source     = pSource
        self.cluster    = pCluster

    def MakeRegularCluster(self, pHeight, pWidth, pLine, pColumn):
        self.cluster = np.zeros((pHeight, pWidth))

        h = pHeight // pLine
        w = pWidth  // pColumn

        for i in range(pLine):
            for j in range(pColumn):
                self.cluster[i*h:(i+1)*h, j*w:(j+1)*w] = (pLine * i) + j

    def MakeRegions(self, pCluster = None):
        if not (pCluster is None): self.cluster = pCluster

        self.regions = [ Region( str(i), np.argwhere(self.cluster == i), self.source ) \
                            for i in np.unique(self.cluster) ]

    def SetWeights(self, pSource = None):
        if not (pSource is None): self.source = pSource

        for r in self.regions:
            r.SetMeanWeight(self.source)

    def SetWeightsByIndex(self, pIndex):
        for r, i in zip(self.regions, pIndex):
            r.w = i

    def GetWeights(self):
        return [r.w for r in self.regions]
