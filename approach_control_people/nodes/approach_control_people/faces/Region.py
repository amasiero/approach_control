#!/usr/bin/env python
# -*- coding: utf-8 -*-

class Region:

    def __init__(self, pName, pIndex, pSource = None):
        self.name   = pName

        self.l      = pIndex.T[0]
        self.c      = pIndex.T[1]

        self.SetMeanWeight(pSource)

    def SetMeanWeight(self, pSource):
        self.w = 1.0 if (pSource is None) else pSource[self.l, self.c].mean()
