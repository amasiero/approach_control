#!/usr/bin/env python
# -*- coding: utf-8 -*-

import os

import glob   as gl
import pickle as pk
import numpy  as np

#from Region import Region

#==============================================================================
# Constante - Mapa de ponderação espacial
#==============================================================================
W = [   2,2,2,1,1,2,2,2, \
        2,3,3,2,2,3,3,2, \
        2,3,3,2,2,3,3,2, \
        1,1,1,2,2,1,1,1, \
        1,1,1,2,2,1,1,1, \
        0,1,1,1,1,1,1,0, \
        0,1,1,1,1,1,1,0, \
        0,1,1,1,1,1,1,0]

#==============================================================================
# Função para carregar a base
#==============================================================================
def LoadBase():
    return {    os.path.basename(f)[:-4] : pk.load(open(f))[1]
                for f in gl.glob('Base/*.pck') }

#==============================================================================
# ChiSquare distance
#==============================================================================
def ChiSquare(h1, h2, w = 1.0):

    err = np.seterr(all='ignore')
    h   = np.nansum( w * ((h1 - h2) ** 2) / (h1 + h2).astype(float) )

    np.seterr(**err)

    return h

def ChiSum(h1, h2, weight):
    return sum([ChiSquare(i, j, k) for i, j, k in zip(h1, h2, weight)])

#==============================================================================
# Authentify
#==============================================================================
def Authentify( T,    # imagem de teste
                C,    # conjunto de confirmação
                N,    # conjunto de negaçao
                W):   # mapa de ponderaçao espacial

    return ChiSum(T, N, W) - ChiSum(T, C, W)
