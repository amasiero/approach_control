#! /usr/bin/env python
# -*- coding: utf-8 -*-
# importing packages
from __future__ import print_function
from skimage import io

import numpy as np
import os.path

root_dir =  os.path.abspath('database/faces')


def load_fei_db_mean():
    return io.imread(root_dir + '/fei_db_mean.jpg', as_grey = True)

def load_andrey_mean():
    return io.imread(root_dir + '/andrey_mean.jpg', as_grey = True)

def load_female_db():
    return io.imread(root_dir + '/female_db.jpg', as_grey = True)

def load_male_db():
	return io.imread(root_dir + '/male_db.jpg', as_grey = True)
