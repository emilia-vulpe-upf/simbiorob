#!/usr/bin/python

import os, sys
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import sympy as sp
from transforms3d._gohlketransforms import identity_matrix

np.set_printoptions(precision=3, suppress=True)  # neat printing

origin, xaxis, yaxis, zaxis = [0, 0, 0], [1, 0, 0], [0, 1, 0], [0, 0, 1]
I = identity_matrix()

def getTv(vx,vy,vz):
  T = sp.Matrix(
    [[1, 0, 0, vx,],
     [0, 1, 0, vy],
     [0, 0, 1, vz],
     [0, 0, 0, 1]]
     )
  return T

def getRz(q):
  Tz = sp.Matrix(
    [[sp.cos(q), -sp.sin(q), 0, 0,],
     [sp.sin(q), sp.cos(q), 0, 0],
     [0, 0, 1, 0],
     [0, 0, 0, 1]]
     )
  return Tz
  
def getRx(q):
  Tx = sp.Matrix(
    [[1, 0, 0, 0],
     [sp.cos(q), -sp.sin(q), 0, 0,],
     [sp.sin(q), sp.cos(q), 0, 0],
     [0, 0, 0, 1]]
     )
  return Tx  

def getRy(q):
  Ty = sp.Matrix(
    [[sp.cos(q), 0, sp.sin(q), 0,],
     [0, 1, 0, 0],
     [-sp.sin(q), sp.cos(q), 0, 0],
     [0, 0, 0, 1]]
     )
  return Ty  

if __name__ == '__main__':
  T1 = getTv(1,0,0)
  Rza = getRz(sp.Symbol("alpha"))
  sp.pprint(T1*Rza)

