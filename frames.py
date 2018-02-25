import mpmath as m
import sys
import numpy as np
import sympy as sp
import re

from anytree import Node, RenderTree, PreOrderIter

import matplotlib.pyplot as plt
from IPython.display import display
from cycler import cycler
import matplotlib.colors as colors
import matplotlib.cm as cmx

#### Basic defitions
xaxis = sp.Matrix([1,0,0,1]).T
yaxis = sp.Matrix([0,1,0,1]).T
zaxis = sp.Matrix([0,0,1,1]).T

SYM_TABLE = {}

#### Basic functions
def rad(a):
  return m.radians(a)

def identity():
  T = sp.Matrix(
    [[1, 0, 0, 0],
     [0, 1, 0, 0],
     [0, 0, 1, 0],
     [0, 0, 0, 1]]
     )
  return T

def transl(vx,vy,vz):
  T = sp.Matrix(
    [[1, 0, 0, vx],
     [0, 1, 0, vy],
     [0, 0, 1, vz],
     [0, 0, 0, 1]]
     )
  return T

def rotz(q):
  Tz = sp.Matrix(
    [[sp.cos(rad(q)), -sp.sin(rad(q)), 0, 0],
     [sp.sin(rad(q)), sp.cos(rad(q)), 0, 0],
     [0, 0, 1, 0],
     [0, 0, 0, 1]]
     )
  return Tz
  
def rotx(q):
  Tx = sp.Matrix(
    [[1, 0, 0, 0],
     [sp.cos(rad(q)), -sp.sin(rad(q)), 0, 0],
     [sp.sin(rad(q)), sp.cos(rad(q)), 0, 0],
     [0, 0, 0, 1]]
     )
  return Tx  

def roty(q):
  Ty = sp.Matrix(
    [[sp.cos(rad(q)), 0, sp.sin(rad(q)), 0],
     [0, 1, 0, 0],
     [-sp.sin(rad(q)), sp.cos(rad(q)), 0, 0],
     [0, 0, 0, 1]]
     )
  return Ty  
  
#### Frame manipulations

def toList(s):
  if "*" in s:
    return s.split("*")
  else:
    return [s]

def SYMB(s):
  if s not in SYM_TABLE:
    sym = sp.Symbol(s, real=True)
    SYM_TABLE[s] = sym
  return SYM_TABLE[s]

def _parseArgs(s):
  """ Transforms s:arg in SYMB('arg') """
  return  re.sub(r's:(\w+)', r"SYMB('\1')", s) 

class Frame(Node):
  def __init__(self, name, transf="identity()", **kwargs):
    super(Frame, self).__init__(str(name), **kwargs)
    self.transf = transf
    self._parseTransf()

  def _parseTransf(self):
    ts = toList(self.transf)
    H = identity()
    for t in ts:
      t = eval(_parseArgs(t))
      H *= t
    self.T = H
      
  def _post_detach(self, parent):
    pass
  
  def _post_attach(self, parent):
    pass
      
  def __repr__(self):
    s = self.name
    if self.transf: s += "("+self.transf+")"
    return s
  
#### Transformation tree

def round_expr(expr, num_digits=2):
  return expr.xreplace({n : round(n, num_digits) for n in expr.atoms(sp.Number)})

class TransformationTree(object):
  def __init__(self):
    self.Hs = []
    self.root = Frame("O")
      
  def __str__(self):
    return str(RenderTree(self.root))
  
  def pos(self, v, frame):
    H = identity()
    for node in PreOrderIter(self.root):
      print (node.name)
      display(node.T)
      H *= node.T
      if node == frame:
          break
      #print(node.T.evalf())
    n = H*sp.Matrix(v)
    display(H)
    return n

  def plotInFrame(self, v, ax, frame_id, artist='o-'):
    O_ax = sp.Matrix([0,0,0,1])
    v = sp.Matrix(v)
    NO_ax = self.Hs[frame_id]*O_ax
    Nv = self.Hs[frame_id]*v     
    ax.plot( (NO_ax[0], Nv[0]), (NO_ax[1], Nv[1]), artist, lw=4, mew=5, alpha=0.7)
    return NO_ax, Nv

  def compose(self):
    H = identity()
    for node in PreOrderIter(self.root):
      H *= node.T
      self.Hs.append(H)

    return H
      
  def plot_frames(self, fsize=(9,9), xl=(-2, 5),yl=(-2,5), verbose=False):
    # set up figure
    plt.rc('text', usetex=True)
    plt.rc('font', family='serif')
    fig = plt.figure(figsize=fsize)
    ax = fig.add_subplot(111, aspect='equal', autoscale_on=False,
                         xlim=xl, ylim=yl)
    ax.grid()

    O_ax = sp.Matrix([0,0,0,1])
    x_ax = sp.Matrix([1,0,0,1])
    y_ax = sp.Matrix([0,1,0,1])
    z_ax = sp.Matrix([0,0,1,1])                    

    cmap = plt.cm.jet
    cNorm  = colors.Normalize(vmin=0, vmax=5)

    scalarMap = cmx.ScalarMappable(norm=cNorm,cmap=cmap)

    H = identity()
    frames = []
    for i, node in enumerate(PreOrderIter(self.root)):
      H *= node.T
      self.Hs.append(H)
      new_O = H * O_ax; v0 = map(float,new_O[:-2]) 
      new_x = H * x_ax; vx = map(float,new_x[:-2])
      new_y = H * y_ax; vy = map(float,new_y[:-2])
      new_z = H * z_ax; vz = map(float,new_z[:-2])      
      if verbose:
        print ("***** Frame:",node.name)
        display(round_expr(H,3))
        display(round_expr(new_O, 3))
        display(round_expr(new_x, 3))
        display(round_expr(new_y, 3))
        print ("***** End frame:",node.name)
      print (v0)
      print (type(v0))
      colorVal = scalarMap.to_rgba(i)
      print (ax)
      ax.arrow(v0[0],  #x1
               v0[1],  # y1
               vx[0]-v0[0], # x2 - x1
               vx[1]-v0[1], # y2 - y1
               color=colorVal, head_width=0.05, head_length=0.1)
      ax.text(vx[0],vx[1],r'$X_{%s}$'%node.name)
      ax.arrow(v0[0],  #x1
               v0[1],  # y1
               vy[0]-v0[0], # x2 - x1
               vy[1]-v0[1], # y2 - y1
               color=colorVal, head_width=0.05, head_length=0.1)
      ax.text(vy[0],vy[1],r'$Y_{%s}$'%node.name)
      
      frames.append((v0, vx, vy))

    return ax, frames


if __name__ == "__main__":
  from sympy import init_printing, pprint
  init_printing(use_latex=True) 
    
  tt = TransformationTree()
  A = Frame("A", transf="rotz(20)*transl(1,0,0)", parent=tt.root)
  B = Frame("B", transf="rotz(80)", parent=A)

  print (tt)

  #tt.plot_frames() #  TODO DOESN'T WORK? #plt.savefig("frames.png")

  tt = TransformationTree()
  A = Frame("A", transf="rotz(s:alpha)*transl(s:L1,0,0)", parent=tt.root)
  B = Frame("B", transf="rotz(s:beta)", parent=A)
  print (tt)
  tt_eff = tt.compose()*sp.Matrix([SYMB("L2"),0,0,1])
  print (tt_eff)
  ab = [SYMB("alpha"),SYMB("beta")]
  print (tt_eff.jacobian(ab))
  print (tt_eff)



