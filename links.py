#!/usr/bin/python

import frames 
import sympy as sp
import matplotlib.pyplot as plt

class Link(frames.Frame):
    def __init__(self, lid, l, angle):
        self.lid = lid
        self.l = l
        self.angle = str(angle)
        t = "rotz(%s)"%str(angle)
        super(Link, self).__init__(lid, transf=t)
  
    def _post_attach(self, parent):
        if isinstance(parent, Link):
            self.transf = "transl(%s,.0,.0)"%str(parent.l) + "*" + self.transf
            #print "New transf", self.transf
            self._parseTransf()
    
class MultiLink(object):
    def __init__(self):
        self.tt = frames.TransformationTree()
        self.links = []
        self.eff = sp.Matrix([1,0,0,1]) # End effector
        self.H = None
        self.J = None
        self.H_eff = None

    def compose(self, *kargs):
        self.addLink(0, .0) # Add effector reference frame
        self.H = self.tt.compose()
        self.H_eff = self.H*self.eff
        symbols = [frames.SYMB(s) for s in kargs]
        if symbols:
          self.J = self.H_eff.jacobian(symbols)

    def addLink(self, v, angle):
        p = self.tt.root
        if self.links: p=self.links[-1]
        l = Link("l%d"%len(self.links),v, angle)
        l.parent = p
        self.links.append(l)
        return l

    def plotLinks(self):
        ax, _ = self.tt.plot_frames(xl=(-1, 6),yl=(-1,6))
        for i,link in enumerate(self.links):
            self.tt.plotInFrame([link.l, 0, 0, 1], ax,i+1)
        self.tt.plotInFrame(self.eff, ax, i+1)

    def positionToJoints(self, x,y,z):
      """ This is the inverse kinematic """
      a,b,g = .0,.0,.0
      return a,b,g

    def jointsToPosition(self, *kargs, **kwargs):
      """ This is the direct kinematic """
      d = {}
      for k,v in kwargs.items():
        d[frames.SYMB(k)] = v
      return self.H_eff.subs(d).evalf()

    def isReachable(self, x,y,z):
      return True
    
if __name__ == '__main__':
  from sympy import init_printing, pprint
  init_printing() 

  l1 = 1.0; a1 = 45.0
  l2 = 2.0; a2 = -10.0
  l3 = 1.5; a3 = -30.0
  arm = MultiLink()
  link1 = arm.addLink(l1, a1)
  link2 = arm.addLink(l2, a2)
  link3 = arm.addLink(l3, a3)
  arm.compose()

  #arm.plotLinks()
  print (arm.tt)
  
  #plt.savefig("links.png")

  arm2 = MultiLink()
  a2l1 = arm2.addLink("s:L1", "s:alpha")
  a2l2 = arm2.addLink("s:L2", "s:beta")
  arm2.compose("alpha","beta")
  print(arm2.tt)
  print(arm2.H_eff)
  print (arm2.jointsToPosition(alpha=45.0,
      beta=-10, 
      L1=1.0, 
      L2=1.0))
  print (arm2.jointsToPosition(alpha=90.0,
      beta=-10,
      L1=1.0,
      L2=1.0))
  print(arm2.tt)
  print(arm2.H_eff)

  #print arm2.tt
  #print arm2.H_eff 
  Js = arm2.J.copy().subs({
      frames.SYMB("alpha"):45.0, 
      frames.SYMB("beta"):-10, 
      frames.SYMB("L1"):1.0, 
      frames.SYMB("L2"):1.0})
  #print arm2.J
  print (Js.evalf())
  print(arm2.tt)




