#!/usr/bin/python

import os, sys
import time

import pythonSB as sb


class ServoSB(object):
  def __init__(self, pin, minAngle=0, maxAngle=180):
    """Pin number, min us output, max us output, min angle, max angle """ 
    self.pin = int(pin)
    self.minUS = 50
    self.maxUS = 250
    self.minAngle = minAngle
    self.maxAngle = maxAngle
    self._configure()

  def _configure(self):
    sb.servo_configure(self.pin, 
          self.minUS, self.maxUS, self.minAngle, self.maxAngle) 

  def set_angle(self, angle):
    sb.servo_set_angle(self.pin, int(angle))

if __name__ == '__main__':
  s7 = ServoSB(7)
  s7.set_angle(7)
  s7b = ServoSB(7, -90, 90)
  s7b.set_angle(0)
