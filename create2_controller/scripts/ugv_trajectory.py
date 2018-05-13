#!/usr/bin/env python
import numpy as np

def normalize(v):
  norm = np.linalg.norm(v)
  assert norm > 0
  return v / norm


class Polynomial:
  def __init__(self, p):
    self.p = p

  def stretchtime(self, factor):
    recip = 1.0 / factor;
    scale = recip
    for i in range(1, len(self.p)):
      self.p[i] *= scale
      scale *= recip

  def shift(self, delta):
    self.p[0] += delta

  # evaluate a polynomial using horner's rule
  def eval(self, t):
    assert t >= 0
    x = 0.0
    for i in range(0, len(self.p)):
      x = x * t + self.p[len(self.p) - 1 - i]
    return x

  # compute and return derivative
  def derivative(self):
    return Polynomial([(i+1) * self.p[i+1] for i in range(0, len(self.p) - 1)])


class TrajectoryOutput:
  def __init__(self):
    self.pos = None   # position [m]
    self.theta = None # angle [rad]
    self.vel = None   # velocity [m/s]
    self.omega = None # angular velocity [rad/s]
    self.acc = None   # acceleration [m/s^2]

# 4d single polynomial piece for x-y-z-yaw, includes duration.
class Polynomial2D:
  def __init__(self, duration, px, py):
    self.duration = duration
    self.px = Polynomial(px)
    self.py = Polynomial(py)

  # compute and return derivative
  def derivative(self):
    return Polynomial2D(
      self.duration,
      self.px.derivative().p,
      self.py.derivative().p)

  def stretchtime(self, factor):
    self.duration *= factor
    self.px.stretchtime(factor)
    self.py.stretchtime(factor)

  def shift(self, dx, dy):
    self.px.shift(dx)
    self.py.shift(dy)

  # see A Stable Tracking Control Method for an Autonomous Mobile Robot
  #     http://ieeexplore.ieee.org/document/126006/
  def eval(self, t):
    result = TrajectoryOutput()
    # flat variables
    result.pos = np.array([self.px.eval(t), self.py.eval(t)])

    # 1st derivative
    derivative = self.derivative()
    result.vel = np.array([derivative.px.eval(t), derivative.py.eval(t)])
    result.theta = np.arctan2(result.vel[1], result.vel[0])

    # 2nd derivative
    derivative2 = derivative.derivative()
    result.acc = np.array([derivative2.px.eval(t), derivative2.py.eval(t)])

    velNormSquared = (result.vel**2).sum()
    if velNormSquared > 1e-6:
      result.omega = (result.vel[1] * result.acc[0] - result.vel[0] * result.acc[1]) / velNormSquared
    else:
      result.omega = 0

    return result


class Trajectory:
  def __init__(self):
    self.polynomials = None
    self.duration = None

  def loadcsv(self, filename):
    data = np.loadtxt(filename, delimiter=",", skiprows=1, usecols=range(33))
    self.polynomials = [Polynomial2D(row[0], row[1:9], row[9:17]) for row in data]
    self.duration = np.sum(data[:,0])

  def stretchtime(self, factor):
    for p in self.polynomials:
      p.stretchtime(factor)
    self.duration *= factor

  def shift(self, dx, dy):
    for p in self.polynomials:
      p.shift(dx, dy)

  def eval(self, t):
    assert t >= 0
    assert t <= self.duration

    current_t = 0.0
    for p in self.polynomials:
      if t < current_t + p.duration:
        return p.eval(t - current_t)
      current_t = current_t + p.duration
