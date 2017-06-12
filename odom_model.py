#!/usr/bin/python
# coding: UTF-8

import random
from math import *
import matplotlib.pyplot as plt
import numpy as np


def R(angle):
    R = np.matrix([[cos(angle),-sin(angle)],[sin(angle),cos(angle)]])
    return  R

class Odom_Model:
    def __init__(self, A1 = 0.02, A2 = 0.02, A3 = 0.02, A4 = 0.02):  
        self.A1 = A1 #Weight of rotation error resulting from rotation
        self.A2 = A2 #Weight of rotation error resulting from translation
        self.A3 = A3 #Weight of translation error resulting from translation
        self.A4 = A4 #Weight of translation error resulting from rotation

    def sample(self, sigma):
        while True:
            r = random.random()
            x1 = 2.0 * r - 1.0
            r = random.random()
            x2 = 2.0 * r - 1.0
            w = x1*x1 + x2*x2
            if not (w > 1.0 or w==0.0):
                break
        return sigma * x2 * sqrt(-2.0*log(w)/w) 

    def update(self, pose_pre, pose_now):
        # Probabilistic Robotics (Table 5.6)
        # https://docs.ufpr.br/~danielsantos/ProbabilisticRobotics.pdf
        A1 = self.A1
        A2 = self.A2
        A3 = self.A3
        A4 = self.A4
        rot1 = atan2(pose_now[1] - pose_pre[1], pose_now[0] - pose_pre[0]) - pose_pre[2]
        trans = sqrt((pose_now[1] - pose_pre[1])**2 + (pose_now[0] - pose_pre[0])**2)
        rot2 = pose_now[2] - pose_pre[2] - rot1
        rot1_hat = rot1-self.sample(A1*rot1 + A2*trans)
        trans_hat = trans-self.sample(A3*trans + A4*(rot1 + rot2))
        rot2_hat = rot2-self.sample(A1*rot2 + A2*trans)
        x_ = pose_pre[0] + trans_hat * cos( pose_pre[2] + rot1_hat)
        y_ = pose_pre[1] + trans_hat * sin( pose_pre[2] + rot1_hat)
        theta_ = pose_pre[2] + rot1_hat + rot2_hat
        pose_now_pre = [x_, y_, theta_]
        return pose_now_pre



pose_1 = [0.,0.,0.3]
pose_2 = [1,1.,1.]
pose_3 = [1,2.,1.5]
pose_4 = [0.5,2.5,1.8]

poses = []
poses.append(pose_1)
poses.append(pose_2)
poses.append(pose_3)
poses.append(pose_4)

x = []
y = []

#odom_model = Odom_Model(0.02, 0.0, 0.002, 0.0)
odom_model = Odom_Model(0.1, 0.0, 0.05, 0.0)
particles = []
for i in range(1000):
    particles.append(pose_1)

plt.figure()
plt.xlim((-1,4))
plt.ylim((-1,4))

for pose in poses:
    print pose
    for i in range(1000):
        pose_now_pre = odom_model.update(particles[i], pose)
        x.append(pose_now_pre[0])
        y.append(pose_now_pre[1])
    plt.scatter(x,y,s=0.1, c= 'b')
    v = np.matrix([[1.],[0.]])
    u=R(pose[2])*v
    plt.quiver(pose[0], pose[1], 2*u[0,0], 2*u[1,0], scale=30)
plt.show()

#plt.figure()
#plt.xlim((-1,4))
#plt.ylim((-1,4))
#
#v = np.matrix([[1.],[0.]])
#u=R(pose_pre[2])*v
#Q1 = plt.quiver(pose_pre[0], pose_pre[1], u[0,0], u[1,0])
#u=R(pose_now[2])*v
#Q1 = plt.quiver(pose_now[0], pose_now[1], u[0,0], u[1,0])
#plt.plot( [pose_now[0],pose_pre[0]], [pose_now[1],pose_pre[1]],c = 'r')
#plt.scatter(x,y,s=1)
#plt.show()
#print rot1,trans,rot2