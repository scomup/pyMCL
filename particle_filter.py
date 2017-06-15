#!/usr/bin/python
# coding: UTF-8
import random
import numpy as np



class Particle_filter:
    def __init__(self, num=1000):
        self.num = num
        self.particles = []

    def set_init_particles(self,pose,sigma1,sigma2):
        for i in range(self.num):
            x = random.gauss(pose[0],sigma1)
            y = random.gauss(pose[1],sigma1)
            a = random.gauss(pose[2],sigma2)
            if a > np.pi:
                a = a % np.pi
            elif a < -np.pi:
                a = a % -np.pi
            print a
            self.particles.append((x,y,a))

