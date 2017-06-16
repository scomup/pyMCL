#!/usr/bin/python
# coding: UTF-8
import random
import numpy as np



class Particle_cloud:
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
            self.particles.append([[x,y,a],1./self.num])

    def update_by_odom_model(self, odom_model_update_func, *args):
        for p in self.particles:
            odom_model_update_func(p[0], args[0], args[1])

if __name__ == '__main__':
    from odom_model import Odom_Model
    odom_model = Odom_Model()
    particle_cloud = Particle_cloud(10)
    particle_cloud.set_init_particles((0,0,0),0,0)
    particle_cloud.update_by_odom_model(odom_model.update, (0,1,0))
    print particle_cloud.particles
