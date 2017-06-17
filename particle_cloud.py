#!/usr/bin/python
# coding: UTF-8
import random
import numpy as np



class Particle_cloud:
    def __init__(self, num=1000):
        self.num = num
        self.particles = []
        self.particles = []
        self.alpha_slow = 0.001
        self.alpha_fast = 0.1
        self.w_slow = 0
        self.w_fast = 0
        self.best_p = [0,0,0]

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
        ps = []
        for p in self.particles:
            p_new = odom_model_update_func(p[0], args[0], args[1])
            ps.append( [p_new,1./self.num] )
        self.particles = ps

    def update_by_laser_model(self, laser_model_prob_func, *args):
        w_tot = 0.
        w_max = 0.
        scan_base = args[0]
        for p in self.particles:
            pose = [scan_base[0] + p[0][0],scan_base[1] + p[0][1],scan_base[2] + p[0][2]]
            w = laser_model_prob_func(pose, args[1]) 
            p[1] = w
            w_tot += w
            if w > w_max:
                w_max = w
                self.best_p = p

        
        if w_tot > 0:
            for p in self.particles:
                p[1] /= w_tot
            w_avg = w_tot/len(self.particles)
            if self.w_slow == 0.0:
              self.w_slow = w_avg
            else:
              self.w_slow += self.alpha_slow * (w_avg - self.w_slow)
            if self.w_fast == 0.0:
              self.w_fast = w_avg
            else:
              self.w_fast += self.alpha_fast * (w_avg - self.w_fast)
        else:
            for p in self.particles:
                p[1] /= 1./len(self.particles)
                
    def update_by_resample(self):  
        resampled = [] 
        w_diff = 1.0 - self.w_fast / self.w_slow  
        if w_diff < 0.0:
            w_diff = 0.0
        ## TODO
        w_diff = 0.0
        c = []
        c.append(0)
        for i in range(self.num):
            c.append(c[i]+self.particles[i][1])
        for count in range(self.num):
            if random.random() < w_diff:
                print "TODO!!!"
                sample = list(self.particles[i])
                resampled.append(sample)
            else:
                r = random.random()
                i = 0
                while i < self.num:
                    if (c[i] <= r) and (r < c[i+1]):
                        break
                    i += 1
                sample = list(self.particles[i])
                sample[1] = 1./self.num
                resampled.append(sample)
        #print "-------"
        #for i in range(self.num):
        #    print i, ':',self.particles[i]
        #print "-------"
        #for i in range(self.num):
        #    print i, ':',resampled[i]
        
        self.particles = resampled
            


                    
                
            
        
    

if __name__ == '__main__':
    from odom_model import Odom_Model
    odom_model = Odom_Model()
    particle_cloud = Particle_cloud(3)
    particle_cloud.set_init_particles((0,0,0),0,0)
    particle_cloud.update_by_odom_model(odom_model.update, (0,1,0),(0,2,0))
    print particle_cloud.particles
