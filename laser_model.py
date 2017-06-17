#!/usr/bin/python
# coding: UTF-8

# Laser model

import random
from math import *
import matplotlib.pyplot as plt
import numpy as np

class Laser_model:
    def __init__(self, prob_map, z_hit = 0.95, z_rand = 0.05, sigma_hit = 0.2, range_max = 8.):  
        self.prob_map = prob_map
        self.z_hit = z_hit
        self.z_rand = z_rand
        self.sigma_hit = sigma_hit
        self.z_hit_denom = 2 * self.sigma_hit * self.sigma_hit
        self.z_rand_mult = 1.0/range_max
        self.range_max = float(range_max)

    def R(self, laser_pose):
        R = np.matrix([[cos(laser_pose[2]),-sin(laser_pose[2]), laser_pose[0]],[sin(laser_pose[2]),cos(laser_pose[2]),laser_pose[1]],[0,0,1] ])
        return  R

    def get_probability(self, laser_pose, scan):
        map_scan, world_scan = self.get_scan_in_world_coord(scan, laser_pose)
        p = 0
        for i in range(map_scan.shape[0]):
            z =  self.prob_map.map_lkf[int(map_scan[i,1]),int(map_scan[i,0])]
            pz = self.z_hit * exp(-(z * z) / self.z_hit_denom) + self.z_rand * self.z_rand_mult
            p += pz*pz*pz
        return p
        
    def get_scan_in_world_coord(self, scan, laser_pose):
        R = self.R(laser_pose)
        scan = np.matrix(scan)
        scan_size = scan.shape[0]
        scan_tmp = scan.transpose()
        tmp = np.zeros((1, scan_size))
        tmp.fill(1)
        scan_tmp = np.vstack([scan_tmp, tmp])
        scan_tmp = np.matrix(scan_tmp)
        scan_fix = R*scan_tmp
        scan_fix = np.array(scan_fix[0:2,:]).transpose()
        map_scan = self.prob_map.world_map(scan_fix)
        world_scan = scan_fix.transpose()[:,0:2]
        return  map_scan, world_scan

if __name__ == '__main__':
    pass
