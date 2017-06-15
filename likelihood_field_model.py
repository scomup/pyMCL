#!/usr/bin/python
# coding: UTF-8

# Likelihood field model
# Probabilistic Robotics (pp. 319)
# https://docs.ufpr.br/~danielsantos/ProbabilisticRobotics.pdf

from PIL import Image
import numpy as np
from math import *

class Prob_map:
    def __init__(self, max_dist = 0.8, scale = 0.025, fre_thr = 200 , occ_thr = 20):
        self.max_dist = max_dist
        self.scale = scale
        self.fre_thr = fre_thr
        self.occ_thr = occ_thr
        self.size_x = 0
        self.size_y = 0

    def read_img(self, img_name):
        image = Image.open(img_name)
        self.map_raw = np.asarray(image)
        self.map_raw = np.flip(self.map_raw, axis = 0)
        self.map_occ = 1*(self.map_raw < self.occ_thr) - 1*(self.map_raw > self.fre_thr)
        self.size_x = self.map_raw.shape[1]
        self.size_y = self.map_raw.shape[0]

    def cal_dist(self, dx, dy):
        dist = sqrt( dx**2 + dy**2 ) * self.scale
        if dist > self.max_dist:
            return self.max_dist
        else:
            return dist

    def set_dist(self, map_mak, mak_lst, map_src, x , y, x_src, y_src):
        if map_mak[y, x]:
            return
        map_mak[y, x] = True
        self.map_lkf[y, x] =  self.cal_dist(x - x_src, y - y_src)
        map_src[y, x] =  [x_src, y_src]
        mak_lst.append((x,y))

    def create_likelihood(self):
        """
        Just for validate the implementation in AMCL
        May have more elegant way ...
        """
        # For make the pocessed cell
        map_mak = (self.map_occ == 1) + (self.map_occ == 0)
        # The pocesses list
        tmp = np.array(np.where(self.map_occ == 1))
        mak_lst = [ (tmp[1][i],tmp[0][i]) for i in range(tmp[0].size)]
        # For record the nearest occupied cell
        assert self.size_x ==  self.size_y, "May error!"
        x, y = np.meshgrid(np.arange(0, self.size_x), np.arange(0, self.size_y))
        map_src = np.dstack((x,y))
        # Store the likelihood field
        self.map_lkf = np.zeros(self.map_raw.shape)
        self.map_lkf = (self.map_occ == 0)*self.max_dist

        while len(mak_lst)>0:
            m = mak_lst.pop(0)
            src = map_src[ m[1], m[0]]
            if m[0] > 0:
                self.set_dist(map_mak, mak_lst, map_src, m[0] - 1, m[1], src[0], src[1])
            if m[1] > 0:
                self.set_dist(map_mak, mak_lst, map_src, m[0], m[1] - 1, src[0], src[1])
            if m[0] < map_mak.shape[0] - 1:
                self.set_dist(map_mak, mak_lst, map_src, m[0] + 1, m[1], src[0], src[1])
            if m[1] < map_mak.shape[1] - 1:
                self.set_dist(map_mak, mak_lst, map_src, m[0], m[1] + 1, src[0], src[1])



if __name__ == '__main__':
    import matplotlib.pyplot as plt
    prob_map = Prob_map()
    prob_map.read_img('map3.pgm')
    prob_map.create_likelihood()
    plt.imshow(prob_map.map_lkf, cmap='Greys', interpolation='nearest')
    plt.show()