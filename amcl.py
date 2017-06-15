#!/usr/bin/python
# coding: UTF-8

from gui import *
from likelihood_field_model import Prob_map
from readbag import *
from particle_filter import *
gui = LSLAMGUI()
bagreader = BagReader('h3.bag', '/scan', '/odom',0,800)
#gui.start()
prob_map = Prob_map()
prob_map.read_img('map3.pgm')
prob_map.create_likelihood()

particle_filter = Particle_filter()
particle_filter.set_init_particles((2.5,2.5,0),0.03,0.01)
ps = [ (p[0]/prob_map.scale,p[1]/prob_map.scale,p[2]) for p in particle_filter.particles ]
gui.setdata(prob_map.map_lkf, ps, [0,0,0], np.zeros((10,2)))
gui.run()
