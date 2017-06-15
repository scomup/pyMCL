#!/usr/bin/python
# coding: UTF-8

from gui import *
from likelihood_field_model import Prob_map

gui = LSLAMGUI()
#gui.start()
prob_map = Prob_map()
prob_map.read_img('map3.pgm')
prob_map.create_likelihood()
gui.setdata(prob_map.map_lkf, [], [0,0,0], np.zeros((10,2)))
gui.run()
