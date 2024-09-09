#!/usr/bin/python
# coding: UTF-8

from gui import AMCLGUI
#from likelihood_field_model import Prob_map
from prob_map import Prob_map
from readbag import BagReader
from odom_model import Odom_Model
from particle_cloud import Particle_cloud
from laser_model import Laser_model
from math import *
import numpy as np
import tf
import time
from PyQt5.QtWidgets import QApplication
import threading


##########################
bagfile = 'h3.bag'
scan_topic = '/scan' 
odom_topic = '/odom'
start_time = 0
end_time = 800
##########################
image_file_name = 'map3.pgm'
##########################
d_thresh_ = .1
a_thresh_ = np.pi/12
##########################
lidar_angle = 0.
lidar_x = 0.
lidar_y = 0.
##########################
original_point = (100,100)
max_dist = 0.8 
resolution = 0.025
fre_thr = 200
occ_thr = 20
##########################
particle_num = 5000
init_partcle_pose_unknow = True
init_partcle_pose = (0,0,0)
init_partcle_trans_sigma = 1.3
init_partcle_rot_sigma = 1.3
alpha_slow =  0.01
alpha_fast = 0.1
##########################
odom_aphla1 = 0.006 #Weight of rotation error resulting from rotation
odom_aphla2 = 0.030 #Weight of rotation error resulting from translation
odom_aphla3 = 0.070 #Weight of translation error resulting from translation
odom_aphla4 = 0.010 #Weight of translation error resulting from rotation
##########################
z_hit = 0.95
z_rand = 0.05
sigma_hit = 0.2
range_max = 8.

class AMCL():
    def __init__(self, raw_data, costmap, gui):
        self.raw_data = raw_data
        self.costmap = costmap
        self.gui = gui
        self.scan_base = (lidar_x,lidar_y,lidar_angle)
        self.particle_cloud = Particle_cloud(particle_num, alpha_slow, alpha_fast)
        self.particle_cloud.set_init_particles(init_partcle_pose_unknow, init_partcle_pose, init_partcle_trans_sigma, init_partcle_rot_sigma)
        self.odom_model = Odom_Model(odom_aphla1, odom_aphla2, odom_aphla3, odom_aphla4)
        self.laser_model = Laser_model(costmap,z_hit, z_rand, sigma_hit, range_max)

    def matrix_to_pose(self, odom):
        al, be, ga = tf.transformations.euler_from_matrix(odom[0:3,0:3])
        return (odom[0,3],odom[1,3],ga)

    def pose_to_matrix(self, pose):
        pose_matrix = tf.transformations.euler_matrix(0.0, 0.0, pose[2])
        pose_matrix[0,3] = pose[0] 
        pose_matrix[1,3] = pose[1]
        return pose_matrix
              
    def checkupdate(self):
        dx = self.pre_pose[0] - self.cur_pose[0]
        dy = self.pre_pose[1] - self.cur_pose[1]
        da = self.pre_pose[2] - self.cur_pose[2]
        trans = sqrt(dx**2 + dy**2)
        update = (trans > d_thresh_) or (fabs(da) > a_thresh_)
        return update

    def gui_update(self):
            ps = [ (p[0][0]/self.costmap.resolution + self.costmap.original_point[0],p[0][1]/self.costmap.resolution + self.costmap.original_point[1], p[0][2]) for p in self.particle_cloud.particles ]
            pose = self.particle_cloud.best_p[0]
            #print self.laser_model.get_probability(self.pre_pose, self.scan)
            pose_gui = [0,0,0]
            pose_gui[0] = pose[0]/self.costmap.resolution + self.costmap.original_point[0]
            pose_gui[1] = pose[1]/self.costmap.resolution + self.costmap.original_point[1]
            pose_gui[2] = pose[2]
            map_scan, world_scan = self.laser_model.get_scan_in_world_coord(self.scan, pose)
            show_likelihood_field = self.gui.checkbox_show_likelihood_field.isChecked()
            if show_likelihood_field:
                gui.setdata(self.costmap.map_lkf, ps, pose_gui,map_scan)
            else:
                gui.setdata(self.costmap.map_raw.astype(float), ps, pose_gui,map_scan)

            
    def step(self):
        scan, odom = self.raw_data[self.idx]
        self.scan = np.matrix(scan)
        odom = np.matrix(odom)
        try:
            self.last_odom = self.cur_odom
            self.cur_odom = odom
            last_odom_inv = np.matrix(np.linalg.inv(self.last_odom))
            odom_delta = last_odom_inv * self.cur_odom
            pose_matrix = self.pose_to_matrix(self.cur_pose)
            new_pose_matrix = pose_matrix * odom_delta
            self.cur_pose = self.matrix_to_pose(new_pose_matrix)
            if not self.checkupdate():
                return False
        except AttributeError:
            self.cur_pose = init_partcle_pose
            self.pre_pose = init_partcle_pose
            self.cur_odom = odom
            return False
        start = time.time()
        self.particle_cloud.update_by_odom_model(self.odom_model.update, self.pre_pose, self.cur_pose)
        elapsed_time = time.time() - start
        #print ("update_by_odom_model elapsed_time:{0}".format(elapsed_time)) + "[sec]"
        start = time.time()
        self.particle_cloud.update_by_laser_model(self.laser_model.get_probability,self.scan_base ,self.scan)
        elapsed_time = time.time() - start
        #print ("update_by_laser_model elapsed_time:{0}".format(elapsed_time)) + "[sec]"
        start = time.time()
        self.particle_cloud.update_by_resample()
        elapsed_time = time.time() - start
        #print ("update_by_resample elapsed_time:{0}".format(elapsed_time)) + "[sec]"
        start = time.time()
        self.pre_pose = self.cur_pose
        return True
        

def update(amcl):
        amcl.idx = 0
        while True:
            time.sleep(0.03)
            if amcl.gui.state == 1:
                update = False
                while not update:
                    if amcl.idx >= len(amcl.raw_data):
                        break
                    update = amcl.step()
                    amcl.idx += 1
                amcl.gui_update()
            elif amcl.gui.state == 2:
                amcl.gui.state = 0
                update = False
                while not update:
                    if amcl.idx >= len(amcl.raw_data):
                        break
                    update = amcl.step()
                    amcl.idx += 1
                amcl.gui_update()    


if __name__ == "__main__":
    app = QApplication([])
    bagreader = BagReader(bagfile, scan_topic, odom_topic, start_time, end_time)
    gui = AMCLGUI()
    costmap = Prob_map(original_point, max_dist, resolution, fre_thr , occ_thr)
    costmap.read_img(image_file_name)
    costmap.create_likelihood()

    amcl = AMCL(bagreader.data, costmap, gui)

    th = threading.Thread(target=update, args=(amcl, ))
    th.start()

    app.exec_()
