#!/usr/bin/python
# coding: UTF-8

from pyqtgraph.Qt import QtCore, QtGui
import numpy as np
import pyqtgraph as pg
import pyqtgraph.opengl as gl
import signal
import Queue
import sys
import time
import threading


class RobotItem(QtGui.QGraphicsItem):
    """a sample robot item"""
    def __init__(self, color):
        super(RobotItem, self).__init__()
        #self.setFlag(QtGui.QGraphicsItem.ItemIsMovable)
        self.setCacheMode(QtGui.QGraphicsItem.DeviceCoordinateCache)
        self.setZValue(1)
        self.color = color
        
    def boundingRect(self):
        adjust = 2.0
        return QtCore.QRectF(-10 - adjust, -10 - adjust, 20 + adjust,
                20 + adjust)

    def paint(self, painter, option, widget):
        #Draw a sample robot
        pen = QtGui.QPen()
        pen.setWidth(1);
        if self.color =='r':
            pen.setBrush(QtCore.Qt.red)
        elif self.color =='b':
            pen.setBrush(QtCore.Qt.blue)
        else:
            pen.setBrush(QtCore.Qt.green)
        painter.setPen(pen)
        painter.setBrush(QtCore.Qt.NoBrush)
        painter.drawEllipse(QtCore.QPointF(0.0, 0.0), 5, 5)
        painter.drawLine(0, 0, 5, 0)

class ParticleItem(QtGui.QGraphicsItem):
    """a sample robot item"""
    def __init__(self, color):
        super(ParticleItem, self).__init__()
        #self.setFlag(QtGui.QGraphicsItem.ItemIsMovable)
        self.setCacheMode(QtGui.QGraphicsItem.DeviceCoordinateCache)
        self.setZValue(1)
        self.color = color
        
    def boundingRect(self):
        adjust = 2.0
        return QtCore.QRectF(-10 - adjust, -10 - adjust, 20 + adjust,
                20 + adjust)

    def paint(self, painter, option, widget):
        #Draw a sample robot
        pen = QtGui.QPen()
        pen.setWidth(0.9);
        if self.color =='r':
            pen.setBrush(QtCore.Qt.red)
        elif self.color =='b':
            pen.setBrush(QtCore.Qt.blue)
        else:
            pen.setBrush(QtCore.Qt.green)
        painter.setPen(pen)
        painter.setBrush(QtCore.Qt.NoBrush)
        painter.drawLine(3, -1, 4, 0)
        painter.drawLine(3, 1, 4, 0)
        painter.drawLine(-4, 0, 4, 0)


class AMCLGUI(threading.Thread):
    def __init__(self):
        threading.Thread.__init__(self)
        self.setDaemon(True)
        self.q = Queue.Queue()
        self.state = 0 
        self.particle_handle = [] 

    def run(self):
        ## Always start by initializing Qt (only once per application)
        app = QtGui.QApplication([])

        ## Define a top-level widget to hold everything
        w = QtGui.QWidget()
        w.resize(800,800)
        w.setWindowTitle("AMCL Viewer")

        ## Create some widgets to be placed inside
        #text = QtGui.QLineEdit('enter text')

        p2d = pg.GraphicsView()

        button_play = QtGui.QPushButton('Play')
        button_play.setFixedWidth(110)
        button_play.clicked.connect(self.handleButton_play)

        button_next = QtGui.QPushButton('Next')
        button_next.setFixedWidth(110)
        button_next.clicked.connect(self.handleButton_next)

        self.checkbox_show_likelihood_field = QtGui.QCheckBox("Show likelihood field")
        self.checkbox_show_likelihood_field.setChecked(False)


        ## Create a grid layout to manage the widgets size and position
        layout = QtGui.QGridLayout()
        w.setLayout(layout)

        ## Add widgets to the layout in their proper positions
        layout.addWidget(p2d, 0, 0, 1, 5)  
        layout.addWidget(button_play, 2, 0)
        layout.addWidget(button_next, 2, 1)
        layout.addWidget(self.checkbox_show_likelihood_field,2,2)


        # Create a viewBox for 2D image
        vb = pg.ViewBox()
        vb.setAspectLocked()
        p2d.setCentralItem(vb)

        self.prob_map = gl.GLSurfacePlotItem(z=np.zeros((80, 80)), shader='shaded', color=(0.5, 0.5, 1, 1))
        self.prob_map.scale(0.5, 0.5, 1.0)
        self.prob_map.translate(-20, -20, 0)


        #Create ImageItem for map
        self.img = pg.ImageItem(np.zeros((400,400)))
        vb.addItem(self.img)

        ## Display the widget as a new window
        w.show()

        ## Set image level
        self.img.setLevels([0, 1])

        #Create ScatterPlotItem for scan data 
        self.sct = pg.ScatterPlotItem(pen = pg.mkPen(None), 
                                      brush = pg.mkBrush("g"), 
                                      size =5, 
                                      antialias = False)
        self.sct.setParentItem(self.img)

        #Create RobotItem(custom) for showing robot pose 
        self.robot = RobotItem('b')
        self.robot.setParentItem(self.img)
        self.robot.setZValue(10)
        
        
        #Set timer
        timer = pg.QtCore.QTimer()
        timer.timeout.connect(self.update)
        timer.start(300)

        ## Start the Qt event loop
        app.exec_()

    def handleButton_play(self):
        self.state = 1  

    def handleButton_next(self):
        self.state = 2  

    def crl_partcles(self):
        while len(self.particle_handle) > 0:
            a = self.particle_handle.pop(0)
            a.setParentItem(None)
            
    def set_partcles(self,particles):
        for particle in particles:
            #a = pg.ArrowItem(angle=particle[2]*180/np.pi + 180., tipAngle=30, baseAngle=20, headLen=10, tailLen=20, tailWidth=1, pen=None, brush='y')
            #print particle
            a = ParticleItem('r')
            a.setPos(particle[0],particle[1])
            a.setRotation(180.*particle[2]/np.pi)
            a.setParentItem(self.img)
            self.particle_handle.append(a)

    def update(self):
        try:
            #Check is there any new data in queue
            prob, particles, pose, newscan = self.q.get(block=False)
            #
            
            self.crl_partcles()
            self.q.queue.clear()

            self.set_partcles(particles)
            #remove previous laser scan data
            self.sct.clear()
            #update map
            #I = np.zeros(prob.shape)
            #I.fill(1)
            self.img.setImage(prob.transpose())
            #update robot pose
            self.robot.setRotation(180.*pose[2]/np.pi)
            self.robot.setPos(pose[0],pose[1])
            #update laser scan
            #spots = [{'pos': pos} for pos in newscan]
            spots = [{'pos': newscan[i,:] } for i in range(newscan.shape[0])]
            self.sct.addPoints(spots)
        except Queue.Empty:
            pass

    def setdata(self, probdata, particles, robotpose, newscan):
        self.q.put( ( probdata, particles, robotpose, newscan) )
        pass

if __name__ == "__main__":
    gui = LSLAMGUI()
    gui.start()
    print 'sample gui test'
    import random
    for i in range(1000):
        time.sleep(0.05)
        newscan = np.zeros((10,2))
        newscan.fill(0.1)
        random.random
        particles = [(100*random.random()+200 , 100*random.random()+200, random.random()) for i in range(1000)]
        #particles = [ (-20,-20,0) ]
        gui.setdata(np.random.rand(400,400), particles, [0,0,i], newscan)

    
