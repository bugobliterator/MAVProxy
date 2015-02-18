import logging
import os
import os.path
import threading
import types
import sys
from mpl_toolkits.mplot3d.axes3d import Axes3D
import matplotlib.pyplot as plt
from matplotlib import cm
import numpy
from pymavlink import mavutil

from time import sleep
from MAVProxy.modules.lib import mp_module
from MAVProxy.modules.lib import mp_util
from MAVProxy.modules.lib import mp_magcalib


class magcalib(mp_module.MPModule):
    def __init__(self, mpstate):
        """Initialise module."""
        super(magcalib, self).__init__(mpstate, "magcalib", "3D plotter")
        self.data = numpy.array([[0,0,0] for i in range(100)], dtype=float)
        self.i = 0
        self.iter = 0
        self.octant = numpy.array([0,0,0,0,0,0,0,0],dtype=int); 
        master = self.mpstate.mav_master[0]
        mpstate.mav_param.show('COMPASS_PRIMARY')
        self.param = numpy.array([20,20,20,20],dtype=float)[numpy.newaxis]
        print 'Magnetometer Calibration Started: rotate your vehicle all axes'
        print 'Iteration: ',self.iter,'/10'
    
    def idle_task(self):
        i = self.i
        if 'SENSOR_OFFSETS' in self.mpstate.status.msgs and 'RAW_IMU' in self.mpstate.status.msgs:
            self.mag_ofs_x = self.mpstate.status.msgs['SENSOR_OFFSETS'].mag_ofs_x
            self.mag_ofs_y = self.mpstate.status.msgs['SENSOR_OFFSETS'].mag_ofs_y
            self.mag_ofs_z =  self.mpstate.status.msgs['SENSOR_OFFSETS'].mag_ofs_z
            self.data[i,0] = self.mpstate.status.msgs['RAW_IMU'].xmag - self.mag_ofs_x
            self.data[i,1] = self.mpstate.status.msgs['RAW_IMU'].ymag - self.mag_ofs_y
            self.data[i,2] = self.mpstate.status.msgs['RAW_IMU'].zmag - self.mag_ofs_z
        else:
            return
        if(i >= 1):
                if((abs(self.data[(i-1),0] - self.data[i,0]) >= 2) or 
                    (abs(self.data[(i-1),1] - self.data[i,1]) >= 2) or 
                    (abs(self.data[(i-1),2] - self.data[i,2]) >= 2)):
                        self.i = self.i + 1
                        sys.stdout.write("Download progress: [%d] \r" % (self.i))
                        sys.stdout.flush()
        elif i < 1:
            self.i = self.i + 1

        if self.i >= 100:
            self.param,self.best_past_fitness = mp_magcalib.magcalib(self.param,self.data[numpy.newaxis])
            print 'Iteratation Done!     '
            print 'best params: ', self.param,'fitness: ',self.best_past_fitness
            if self.best_past_fitness > 1 or self.iter < 9:
                self.i = 0
                self.iter = self.iter+1
                print 'Iteration: ',self.iter+1
            else:
                print '10/10 Iterations Completed.'
                print 'Best Parameters found:  '
                print 'best params(rad,off1,off2,off3): ', self.param,'last_fitness: ',self.best_past_fitness

def init(mpstate):
    '''initialise module'''
    return magcalib(mpstate)   