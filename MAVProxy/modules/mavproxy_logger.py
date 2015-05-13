import logging
import os
import os.path
import threading
import types
import sys
from pymavlink import mavutil

from time import sleep
from MAVProxy.modules.lib import mp_module
from MAVProxy.modules.lib import mp_util
from PyCRC.CRCCCITT import CRCCCITT
import time


class logger(mp_module.MPModule):
    def __init__(self, mpstate):
        """Initialise module."""
        super(logger, self).__init__(mpstate, "logger", "log")
        print "Logging Started!!"
        self.block_cnt = 0
        self.logfile = open('/Users/Siddharth/Documents/Programming/MAVProxy/magMAVproxy/MAVProxy/modules/log.bin', 'w')
        self.logfile.truncate()
        self.prev_cnt = 0
        self.download = 0
        self.prev_download = 0
        self.start = time.time()

    def idle_task(self):
        end = time.time()
        if (end - self.start) >= 10:
            print (self.download - self.prev_download)/((end - self.start)*1000), "Kb/s "
            self.start = time.time()
            self.prev_download = self.download

    def mavlink_packet(self, m):
        self.master.mav.remote_log_req_block_send(self.block_cnt)
        if m.get_type() == 'REMOTE_LOG_DATA_BLOCK':
            if self.block_cnt == m.block_cnt:
                size = m.block_size
                data = bytearray(m.data[:size])
                self.logfile.write(data)
                self.logfile.flush()
                self.download+=size
                self.block_cnt+=1
                #print self.status.msgs['REMOTE_LOG_DATA_BLOCK']
            
def init(mpstate):
    '''initialise module'''
    return logger(mpstate)