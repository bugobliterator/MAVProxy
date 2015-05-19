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
import time


class logger(mp_module.MPModule):
    def __init__(self, mpstate):
        """Initialise module."""
        super(logger, self).__init__(mpstate, "logger", "log")
        print "Logging Started!!"
        self.block_cnt = 1
        self.logfile = open('/Users/Siddharth/Documents/Programming/MAVProxy/magMAVproxy/MAVProxy/modules/log.bin', 'wb')
        self.logfile.truncate()
        self.prev_cnt = 0
        self.download = 0
        self.prev_download = 0
        self.start = time.time()
        self.missing_blocks = []
        self.new_log_started = False

    def idle_task(self):
        end = time.time()
        if self.new_log_started == False :
            self.master.mav.remote_log_block_status_send(0,1)
        else:
            self.master.mav.remote_log_block_status_send(self.block_cnt,1)
        if (end - self.start) >= 10:
            print "Log Download Rate:  ",(self.download - self.prev_download)/((end - self.start)*1000), "Kb/s "
            self.start = time.time()
            self.prev_download = self.download
        for missed_block in missing_blocks:
            print "Requesting Missed Block!!"
            self.master.mav.remote_log_block_status_send(self.block_cnt,0)

    def mavlink_packet(self, m):
        if m.get_type() == 'REMOTE_LOG_DATA_BLOCK':
            if m.block_cnt == 1:
                self.new_log_started = True
            if self.new_log_started == True:
                size = m.block_size
                data = ''.join(str(chr(x)) for x in m.data[:size])
                ofs = size*(m.block_cnt - 1)
                self.logfile.seek(ofs)
                self.logfile.write(data)
                self.logfile.flush()
                if(m.block_cnt - self.block_cnt > 1): #missed blocks
                    for blocks in range(self.block_cnt+1, m.block_cnt):
                        self.missing_blocks.append(blocks)
                elif(m.block_cnt - self.block_cnt < 1):
                    self.missed_blocks.remove(m.block_cnt)
                self.download+=size
                self.block_cnt=m.block_cnt

def init(mpstate):
    '''initialise module'''
    return logger(mpstate)
