"""
  MAVExplorer Visualisation module
"""
from visual import *
from visual.controls import *
from visual.graph import *
from math import *
from lib.plane_obj import *
from lib import mavmemlog
from pymavlink import mavutil
from pymavlink.mavextra import *
import os
import time
import transforms3d
import wx
import copy

class MavPlay(object):
    def __init__(self):
        self.margin = 20
        self.width, self.height = wx.GetDisplaySize()
        self.window = window(menus=True, title='MavPlay', width=self.width, height=10*self.height/11,
            style=wx.SYSTEM_MENU | wx.CAPTION | wx.CLOSE_BOX | wx.RESIZE_BORDER | wx.FULL_REPAINT_ON_RESIZE)
        self.panel = self.window.panel
        self.scene = display(window=self.window, width=self.window.width, height=(self.window.height*4/5))
        self.scene.title = "MavPlay Visualizer"
        self.scene.range=(12,12,12)
        self.scene.forward = (10,10,5)
        self.scene.up=(0,0,-1)
        self.scene.select()
        self.axes = axes(axiswidth=0.0, tics=True)
        distant_light(direction=(0.22, 0.44, -0.88),color=color.gray(0.8))
        self.frame = frame()
        self.vehicle = plane(frame = self.frame)
        self.vehicle.pos = (0,0,-.1)
        self.floor = box(size=(1000, 0.1, 1000), up=(0,0,-1), material=materials.wood)
        self.mlog = None
        wx.StaticText(self.panel, pos=(self.margin, self.scene.height+self.margin), label='Seek')
        self.seek = wx.Slider(self.panel, pos=(self.margin, self.scene.height+self.margin*3/2), size=(self.scene.width,20), minValue=0, maxValue=10000)
        self.seek.Bind(wx.EVT_SLIDER, self.onseek)

        wx.StaticText(self.panel, pos=(self.margin, self.scene.height+3*self.margin), label='Speed')
        self.speed = wx.Slider(self.panel, pos=(self.margin, self.scene.height+self.margin*3.5), size=(self.scene.width/4,20), minValue=-4, maxValue=4)
        self.speed.SetValue(0)

    def onseek(self, e):
        obj = e.GetEventObject()
        val = obj.GetValue()
        self.mlog.set_pct(val/100)

    def add_mav(self, mlog):
        self.mlog = mlog

    def run(self):
        first_val = True
        last_att = None
        rmat = Matrix3()
        rfnd_dist = 0
        while True:
            msg = self.mlog.recv_msg()
            if msg is None:
                break
            if msg.get_type() == "RFND":
                rfnd_dist = msg.Dist1
            if msg.get_type() == "POS":
                if first_val and abs(msg.Lat) > 0.3 and abs(msg.Lng) > 0.3 and abs(msg.Alt) > 0.1:
                    home = vector(msg.Lat, msg.Lng, msg.Alt)
                    first_val = False
                    last_time = msg.TimeUS
                elif first_val == False:
                    time_delta = msg.TimeUS - last_time
                    r = 1000000/time_delta
                    r *= 2**self.speed.GetValue()
                    if r <= 1 :
                        last_time = msg.TimeUS
                        continue
                    rate(r)
                    if (home.z - msg.Alt) < -args.max_rfnd:
                        curr_pos = vector((msg.Lat - home.x) * 111318.84502145034,
                            (msg.Lng - home.y) * 111318.84502145034 * cos(home.x*pi/180),
                            (home.z - msg.Alt))
                    else:
                        curr_pos = vector((msg.Lat - home.x) * 111318.84502145034,
                            (msg.Lng - home.y) * 111318.84502145034 * cos(home.x*pi/180),
                            (-rfnd_dist))
                    self.seek.SetValue(self.mlog.get_pct()*100)
                    self.vehicle.pos = (curr_pos.x, curr_pos.y, curr_pos.z-3)
                    rmat.from_euler(curr_att.x,curr_att.y,curr_att.z)
                    axis = rmat * Vector3(1,0,0) 
                    up = rmat * Vector3(0,0,-1)
                    last_att = curr_att
                    self.vehicle.axis = (axis.x, axis.y, axis.z)
                    self.vehicle.up = (up.x, up.y, up.z)
                    self.scene.center = (curr_pos.x, curr_pos.y, curr_pos.z-3)
                    last_time = msg.TimeUS
            if msg.get_type() == "ATT":
                    curr_att = vector(msg.Roll*pi/180, msg.Pitch*pi/180, msg.Yaw*pi/180)

if __name__ == "__main__":
    from argparse import ArgumentParser
    parser = ArgumentParser(description=__doc__)
    parser.add_argument("--max-rfnd", type=float,default=0, help="Maximum Rangefinder value")
    parser.add_argument("--min-rfnd",  type=float,default=0, help="Minimum Rangefinder value")
    parser.add_argument("--alt", default='BARO', help="use baro sensor for hieght data")
    parser.add_argument("--gps-pos", default='GPS', help="use GPS for lat long")
    parser.add_argument("--use-ekf-data", action='store_true', help="use ekf data for position value")
    parser.add_argument("--dialect", default="ardupilotmega", help="MAVLink dialect")
    parser.add_argument("logs_fields", metavar="<LOG or FIELD>", nargs="+")

    args = parser.parse_args()
    filenames = []

    mpl = MavPlay()
    for files in args.logs_fields:
        if os.path.exists(files):
            mlog = mavmemlog.mavmemlog(mavutil.mavlink_connection(files, notimestamps=False,
                                              zero_time_base=False,
                                              dialect=args.dialect))
            mpl.add_mav(mlog)
        else:
            print "File Not Found!!"
    mpl.run()
