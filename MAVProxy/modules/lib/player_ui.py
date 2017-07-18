"""
  MAVExplorer Visualisation module's UI
"""
from MAVProxy.modules.lib import player
import platform
if platform.system() == 'Darwin':
    from billiard import Pipe, Process, Event, forking_enable, freeze_support
else:
    from multiprocessing import Pipe, Process, Event, freeze_support

class Player_UI(object):
    """docstring for ClassName"""
    def __init__(self, mestate):
        self.mestate = mestate

    def run_player(self, graphdef):
        if 'mestate' in globals():
            self.mestate.console.write("Running Player...")
        else:
            self.mestate.child_pipe_send_console.send("Running Player...")
        self.player = player.MavPlay()
        self.player.add_mav(self.mlog)
        if platform.system() == 'Darwin':
            forking_enable(False)

        child = Process(target=self.player.run)
        child.start()