from .robotiq_initializer import robotiq_initialize
from .robotiq_functions import *

"""Thresholds (greater or equal to threshold)"""
position_layer_0_r = 230
position_layer_1_r = 228
position_layer_2_r = 220
position_layer_0_l = 230
position_layer_1_l = 228
position_layer_2_l = 220

class GripCtrller(object):
    def __init__(self, simulation=False):
        self.serL = None
        self.serR = None
        self.sim = simulation
    
    def initialize(self):
        if self.sim is False:
            self.serL, self.serR = robotiq_initialize()
    
    def open_left(self):
        if self.sim is False:
            self._check_connection()
            setPosition(100, self.serL)

    def open_right(self):
        if self.sim is False:
            self._check_connection()
            setPosition(100, self.serR)

    def close_left(self):
        if self.sim is False:
            self._check_connection()
            setPosition(240, self.serL)

    def close_right(self):
        if self.sim is False:
            self._check_connection()
            setPosition(240, self.serR)
    
    def check_state_left(self):
        if self.sim is False:
            return check_detailed_status(self.serL,
                        position_layer_0_l,
                        position_layer_1_l,
                        position_layer_2_l)
        else:
            return 1

    def check_state_right(self):
        if self.sim is False:
            return check_detailed_status(self.serR,
                        position_layer_0_r,
                        position_layer_1_r,
                        position_layer_2_r)
        else:
            return 1
    
    def _check_connection(self):
        if self.serL is None or self.serR is None:
            raise Exception("Please initialize the gripper before controlling.")


