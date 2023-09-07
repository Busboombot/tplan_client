#! /usr/local/bin/python
from __future__ import division

from collections import namedtuple
from math import copysign
from time import time, sleep
from enum import Enum
from dataclasses import dataclass, field
from typing import List


def mkmap(r1, r2, d1, d2):
    """Map from one interval range to another"""
    r = r2 - r1
    d = d2 - d1

    def range(x):

        sign = x / abs(x) if x != 0 else 0
        x = abs(x)

        if x < r1:
            x = r1
        elif x > r2:
            x = r2

        s = float(x - r1) / float(r)
        v = d1 + (s * d)

        return sign * v

    return range


JoyValues = namedtuple('JoyValues', 'seq now delta button trigger axes'.split())


class Button(Enum):
    Y = 'y'
    B = 'b'
    A = 'a'
    X = 'x'
    STICK_R = 'stick_r'
    STICK_L = 'stick_l'
    START = 'start'
    BACK = 'back'
    RT = 'rt'
    LT = 'lt'
    RB = 'rb'
    LB = 'lb'


class Hat(Enum):
    N = (0, 1)
    NE = (1, 1)
    E = (1, 0)
    SE = (1, -1)
    S = (0, -1)
    SW = (-1, -1)
    W = (-1, 0)
    NW = (-1, 1)
    NONE = (0, 0)


@dataclass
class HidJoyValues:
    t: float = 0
    d_t: float = 0

    l_x: float = 0
    l_y: float = 0
    r_x: float = 0
    r_y: float = 0
    h_x: float = 0
    h_y: float = 0

    hat: str = None

    buttons: List[Button] = field(default_factory=list)

    speeds = [3000, 10000]

    def __key(self):
        return (self.l_x, self.l_y, self.r_x, self.r_y, self.hat, *self.buttons)

    def __hash__(self):
        return hash(self.__key())

    def __eq__(self, other):
        if isinstance(other, HidJoyValues):
            return self.__key() == other.__key()
        return NotImplemented

    @property
    def axes(self):
        return [self.l_x, self.l_y, self.r_x, self.r_y, self.h_x, self.h_y]

    def __str__(self):
        return " ".join(map(lambda v: f"{round(v, 3):+1.3f}", self.axes)) + " " + " ".join(self.buttons)

    @property
    def moves(self):
        speed = self.speeds[int(Button.LT in self.buttons)]

        return list(map(lambda v: int(speed * v), self.axes))


class HidJoystick:
    """A Joystick class that uses the first Logitech joystick, via the Hid library. """

    def __init__(self, interval: float = .1):
        import hid

        self.d_t = interval  # seconds

        self.vid, self.pid = self.find_first_joystick()

        self.device = hid.device(self.vid, self.pid)
        self.device.open(self.vid, self.pid)

        # Left and right joystick, x and y axes
        self.lj_x, self.lj_y, self.rj_x, self.rj_y = [None] * 4

        # Buttons a, b , x, y
        self.b_y, self.b_b, self.b_a, self.b_x = [None] * 4
        self.stick_r, self.stick_l = None, None  # Stick buttons
        self.start, self.back = None, None
        self.rt, self.lt = None, None  # Triggers
        self.rb, self.lb = None, None  # Left and right buttons
        self.hat, self.hat_dir = None, None

        self.time = time()
        self.last_time = self.time

        self.last_jv = None

    @classmethod
    def find_first_joystick(cls, name='Logitech Dual Action'):
        """Find the first Logitec Dual device"""
        import hid

        vid, pid = None, None

        for device in hid.enumerate():

            #print(f"0x{device['vendor_id']:04x}:0x{device['product_id']:04x} {device['product_string']}")

            if name in device['product_string']:
                vid = device['vendor_id']
                pid = device['product_id']

                return vid, pid
        return None, None

    def __iter__(self):
        while True:
            jv = self.next()
            if jv:
                yield jv

    def next_until(self, interval):

        while True:
            jv = self.next()

            if time() - self.last_time > self.d_t and jv is not None:
                self.last_time = time()
                return jv


    def next(self):

        z = self.device.read(max_length=8, timeout_ms=10) # self.d_t * 1000)

        if z:
            jv = HidJoyValues()

            jv.r_x, jv.r_y, jv.l_x, jv.l_y, *buttons = z

            def remap(v):
                v = v - 127.5

                if abs(v) <= 4:
                    v = 0

                return v / 127.5

            jv.r_x, jv.r_y, jv.l_x, jv.l_y = map(remap, (jv.r_x, jv.r_y, jv.l_x, jv.l_y))

            state_l = []
            for i, g in enumerate(buttons[:3]):
                for j, b in enumerate(f"{g:08b}"):
                    state_l.append(b)

            y, b, a, x, *state_l = state_l

            hat, state_l = state_l[:4], state_l[4:]  # hat is 4 bits

            hat = int(''.join([str(e) for e in hat]), 2)

            jv.hat = list(Hat.__members__.keys())[hat]
            jv.h_x, jv.h_y = list(Hat.__members__.values())[hat].value

            stick_r, stick_l, start, back, rt, lt, rb, lb, *state_l = state_l

            jv.buttons = [name for name, val in zip(Button.__members__.values(),
                                                    (y, b, a, x, stick_r, stick_l, start, back, rt, lt, rb, lb))
                          if int(val)]

            self.last_jv = jv
        else:
            jv = self.last_jv


        return jv
