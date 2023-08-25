#! /usr/local/bin/python
from __future__ import division

from collections import namedtuple
from math import copysign
from time import time, sleep


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


class RobotJoystick():

    def __init__(self, t=None):
        """Read the pygame joystick and yield frequency values for the stepper motors.

        param t: minimum frequency, in seconds,  at which to yield a result, even if there are no changes.
        Defaults to 1
        """

        self.init()

        if t:
            self.interval = int(t * 1000)
        else:
            self.interval = 500

        self.last = JoyValues(0, 0, 0, [], [], [0] * 6)


class PygameJoystick(RobotJoystick):

    def init(self):
        import pygame
        pygame.init()
        pygame.joystick.init()

        self.joysticks = [pygame.joystick.Joystick(x) for x in range(pygame.joystick.get_count())]

        for j in self.joysticks:
            j.init()

        if len(self.joysticks) == 0:
            raise Exception("Didn't find any joysticks")

        for i, j in enumerate(self.joysticks):
            pass
            # print j.get_init(), j.get_name()
            # print [j.get_axis(k) for k in range(j.get_numaxes())]
            # print [j.get_button(z) for z in range(j.get_numbuttons())]

    def __del__(self):
        import pygame
        pygame.display.quit()

    def __iter__(self):
        import pygame
        from pygame.locals import KEYDOWN, K_ESCAPE, QUIT, \
            JOYAXISMOTION, JOYBALLMOTION, JOYBUTTONDOWN, JOYBUTTONUP, JOYHATMOTION

        joy_events = (JOYAXISMOTION, JOYBALLMOTION, JOYBUTTONDOWN, JOYBUTTONUP, JOYHATMOTION)

        seq = 0

        LOWER_LIMIT = 0.03
        mp = mkmap(LOWER_LIMIT, 1, 0, 1)

        def p(axis):
            v = j.get_axis(axis)
            v = v if abs(v) > LOWER_LIMIT else LOWER_LIMIT
            v = mp(v)
            return copysign(abs(v), v)

        interval_s = self.interval / 1e3
        lasttime = time()
        while True:

            event = pygame.event.wait(self.interval)

            if (event.type == KEYDOWN and event.key == K_ESCAPE):
                pygame.quit()
                return

            elif (event.type == QUIT):
                pygame.quit()
                return

            if event and event.type in joy_events:
                i = event.joy
                j = self.joysticks[i]

                button = [z + 1 for z in range(j.get_numbuttons()) if j.get_button(z) and z in range(4)]

                trigger = [z - 4 for z in range(j.get_numbuttons()) if j.get_button(z) and z in range(4, 8)]

                if j.get_numhats():
                    hats = [j.get_hat(i) for i in range(j.get_numhats())]

                    axes = [p(axis) for axis in range(j.get_numaxes())] + \
                           [copysign(abs(h), h) for h in hats[0]]

                else:
                    axes = [p(axis) for axis in range(j.get_numaxes())]

            else:
                button = self.last.button
                trigger = self.last.trigger
                axes = self.last.axes

            now = time()
            delta = now - lasttime

            if now - lasttime < interval_s:
                sleep(interval_s - (now - lasttime))

            lasttime = now

            self.last = JoyValues(seq, now, delta, button, trigger, axes)
            seq += 1

            yield self.last

from dataclasses import dataclass, field
from typing import List

@dataclass
class HidJoyValues:
    l_x: float = 0
    l_y: float = 0
    r_x: float = 0
    r_y: float = 0

    hat: str = None

    buttons: List[str] = field(default_factory=list)


    def __key(self):
        return (self.l_x, self.l_y, self.r_x, self.r_y, self.hat, *self.buttons)

    def __hash__(self):
        return hash(self.__key())

    def __eq__(self, other):
        if isinstance(other, HidJoyValues):
            return self.__key() == other.__key()
        return NotImplemented


class HidJoystick:
    """A Joystick class that uses the first Logitech joystick, via the Hid library. """

    hat_dirs = ['N','NE','E','SE','S','SW','W','NW',None]

    button_names = ['y', 'b', 'a', 'x', 'stick_r', 'stick_l', 'start', 'back', 'rt', 'lt', 'rb', 'lb']

    def __init__(self):
        import hid

        self.vid, self.pid = self.find_first_joystick()

        self.device = hid.device(self.vid, self.pid)
        self.device.open(self.vid, self.pid)

        self.last = None
        self.last_state_i = None

        # Left and right joystick, x and y axes
        self.lj_x, self.lj_y, self.rj_x, self.rj_y = [None] * 4

        # Buttons a, b , x, y
        self.b_y, self.b_b, self.b_a, self.b_x = [None] * 4
        self.stick_r, self.stick_l = None, None # Stick buttons
        self.start, self.back = None, None
        self.rt, self.lt = None, None # Triggers
        self.rb, self.lb = None, None # Left and right buttons
        self.hat, self.hat_dir = None, None

    @classmethod
    def find_first_joystick(cls, name='Logitech Dual Action'):
        """Find the first Logitec Dual device"""
        import hid

        vid, pid = None, None

        for device in hid.enumerate():

            print(f"0x{device['vendor_id']:04x}:0x{device['product_id']:04x} {device['product_string']}")

            if name in device['product_string']:
                vid = device['vendor_id']
                pid = device['product_id']

        return vid, pid

    def __iter__(self):
        while True:
            jv = self.next()
            if jv:
                yield jv

    def next(self):

        jv = HidJoyValues()

        z = self.device.read(max_length=8, timeout_ms=50)

        if not z:
            return None

        jv.r_x, jv.r_y, jv.l_x, jv.l_y, *buttons = z

        state_l = []
        for i, g in enumerate(buttons[:3]):
            for j, b in enumerate(f"{g:08b}"):
                state_l.append(b)

        y, b, a, x, *state_l = state_l

        hat, state_l = state_l[:4], state_l[4:] # hat is 4 bits

        hat = int(''.join([str(e) for e in hat]), 2)

        jv.hat = self.hat_dirs[hat]

        stick_r, stick_l, start, back, rt, lt, rb, lb, *state_l = state_l

        jv.buttons = [ name for name, val in zip(self.button_names,
                                              (y, b, a, x, stick_r, stick_l, start, back, rt, lt, rb, lb))
                    if int(val)]

        return jv
