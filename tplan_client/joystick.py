#! /usr/local/bin/python
from __future__ import division

from collections import namedtuple
from math import copysign
from time import time


def mkmap(r1, r2, d1, d2):
    """Map from one interval range to another"""
    r = r2 - r1
    d = d2 - d1

    def range(x):

        sign = x/abs(x) if x!= 0 else 0
        x = abs(x)

        if x < r1:
            x = r1
        elif x > r2:
            x = r2

        s = float(x - r1) / float(r)
        v = d1 + (s * d)

        return sign*v

    return range



JoyValues = namedtuple('JoyValues', 'seq now delta button axis_mode axes'.split())


class PygameJoystick(object):

    def __init__(self, t=None):
        """Read the pygame joystick and yield frequency values for the stepper motors. 
        
        param t: minimum frequency, in seconds,  at which to yield a result, even if there are no changes.
        Defaults to 1 
        """

        import pygame

        pygame.init()
        pygame.joystick.init()

        self.joysticks = [pygame.joystick.Joystick(x) for x in range(pygame.joystick.get_count())]

        for j in self.joysticks:
            j.init()

        for i, j in enumerate(self.joysticks):
            pass
            # print j.get_init(), j.get_name()
            # print [j.get_axis(k) for k in range(j.get_numaxes())]
            # print [j.get_button(z) for z in range(j.get_numbuttons())]

        if t:
            self.interval = int(t * 1000)
        else:
            self.interval = 500

        self.last = JoyValues(0, 0, 0, [], 0, [0] * 6)

    def __iter__(self):
        import pygame
        from pygame.locals import KEYDOWN, K_ESCAPE, QUIT, \
            JOYAXISMOTION, JOYBALLMOTION, JOYBUTTONDOWN, JOYBUTTONUP, JOYHATMOTION

        joy_events = (JOYAXISMOTION, JOYBALLMOTION, JOYBUTTONDOWN, JOYBUTTONUP, JOYHATMOTION)

        seq = 0;
        lasttime = time()

        def p(axis):
            v = j.get_axis(axis)
            v = v if abs(v) > .03 else 0
            return copysign(abs(v), v)

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

                axis_mode = [z for z in range(j.get_numbuttons()) if j.get_button(z) and z in range(4, 8)]

                hats = [j.get_hat(i) for i in range(j.get_numhats())]

                axes = [p(axis) for axis in range(j.get_numaxes())] + \
                       [copysign(abs(h), h) for h in hats[0]]

            else:
                button = self.last.button
                axis_mode = self.last.axis_mode
                axes = self.last.axes

            now = time()
            delta = now - lasttime

            if delta > self.interval/1e6:

                self.last = JoyValues(seq, now, delta, button, axis_mode, axes)
                lasttime = now
                seq += 1

                yield self.last

    def __del__(self):
        import pygame
        pygame.display.quit()
