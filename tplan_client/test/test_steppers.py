import logging
import unittest

from tplan_client.messages import *
from tplan_client.joystick import *
from tplan_client.proto import SyncProto
from tplan_client.test import make_axes

import numpy as np

logging.basicConfig(level=logging.DEBUG)
from tplan_client.messages import OutMode

packet_port = '/dev/cu.usbmodem_Busbot_ss0011'  # Test
encoder_port = None
baudrate = 115200  # 20_000_000


def cb(p, m):
    if m.code != CommandCode.ALIVE:
        print(m,  m.payload)



class TestSteppers(unittest.TestCase):

    def init(self, v=700, axes_name='axes1', usteps=16, a=.1,
             highvalue=OutVal.HIGH, outmode=OutMode.OUTPUT_OPENDRAIN,
             debug_print=False, debug_tick=False,
             segment_pin=27, limit_pint=29, period=4,
             use_encoder=True):

        d = make_axes(v, a, usteps=usteps, steps_per_rotation=200,
                      highval=highvalue, output_mode=outmode)

        p = SyncProto(packet_port, encoder_port if use_encoder else None)
        p.encoder_multipliers[0] = 1 + (1 / 3)

        p.config(period, segment_pin, limit_pint, debug_print,
                 debug_tick, axes=d[axes_name]);

        p.mspr = d['mspr']
        p.x_1sec = d['x_1sec']

        return p

    def config(self, key):
        p = SyncProto(packet_port, None)
        d = make_axes(500, .1, usteps=10, steps_per_rotation=200)
        p.config(4, 18, 32, False, False, axes=d[key])

        return p;

    def test_hidjoystick(self ):

        j = HidJoystick()

        last = None
        for e in j:
            if True or e != last:
                print(e.moves)
                last = e

    def test_rmove6(self):


        logging.basicConfig(level=logging.DEBUG)

        p = self.init(700, a=4, usteps=10, axes_name='axes6',
                      debug_print=False,
                      outmode=OutMode.OUTPUT_OPENDRAIN);

        p.reset()
        p.run()

        def mmult(m, f):
            return [ e*f for e in m]

        dist = 25_000 #  253_000*2
        m = [dist]*6
        mn = mmult(m, -2)

        try:
            for i in range(2):
                p.rmove(m)
                p.rmove(mn)
                p.rmove(m)
                #p.runempty(cb, 10)

            p.runempty(cb, 20)
            p.info()
        except:
            p.stop()
            p.reset()
            raise


        #p.stop()

    def test_fake_joystick(self):
        from time import time

        def get_js_move():
            for e in PygameJoystick(t=.1):
                button = max([0] + e.button)
                m = freq_map[int(3 in e.trigger)]
                yield e, [int(m(a)) for a in e.axes]

        logging.basicConfig(level=logging.DEBUG)

        p = self.init(2000, a=1, usteps=10, axes_name='axes6',
                      debug_print=False,
                      outmode=OutMode.OUTPUT_OPENDRAIN);

        p.reset()
        p.run()

        move = [10_000, 100, 0, 0, 0, 0 ]

        for i in range(100):
            p.jog(.25, move)
            p.update(timeout=0)
            sleep(.1)


        p.info()
        p.stop()




if __name__ == '__main__':
    unittest.main()
