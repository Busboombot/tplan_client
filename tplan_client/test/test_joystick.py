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


# Different maps for each max speed
freq_map = [
    mkmap(0, 1, 0, 5000),
    mkmap(0, 1, 0, 10000),
    # mkmap(0, 1, 0, 8000),
    # mkmap(0, 1, 0, 11000),
    # mkmap(0, 1, 0, 15000)
]


class TestJoystick(unittest.TestCase):
    def init(self, v=800, axes_name='axes1', usteps=16, a=.1,
             highvalue=OutVal.HIGH, outmode=OutMode.OUTPUT,
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

    def test_enumerate(self ):
        import hid

        for device in hid.enumerate():
            print(f"0x{device['vendor_id']:04x}:0x{device['product_id']:04x} {device['product_string']}")

    def test_hidjoystick(self ):

        print(HidJoystick.find_first_joystick())

        j = HidJoystick()

        last = None
        for e in j:
            if True or e != last:
                print(e.moves)
                last = e

    def test_joystick(self):
        from time import time

        j = HidJoystick(interval=0.1)

        logging.basicConfig(level=logging.DEBUG)

        p = self.init(2000, a=1, usteps=10, axes_name='axes6',
                      debug_print=False,
                      outmode=OutMode.OUTPUT_OPENDRAIN);

        p.reset()
        p.run()

        last = time()
        while True:
            jv = j.next_until(.01)

            d_t = time()-last
            last = time()

            move = jv.moves

            p.jog(.2, move)

            print(round(d_t, 3), move, p.current_state.queue_length)

            p.update(timeout=0)
            #sleep(.05)

        p.info()
        p.stop()

    def test_jog_moves(self ):

        mf = [ 500, 0, 0, 0, 0, 0]
        mr = [-500, 0, 0, 0, 0, 0]

        logging.basicConfig(level=logging.DEBUG)

        p = self.init(2000, a=1, usteps=10, axes_name='axes6',
                      debug_print=False,
                      outmode=OutMode.OUTPUT_OPENDRAIN);

        p.reset()
        p.run()

        def run_move(m_0, sleep_t=.2):

            for i in list(range(1, 10)) + list(range(15, 5,-1)):
                m = (np.array(m_0) * i).tolist()
                p.jog(.4, m)
                p.update(timeout=0)

                sleep(sleep_t)



        last = time()
        for e in range(2):
            run_move(mf)
            run_move(mr)
            run_move(mr)
            run_move(mf)

        p.info()
        p.stop()


if __name__ == '__main__':
    unittest.main()
