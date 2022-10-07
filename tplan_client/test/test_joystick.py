import logging
import unittest

from tplan_client.messages import *
from tplan_client.joystick import *
from tplan_client.proto import SyncProto
from tplan_client.test import make_axes

logging.basicConfig(level=logging.DEBUG)
from tplan_client.messages import OutMode

packet_port = '/dev/cu.usbmodem_Busbot_ss0011'  # Test
encoder_port = None
baudrate = 115200  # 20_000_000


def cb(p, m):
    if m.code != CommandCode.ALIVE:
        print(m, "E=", p.empty, "R=", p.running, m.payload)

# Different maps for each max speed
freq_map = [
    mkmap(0, 1, 0, 5000),
    mkmap(0, 1, 0, 15000),
    #mkmap(0, 1, 0, 8000),
    #mkmap(0, 1, 0, 11000),
    #mkmap(0, 1, 0, 15000)
]

class TestJoystick(unittest.TestCase):

    def test_joystick(self):
        from time import time

        def cb(p, m):
            if m.name != 'MESSAGE':
                print(m)

        def get_js_move():
            for e in PygameJoystick(t=.15):
                button = max([0] + e.button)
                m = freq_map[ int(3 in e.trigger)]
                yield e, [int(m(a)) for a in e.axes]

        logging.basicConfig(level=logging.DEBUG)

        p = SyncProto(packet_port, None)

        d = make_axes(500, .1, usteps=10, steps_per_rotation=200)
        p.config(4, 18, 32, False, False, axes=d['left'])

        p.reset()
        p.run()

        last = time()
        for e, move in get_js_move():
            p.jog(.2, move[:3])
            print( round(time()-last, 3), move[:3])

            last = time()

        p.info()
        p.stop()

    def test_random_jog(self):

        from random import random

        def cb(p, m):
            print(m)

        p = SyncProto(packet_port, baudrate)

        d = make_axes(250, .2, usteps=32, steps_per_rotation=48)
        jog_interval = .5  # Secs between jog messages
        s = d['x_1sec'] * jog_interval  # Max steps between jog intervals, but for a bit longer than the

        p.config(4, True, False, False, axes=d['axes6']);
        p.run()

        for i in range(10_000_000):
            moves = [2 * (random() - .5) * s for i in range(6)]
            print(jog_interval * 1.5, moves)
            p.jog(jog_interval * 1.5, moves)
            sleep(jog_interval)

        p.read_empty(cb);

        p.info()
        p.stop()

    def test_joy_move(self):

        def get_joy():
            while True:
                with open('/tmp/joystick') as f:
                    return [float(e) for e in f.readline().split(',')]

        last_time = time()
        last_velocities = [0] * 6
        seq = 0

        while True:

            e = get_joy()

            dt = time() - last_time

            if dt >= .20 and len(proto) <= 2:
                last_time = time()

                velocities = e + [0, 0]

                x = [.5 * (v0 + v1) * dt for v0, v1 in zip(last_velocities, velocities)]

                msg = Command(seq, 10, dt * 1e6, last_velocities, velocities, x)

                # proto.write(msg)

                seq += 1

                last_velocities = velocities
            elif dt < .20:
                sleep(.20 - dt)


if __name__ == '__main__':
    unittest.main()
