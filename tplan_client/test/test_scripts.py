import logging
import unittest

from tplan_client.messages import *
from tplan_client.proto import SyncProto
from tplan_client.test import make_axes

packet_port = '/dev/cu.usbmodem_Busbot_ss0011'  # Test
encoder_port = None
baudrate = 115200  # 20_000_000

logging.basicConfig(level=logging.DEBUG)


# Axis configurations for the robot
# Tuples are: step pin, dir pin, enable pin, max_v, max_a

def cb(p, m):
    # if m.code != CommandCode.ALIVE:
    print(m, "E=", p.empty, "R=", p.running, m.payload)


class TestSerial(unittest.TestCase):
    # Determines whether the steppers are enables with an output value of high or low
    # Different for different stepper drivers
    ENABLE_OUTPUT = False

    def setUp(self) -> None:
        logging.basicConfig(level=logging.DEBUG)

    def tearDown(self) -> None:
        pass

    def init(self, v=800, axes_name='axes1', usteps=16, a=.1,
             highvalue=OutVal.HIGH, outmode=OutMode.OUTPUT,
             segment_pin=27, limit_pint=29, period=4,
             use_encoder=True):
        d = make_axes(v, a, usteps=usteps, steps_per_rotation=200,
                      highval=highvalue, output_mode=outmode)

        p = SyncProto(packet_port, encoder_port if use_encoder else None)
        p.encoder_multipliers[0] = 1 + (1 / 3)

        p.config(period, segment_pin, limit_pint, False, False, axes=d[axes_name]);

        p.mspr = d['mspr']
        p.x_1sec = d['x_1sec']

        return p

    def config(self, key):
        p = SyncProto(packet_port, None)
        d = make_axes(500, .1, usteps=10, steps_per_rotation=200)
        p.config(4, 18, 32, False, False, axes=d[key])

        return p;

    def test_echo(self):
        """Test changing the configuration"""

        p = SyncProto(packet_port, None)
        p.send_command(CommandCode.ECHO, 'This is the payload')

        p.runout(lambda p, m: print(m, m.payload), timeout=1)

    def test_noop(self):
        """Test changing the configuration"""
        p = SyncProto(packet_port, None)
        p.noop()
        p.update(cb)

    def test_run(self):
        """Test changing the configuration"""
        p = SyncProto(packet_port, None)
        p.run()

    def test_stop(self):
        """Test changing the configuration"""
        p = SyncProto(packet_port, None)
        p.stop()

    def test_queue(self):
        """Test changing the configuration"""

        def cb(p, m, handled):
            print(m, "E=", p.empty, "R=", p.running, m.payload)

        p = SyncProto(packet_port, None)
        p.queue()

        p.update(cb, 1)

    def test_info(self):
        """Test changing the configuration"""

        def cb(p, m, handled):
            print(m, handled)

        p = SyncProto(packet_port, None, timeout=1)

        p.info()

        p.update(cb, 1)

    def test_config(self):
        """Test changing the configuration"""

        p = self.config('right')

        p.info()

        p.update()

    def test_runout(self):
        """Test changing the configuration"""

        def cb(p, m, handled):
            print(m, "E=", p.empty, "R=", p.running, m.payload)

        p = SyncProto(packet_port, None)

        p.run()
        p.runempty(cb)

    def test_load_moves(self):
        def cb(p, m):
            print(m, p.queue_time)

        d = make_axes(1000, 1, usteps=10, steps_per_rotation=200)

        p = SyncProto(packet_port, None, baudrate)
        p.config(4, self.ENABLE_OUTPUT, False, False, axes=d['right']);

        p.stop()
        s = d['x_1sec'] / 2

        for i in range(4):
            p.rmove((s, s, s))
            p.rmove((-s, -s, -s))

        p.stop()
        p.info()

    def test_simple_rmove(self):
        """A simple move with 1 axis"""

        logging.basicConfig(level=logging.DEBUG)

        d = make_axes(1000, 1, usteps=10, steps_per_rotation=200)

        p = SyncProto(packet_port, None, baudrate)
        p.config(4, self.ENABLE_OUTPUT, False, False, axes=d['left']);

        p.info()
        p.run()
        s = d['x_1sec'] * 2

        for i in range(4):
            p.rmove((s, s, s))
            p.rmove((-s, -s, -s))

        p.runempty(cb)

    def test_simple_jog(self):
        """A simple move with 1 axis"""
        from time import sleep

        logging.basicConfig(level=logging.DEBUG)

        p = SyncProto(packet_port, None, baudrate)
        d = make_axes(1000, 1, usteps=10, steps_per_rotation=200)
        p.config(4, self.ENABLE_OUTPUT, False, False, axes=d['left']);

        p.info()
        p.run()
        s = 20_000
        t = .2

        p.run()

        def stepped_v():
            p.vmove(t, [s] * 3)
            p.vmove(t, [-s] * 3)
            p.vmove(t, [s / 2] * 3)
            p.vmove(t, [-s / 2] * 3)
            p.vmove(t, [s / 4] * 3)
            p.vmove(t, [-s / 4] * 3)
            p.vmove(t, [s / 10] * 3)
            p.vmove(t, [-s / 10] * 3)

        moves = [[-500 * i] * 3 for i in range(0, 15, 2)]
        moves = moves + moves[::-1]
        # for m in moves:

        for i in range(4):
            p.vmove(.5, [0, 0, 10000])
            p.vmove(.5, [0, 0, -10000])
            sleep(.4)
            # p.update(cb)

        p.runempty(cb);
        p.info()


if __name__ == '__main__':
    unittest.main()
