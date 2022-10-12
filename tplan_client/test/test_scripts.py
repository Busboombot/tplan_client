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

        p = self.init(1000, usteps=10, axes_name='axes6',
                      debug_print=True,
                      outmode=OutMode.OUTPUT_OPENDRAIN);

        p.info()

        # p.update()

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
        p.init(axes=d['right']);

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

        p = self.init(1000, a=1, usteps=10, axes_name='axes6',
                      debug_print=False,
                      outmode=OutMode.OUTPUT_OPENDRAIN);

        p.reset()

        p.info()
        p.stop()

        s = p.x_1sec * 1
        #s = 8000
        n = len(p.axes)
        m1 = [s * 1, s * 1, s * 1, s * 1, s * 1, s * 1]  # [s]*n
        m2 = [-e for e in m1]

        for i in range(1):
            p.rmove(m1)
            p.rmove(m2)

        p.run()
        p.runempty(cb)

    def test_simple_jog(self):
        """A simple move with 1 axis"""
        from time import sleep

        logging.basicConfig(level=logging.DEBUG)

        p = self.init(2000, a=1, usteps=10, axes_name='axes6',
                      debug_print=False,
                      outmode=OutMode.OUTPUT_OPENDRAIN);

        p.reset()

        p.info()
        p.run()

        s = 10000
        n = len(p.axes)
        m1 = [s * 1]*n

        for i in range(100):
            p.jog(.2, m1)
            p.jog(.2, m1)
            p.jog(.2, m1)
            sleep(.2)
            p.update(cb, timeout=0)

        p.runempty(cb)


if __name__ == '__main__':
    unittest.main()
