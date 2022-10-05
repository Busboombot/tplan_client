import logging
import unittest

from tplan_client.messages import *
from tplan_client.proto import SyncProto
from tplan_client.test import make_axes

# packet_port = '/dev/cu.usbmodem64213801'  # Production
packet_port = '/dev/cu.usbmodem_Busbot_ss0011'  # Test

encoder_port = None
# encoder_port = '/dev/cu.usbmodem6387471'
# encoder_port = '/dev/cu.usbmodem63874601'  # Production

baudrate = 115200  # 20_000_000

logging.basicConfig(level=logging.DEBUG)


# Axis configurations for the robot
# Tuples are: step pin, dir pin, enable pin, max_v, max_a

class TestSerial(unittest.TestCase):
    # Determines whether the steppers are enables with an output value of high or low
    # Different for different stepper drivers
    ENABLE_OUTPUT = False

    def setUp(self) -> None:
        pass

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

    def test_echo(self):
        """Test changing the configuration"""

        p = SyncProto(packet_port, None)
        p.send_command(CommandCode.ECHO, 'This is the payload')

        p.runout(lambda p, m: print(m, m.payload), timeout=1)

    def test_run(self):
        """Test changing the configuration"""

        p = SyncProto(packet_port, None)
        p.run()

    def test_stop(self):
        """Test changing the configuration"""

        p = SyncProto(packet_port, None)
        p.stop()

    def test_info(self):
        """Test changing the configuration"""

        def cb(p, m, handled):
            print(m, handled)

        p = SyncProto(packet_port, None, timeout=1)

        p.info()

        p.update(cb, 1)

    def test_config(self):
        """Test changing the configuration"""

        p = SyncProto(packet_port, None)

        d = make_axes(500, .1, usteps=10, steps_per_rotation=200)
        p.config(4, 18, 32, False, False, axes=d['axes3']);
        p.info()

        p.info()

        p.update()

    def test_simple_rmove(self):
        """A simple move with 1 axis"""

        def cb(p, m):
            print(m, p.queue_time)

        logging.basicConfig(level=logging.DEBUG)

        d = make_axes(1000, 1, usteps=10, steps_per_rotation=200)

        p = SyncProto(packet_port, None, baudrate)
        p.config(4, self.ENABLE_OUTPUT, False, False, axes=d['right']);

        p.info()
        p.run()
        s = d['x_1sec']

        for i in range(4):
            p.rmove((s, s, s))
            p.rmove((-s, -s, -s))
            p.runout(cb, timeout=1);

        p.info()
        p.stop()


if __name__ == '__main__':
    unittest.main()
