import logging
import selectors
import time
from time import time
from collections import deque
import serial
from typing import Union, Tuple, List, Any, Dict

from .messages import *

logger = logging.getLogger('message')

# These parameters must match those in the  firmware,
# in idx_stepper.h
TIMEBASE = 1_000_000  # microseconds
N_BIG = 2 ** 32 - 1  # ULONG_MAX
N_AXES = 6
fp_bits = 8  # Bits in the fraction portion of the floating point representation

TERMINATOR = b'\0'

from dataclasses import dataclass


class TimeoutException(Exception):
    pass


class ProtocolException(Exception):
    pass


def _message_callback(proto, m):
    try:
        pl = m.payload.decode('utf8').strip()
    except:
        pl = m.payload

    if m.code == CommandCode.DEBUG:
        logger.debug(pl)
    elif m.code == CommandCode.ERROR:
        logger.error(pl)
    if m.code == CommandCode.ECHO:
        pass
    else:
        logger.info(pl)

    return pl


@dataclass
class AxisState():
    spos: int = None  # Stepper position
    epos: int = None  # Encoder position
    hl_limit: int = None
    lh_limit: int = None
    last_limit: int = None
    direction: int = None

    def __str__(self):
        d = '+' if self.direction else '-'
        return f"<AS {d} {self.spos}/{self.epos} hl{self.hl_limit} lh{self.lh_limit}"


class SyncProto(object):

    def __init__(self,
                 stepper_port, encoder_port=None, stepper_baud=115200, encoder_baud=115200,
                 message_callback=None, timeout=.1):

        self.step_ser = serial.Serial(stepper_port, baudrate=stepper_baud, timeout=timeout)

        if encoder_port is not None:
            self.enc_ser = serial.Serial(encoder_port, baudrate=encoder_baud, timeout=timeout)
        else:
            self.enc_ser = None

        if message_callback is None:
            message_callback = _message_callback

        self.message_callback = message_callback

        self.timeout = timeout

        self.encoder_multipliers = [1] * N_AXES

        self.empty = True
        self.seq = 0;
        self.last_ack = -1;
        self.last_done = -1;
        self.last_message_time = time(); # time of last message recieved

        self.running = False

        self._reset_states()

        self.sel = selectors.DefaultSelector()

        self.sel.register(self.step_ser, selectors.EVENT_READ, self.read_stepper_message)

        if self.enc_ser:
            self.sel.register(self.enc_ser, selectors.EVENT_READ, self.read_encoder_message)

        self.raw_queue = deque()
        self.raw_queue = deque() # handled queue

    def read_stepper_message(self, ser):
        data = ser.read_until(TERMINATOR)
        try:
            m = CommandHeader.decode(data[:-1])
            return m
        except Exception as e:
            print(e, data)
            return None

    def read_encoder_message(self, ser):
        data = ser.read_until(TERMINATOR)
        try:
            m = EncoderReport.decode(data[:-1])
            return m
        except Exception as e:
            print(e, data)
            return None

    def update_ser(self, timeout=False):
        '''Read all outstanding messages, handle them, and add them to the queue,'''

        events = self.sel.select(timeout)

        for key, mask in events:
            f, ser = key.data, key.fileobj
            m = f(ser)
            if m:
                self.last_message_time = time()
                if m.code in (CommandCode.ACK, CommandCode.ECHO):
                    self.last_ack = m.seq

                self.raw_queue.append(m)

        return len(events)

    def update_ser_until_ack(self, ack_seq):

        t0 = time()
        while(time()-t0 < 2):
            self.update_ser(.2)
            if self.last_ack == ack_seq:
                return True

        raise TimeoutException(f"Did not get ACK for seq_id {ack_seq}" )

    def update_ser_until_empty(self):

        while self.update_ser(.05):
            pass

    def update(self, cb= None, timeout=.5):
        """ For at least the duration of the timeout, read messages from the
        serial port andprocess them.

        :param cb: Callback function for messages
        :type cb: callable
        :param timeout: Minimum time, in seconds, to update and process messages
        :type timeout: number
        :return: None
        :rtype:
        """
        from inspect import signature

        if timeout is False:
            timeout == 0

        t0 = time()
        while True:
            self.update_ser_until_empty()

            while (len(self.raw_queue) > 0):
                m = self.raw_queue.popleft();

                r = self.handle_message(m)

                if cb:
                    sig = signature(cb)
                    if len(sig.parameters) == 3:
                        cb(self, m, r)
                    elif len(sig.parameters) == 2:
                        cb(self, m)
                    else:
                        cb(m)

            if time()-self.last_message_time>timeout:
                break;


    def handle_message(self, m):

        m.recieve_time = time()
        handled = False

        if m.code in (CommandCode.ACK, CommandCode.NACK, CommandCode.ECHO, CommandCode.ALIVE):
            # Handle the case where the seq ids wrap around
            self.last_ack = max( self.last_ack, m.seq) if abs(self.last_ack - m.seq) < 2**15 else m.seq

        if m.code in (CommandCode.ERROR, CommandCode.MESSAGE, CommandCode.ECHO):

            if self.message_callback:
                m.payload = self.message_callback(self, m)
            else:
                pass

            return True

        elif m.code in (CommandCode.ACK, CommandCode.DONE, CommandCode.EMPTY,
                        CommandCode.ZERO, CommandCode.ALIVE):

            m.payload = self.current_state = CurrentState(m.payload)
            self.empty = self.current_state.empty
            self.running = self.current_state.running

            for p, ax in zip(self.current_state.positions, self.axis_state):
                ax.spos = p

            if m.code == CommandCode.EMPTY:
                self.empty = True;
            elif m.code == CommandCode.DONE:
                self.last_done = m.seq

            return True

        elif m.code > CauseCode.START:

            self.encoder_state = m.encoders

            for i, (es, mult, ax) in enumerate(zip(self.encoder_state, self.encoder_multipliers, self.axis_state)):

                ax.epos = int(round(es.position * mult))

                if i == m.axis_code:

                    if es.limit_code == LimitCode.LH:
                        ax.lh_limit = ax.last_limit = ax.epos

                    elif es.limit_code == LimitCode.HL:
                        ax.hl_limit = ax.last_limit = ax.epos

            return True

        elif m.code in ( CommandCode.NACK, ):
            return True

        return False

    def runout(self, cb=None, timeout=False):

        self.update(cb, timeout)

    def runempty(self, cb=None, timeout=1):
        """ Update and process messages until the step controller
        reports and empty queue, or until timeout expires
        :param cb:
        :type cb:
        :param timeout: Expiration time, measured from the time the last message was recieved
        :type timeout:
        :return:
        :rtype:
        """

        if not self.running:
            self.run()

        while not self.empty:
            self.update(cb, .1)

            if time()-self.last_message_time > timeout:
                break

    def runlen(self, l, cb=None, timeout=False):
        """ Run until the quelength is less than l, or the timeout expires
        :param l:
        :type l:
        :param cb:
        :type cb:
        :param timeout:
        :type timeout:
        :return:
        :rtype:
        """

        if not self.running:
            self.run()

        while self.queue_length > l:
            self.update(cb, .1)

            if time() - self.last_message_time > timeout:
                break


    def _reset_states(self):

        self.axis_state = [AxisState() for _ in range(N_AXES)]

        self.current_state = CurrentState()
        self.encoder_state = [None] * N_AXES

    def close(self):

        self.sel.close()
        self.step_ser.close()
        if self.enc_ser:
            self.enc_ser.close()



    def iupdate(self, timeout=False):
        t = time()
        while True:
            self.update_ser()
            yield from self

            if timeout is not False and time() > t + timeout:
                break

    def __iter__(self):

        while True:
            try:
                yield self.raw_queue.popleft()
            except IndexError:
                return


    @property
    def queue_length(self):
        return self.current_state.queue_length

    @property
    def queue_time(self):
        return self.current_state.queue_time

    # Sending messages to the stepper controller
    #

    def send(self, m, timeout=False):

        m.send_time = time()
        self.seq += 1
        m.seq = self.seq

        b = m.encode()
        self.step_ser.write(b)

        self.update_ser_until_ack(m.seq)

        return m


    def send_command(self, c, payload=None, timeout=False):
        self.send(CommandHeader(seq=self.seq, code=c, payload=payload), timeout=timeout)

    def config(self, itr_delay: int = 4, segment_complete_pin=0, limit_hit_pin=0,
               debug_print: bool = False, debug_tick: bool = False,
               axes: List[AxisConfig] = []):

        # Send the top level config, to set the number of
        # axes
        self.send(ConfigCommand(len(axes), itr_delay, segment_complete_pin, limit_hit_pin,
                                debug_print, debug_tick))

        self.axes = axes

        # Then send the config for each axis.
        for ac in axes:
            self.send(ac)

    def _move(self, code: int, x: Union[List[Any], Tuple[Any], Dict], t=0):

        # Convert a dict-based move into an array move.
        if isinstance(x, dict):
            x_ = [0] * len(self.axes)
            for k,v in x.items():
                assert isinstance(k, int)
                x_[k] = v
            x = x_

        m = MoveCommand(code, x, t=t)

        m.done = False

        self.current_state.queue_length += 3;

        self.send(m)
        self.empty = False;

    def amove(self, x: Union[List[Any], Tuple[Any], Dict]):
        """Absolute position move"""
        self._move(CommandCode.AMOVE, x, t=0)

    def rmove(self, x: Union[List[Any], Tuple[Any], Dict]):
        "Relative position move"
        self._move(CommandCode.RMOVE, x, t=0)

    def hmove(self, x: Union[List[Any], Tuple[Any], Dict]):
        "A homing move, which will stop when it gets to a limit. "
        self._move(CommandCode.HMOVE, x, t=0)

    def vmove(self, t: float, x: Union[List[Any], Tuple[Any], Dict]):
        "A velocity move "
        self._move(CommandCode.VMOVE, x, t=t)

    def jog(self, t: float, x: Union[List[Any], Tuple[Any], Dict]):
        """Jog move. A jog move replaces the last move on the (step generator side)
        planner, then becomes a regular relative move. """
        self._move(CommandCode.JMOVE, x, t=t)

    def echo(self, payload='Echo'):
        self.running = True
        self.send_command(CommandCode.RUN, payload=payload)

    def noop(self):
        self.send_command(CommandCode.NOOP)

    def queue(self):
        self.send_command(CommandCode.QUEUE)

    def run(self):
        self.send_command(CommandCode.RUN)

    def stop(self):
        self.send_command(CommandCode.STOP)

    def info(self):
        self.send_command(CommandCode.INFO)


    def reset(self):
        self.runout()
        self._reset_states()
        self.send_command(CommandCode.RESET)
        self.runout()

    def zero(self):
        self.runout()
        self._reset_states()
        if self.enc_ser:
            self.enc_ser.write(b'z')
        self.send_command(CommandCode.ZERO)
        self.runout()

    def pollEncoders(self):
        self.enc_ser.write(b'p')

        for m in self.iupdate(1):
            if m.name == 'POLL':
                return m
