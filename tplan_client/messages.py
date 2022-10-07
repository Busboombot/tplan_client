from __future__ import print_function

import struct
from dataclasses import dataclass
from enum import IntEnum
from typing import List, Union

from cobs import cobs

from tplan_client.crc8 import crc8

TIMEBASE = 1e6


class ProtoError(Exception):
    pass


class SerialPacketError(ProtoError):
    pass


class CRCError(ProtoError):
    pass


class BadMoveCodeError(ProtoError):
    pass


class OutMode(IntEnum):
    OUTPUT = 1
    OUTPUT_OPENDRAIN = 4


class OutVal(IntEnum):
    HIGH = 1
    LOW = 0


class CommandCode(IntEnum):
    ACK = 1
    NACK = 2
    DONE = 3  # A Movement command is finished.
    EMPTY = 4  # Queue is empty, nothing to do.

    RMOVE = 11  # a normal movement segment
    AMOVE = 12  # a normal movement segment
    JMOVE = 13  # a normal movement segment
    HMOVE = 14  # a normal movement segment
    VMOVE = 15

    RUN = 21
    STOP = 22
    RESET = 23
    ZERO = 24
    SET = 25

    CONFIG = 31 # Reset the configuration
    AXES = 32 # Configure an axis

    MESSAGE = 91  # Payload is a message; the next packet is text
    ERROR = 92  # Some error
    ECHO = 93  # Echo the incomming header
    DEBUG = 94  # Echo the incomming header
    INFO = 95  # Send back info messages about state and condition
    QUEUE = 96  # Print out the movement queue
    SYNC = 96  # Sync state
    ALIVE = 98 # Step controller tells client it is still alive.
    NOOP = 99  # Does nothing, but does get ACKed

    def __repr__(self):
        return self.name

    def __str__(self):
        return self.name


class CommandHeader(object):
    # struct Header {
    #   uint16_t seq; // Packet sequence number
    #   enum Command code : 6; // Command number
    #   enum AckNack ack_nack: 2;
    #   enum ReqResp req_resp: 1;
    #   enum FlowControl flow_control: 2;
    #   uint8_t n_axes: 5; // Number of axes
    #   uint32_t crc = 0; // Payload CRC // 4
    # }; // 8 bytes

    msg_fmt = ('<H' +  # seq
               'B' +  # Code
               'B')  # CRC8

    size = struct.calcsize(msg_fmt)

    def __init__(self, seq, code, crc=0, payload=None):

        self.seq = seq  # Gets set when sent
        self.code = CommandCode(code)
        self.crc = crc

        self.acked = None  # Set to true after ACK is recieved, or False if Nacked

        self.ack = None  # ACK header
        self.nack = None  # Nack header

        self.send_time = None
        self.payload = payload

    @property
    def is_ack(self):
        return self.code == CommandCode.ACK

    @property
    def name(self):
        return self.code.name

    @staticmethod
    def unpack(data):
        seq, code, crc = struct.unpack(CommandHeader.msg_fmt, data)

        o = CommandHeader(seq, code, crc)

        return o

    def _pack(self, crc=0):

        msg = [self.seq, int(self.code), crc]

        # Build the packet without the CRC as zero
        try:
            return struct.pack(self.msg_fmt, *msg)
        except:
            print("Failed to build struct for :", msg)
            raise

    def encode(self):

        if self.payload:
            try:
                d = self.payload.encode('ascii')  # Guess it is a string
            except AttributeError:
                try:
                    d = self.payload.encode()
                except AttributeError:
                    d = self.payload
        else:
            d = bytearray()

        p = self._pack(0) + d
        crc = crc8(p)

        p = self._pack(crc) + d

        return cobs.encode(p) + bytearray([0])

    @classmethod
    def decode(cls, d):

        b = cobs.decode(d)
        d = bytearray(b)

        h = cls.unpack(d[:cls.size])

        d[3] = 0  # CRC is calculated with a zero here.
        that_crc = crc8(d)
        if h.crc != that_crc:
            raise CRCError(f"CRC Check failed: {h.crc} != {that_crc} ")

        h.payload = d[cls.size:]

        return h

    def __hash__(self):
        return hash((self.code, self.seq, self.crc))

    def __eq__(self, other):
        return (
                self.crc == other.crc and
                self.code == other.limit_code and
                self.seq == other.seq)

    def sid(self):
        """string ident"""
        return str(self)

    def __str__(self):
        return f"<ST #{self.seq} {str(self.code)} > "


class MoveCommand(object):
    msg_fmt = ('<' +
               'I' +  # segment_time
               '6i')  # steps

    size = struct.calcsize(msg_fmt)

    def __init__(self, code: int, x: List[int], t: float = 0):
        self.x = [int(e) for e in x] + [0] * (6 - len(x))
        self.t = int(round(t * TIMEBASE))  # Convert to integer microseconds

        if code not in (CommandCode.AMOVE, CommandCode.RMOVE,
                        CommandCode.JMOVE, CommandCode.HMOVE,
                        CommandCode.VMOVE):
            raise BadMoveCodeError("Bad Code {}".format(code))

        self.header = CommandHeader(seq=0, code=code)

    @property
    def seq(self):
        return self.header.seq

    @seq.setter
    def seq(self, v):
        self.header.seq = v

    def encode(self):

        self.header.payload = self.pack()

        return self.header.encode()

    def pack(self):
        try:

            return struct.pack(self.msg_fmt, self.t, *self.x)
        except:
            print(self.__dict__)
            raise

    def __repr__(self):
        return f"<Move {self.x} >"


class AxisConfig(object):
    msg_fmt = ('<' +
               'B' +  # Axis
               '3B' +  # step, dir, enable pins
               '3B' +  # High or Low for step, direction and enable
               '3B' +  # Output mode, OUTPUT or OUTPUT_OPENDRAIN
               '2B' +  # Padding
               'I' +  # v_max
               'I'  # a_max
               )

    size = struct.calcsize(msg_fmt)

    def __init__(self, axis_num: int, step_pin: int, direction_pin: int, enable_pin: int,
                 high_value: Union[tuple, int], output_mode: Union[tuple, int],
                 v_max: int, a_max: int):
        self.axis_num = axis_num

        self.mode = 0
        self.step_pin = step_pin
        self.direction_pin = direction_pin
        self.enable_pin = enable_pin
        self.high_value = high_value if isinstance(high_value, tuple) else (high_value,) * 3
        self.output_mode = output_mode if isinstance(output_mode, tuple) else (output_mode,) * 3

        self.v_max = int(v_max)
        self.a_max = int(a_max)

        self.header = CommandHeader(seq=0, code=CommandCode.AXES)

    @property
    def seq(self):
        return self.header.seq

    @seq.setter
    def seq(self, v):
        self.header.seq = v

    def encode(self):
        self.header.payload = struct.pack(self.msg_fmt,
                                          self.axis_num,
                                          self.step_pin, self.direction_pin, self.enable_pin,
                                          *self.high_value,
                                          *self.output_mode,
                                          0, 0,
                                          self.v_max, self.a_max)

        return self.header.encode()


class ConfigCommand(object):
    msg_fmt = ('<' +
               'B' +  # n_axes
               'B' +  # interrupt_delay
               'B' +  # Segment Complete Pin
               'B' +  # Limit hit pin
               'B' +  # uint8_t yellow_led_pin=0;
               'B' +  # uint8_t blue_led_pin=0;
               'B' +  # uint8_t running_led_pin=0;
               'B' +  # uint8_t empty_led_pin=0;
               'B' +  # uint8_t builtin_led_pin=0;
               'B' +  # debug_print
               'B'  # debug_tick
               )

    size = struct.calcsize(msg_fmt)

    def __init__(self, n_axes: int, itr_delay: int,
                 segment_complete_pin: int = 12, limit_hit_pin: int = 0,
                 debug_print: bool = False, debug_tick: bool = False):
        """

        :rtype: object
        """
        self.n_axes = n_axes
        self.itr_delay = itr_delay

        self.debug_print = debug_print
        self.debug_tick = debug_tick

        self.segment_complete_pin = segment_complete_pin
        self.limit_hit_pin = limit_hit_pin

        self.yellow_led_pin = 26
        self.blue_led_pin = 31
        self.running_led_pin = 28
        self.empty_led_pin = 30
        self.builtin_led_pin = 13

        self.header = CommandHeader(seq=0, code=CommandCode.CONFIG)

    @property
    def seq(self):
        return self.header.seq

    @seq.setter
    def seq(self, v):
        self.header.seq = v

    def encode(self):
        self.header.payload = struct.pack(self.msg_fmt,
                                          self.n_axes,
                                          self.itr_delay,
                                          self.segment_complete_pin,
                                          self.limit_hit_pin,
                                          self.yellow_led_pin,
                                          self.blue_led_pin,
                                          self.running_led_pin,
                                          self.empty_led_pin,
                                          self.builtin_led_pin,
                                          self.debug_print,
                                          self.debug_tick)

        return self.header.encode()


class CurrentState(object):
    msg_fmt = ('<' +
               'i' +  # queue_length
               'I' +  # queue_time
               '6i' +  # stepper_postitions
               '6i' + # planner_postitions
               'I' # Flags
               )

    size = struct.calcsize(msg_fmt)

    def __init__(self, b=None):

        if b:
            self.queue_length, self.queue_time, *positions, self.flags = struct.unpack(self.msg_fmt, b)
        else:
            self.queue_length, self.queue_time, *positions = 0, 0, []

        self.positions, self.planner_positions = positions[:6], positions[6:]

    @property
    def running(self):
        return bool(self.flags & 1)

    @property
    def empty(self):
        return bool(self.flags & 2)

    def __str__(self):
        return f"[ l{self.queue_length} t{round(self.queue_time/1e6, 4)} {self.positions} E={self.empty}  R={self.running}]"


encoder_msg_fmt = (
        'H' +  # seq
        'B' +  # code
        'b' +  # axis_index
        '6c' +  # limit_states[6]
        '6i'  # Positions[6]
)


class LimitCode(IntEnum):
    HL = 0b10
    LH = 0b01
    HH = 0b11
    LL = 0b00


class CauseCode(IntEnum):
    START = 100
    LIMIT = 101
    ZEROED = 102
    SEGDONE = 103
    POLL = 104


@dataclass()
class EncoderReport:
    axis_code: int
    code: CauseCode
    encoders: "EncoderState"

    def __str__(self):
        s = ' '.join(str(e) for e in self.encoders)
        return f"<ER {self.cause.name}@{self.axis_code} {s}> "

    @property
    def is_ack(self):
        return False

    @property
    def name(self):
        return self.cause.name

    @classmethod
    def decode(cls, data):

        encoders = []

        raise Exception("Check this code against the new message format")
        try:
            try:
                v = struct.unpack(encoder_msg_fmt, cobs.decode(data))
            except struct.error as e:
                print(e)
                print('Got data len=', len(data));
                raise

            cause = CauseCode(int.from_bytes(v[6], 'big'))
            axis = int.from_bytes(v[7], 'big')

            if axis == 0:
                axis = None
            else:
                axis -= 1

            for limit_states, positions in zip(v[:6], v[-6:]):
                ls = int.from_bytes(limit_states, "big")
                direction = (ls & 4) >> 2
                limit_code = LimitCode(ls & 3)

                encoders.append(EncoderState(limit_code, direction, positions))

            return EncoderReport(axis, cause, encoders)

        except Exception as e:
            raise


@dataclass
class EncoderState:
    """Class for keeping track of an item in inventory."""
    limit_code: LimitCode
    direction: int
    position: float

    def sid(self):
        """string ident"""
        return f"< ## {self.cause.name} {self.limit_code.name}>"

    def __repr__(self):
        return (f'E[{self.limit_code.name} {"<" if self.direction else ">"}{self.position}]')

    def __str__(self):
        return (f'{self.limit_code.name}{"<" if self.direction else ">"}{self.position}')
