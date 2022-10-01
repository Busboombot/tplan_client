#
"""

Create a list of movements for a set of axes, then calculate
an optimum trapezoidal velocity profile.

Each movement is a segment, a period of time in which all axes
must start and stop at the same time. The segments have three periods:

    a: Acceleration
    c: Constant velocity
    d: Deceleration

( the names for the A and D periods refer to an idealized trapezoidal profile,
where a is the first part of the profile and d is the last. in actual cases, a
may have a deceleration, and d an acceleration. The c period is always 0
acceleration )

The optimization procedure attempts to execute every segment in the minimum
time, by ensuring that the longest axis is accelerate to the max velocity
as the max acceleration, and that there is a minimal change in velocity between
adjacent segments.

The algorithms for SimSegment and the c and cn parameters are based on
the article, 'Generate stepper-motor speed profiles in real time',
from https://www.embedded.com/generate-stepper-motor-speed-profiles-in-real-time/

------

Joint segment shapes



"""
from collections import deque
from dataclasses import dataclass, asdict, replace
from typing import List

import pandas as pd

from .isolver import SplitInfo


def binary_search(f, v_min, v_guess, v_max):
    for i in range(20):

        x = f(v_guess)

        if round(x) > 0:
            v_guess, v_min = (v_max + v_guess) / 2, v_guess

        elif round(x) < 0:
            v_guess, v_max = (v_min + v_guess) / 2, v_guess

        else:
            return v_guess

        if abs(v_min - v_max) < .05:
            return v_guess

    else:
        return None


def accel_xt(v_i, v_f, a):
    """Distance and time required to accelerate from v0 to v1 at acceleration a"""

    if v_f == v_i:
        return 0, 0

    if v_f < v_i:
        a = -a

    t = (v_f - v_i) / a  # Time to change from v0 to v1 at max acceleration
    x = (v_i + v_f) / 2 * t

    return x, t  # Area of velocity trapezoid


def accel_acd(v_0, v_c, v_1, a):
    """ Same result as running accel_xt, for v_0->v_c and v_c->v_1,
    and adding the x and t values.
    """
    t_ad = (abs(v_c - v_0) + abs(v_c - v_1)) / a
    x_ad = abs((v_0 ** 2 - v_c ** 2) / (2 * a)) + abs((v_1 ** 2 - v_c ** 2) / (2 * a))

    return x_ad, t_ad


def consistantize(b, return_error=False):
    """Recalculate t to make x and v values integers, and everything more consistent
     This operation will maintain the original value for x, but may change t"""

    b.x_a = b.x_a
    b.x_d = b.x_d
    b.x_c = b.x - (b.x_a + b.x_d)

    b.t_a = abs((b.v_c - b.v_0) / b.joint.a_max)
    b.t_d = abs((b.v_c - b.v_1) / b.joint.a_max)

    if round(b.x_c) == 0:  # get rid of small negatives
        b.x_c = 0

    if b.v_c != 0:
        b.t_c = abs(b.x_c / b.v_c)
        # assert b.t_c >= 0, (b.v_c, b.t_c, b.x_c)
    else:
        b.t_c = 0

    b.t = b.t_a + b.t_c + b.t_d

    # Check error against area calculation
    if return_error:
        x_ad, t_ad = accel_acd(b.v_0, b.v_c, b.v_1, b.joint.a_max)
        t_c = round(b.t - t_ad, 8)  # Avoid very small negatives
        x_c = b.v_c * t_c
        x = x_ad + x_c
        x_e = x - b.x
        return b.x_c, x_e
    else:
        return b.x_c


def sign(x):
    if x == 0:
        return 0
    elif x > 0:
        return 1
    else:
        return -1


def same_sign(a, b):
    return int(a) == 0 or int(b) == 0 or sign(a) == sign(b)


@dataclass
class Joint:
    v_max: float
    a_max: float
    # distances below the small x limit will never hit v_max before needing to decelerate
    small_x: float = None  # min distance for v_max->0->v_max
    max_discontinuity: float = None  # Max velocity difference between adjacent blocks

    def __post_init__(self):
        self.small_x = (self.v_max ** 2) / (self.a_max)
        self.max_discontinuity = self.a_max / self.v_max  # Max vel change in 1 step

    def new_block(self, x, v_0=None, v_1=None):
        v_0 = v_0 if v_0 is not None else self.v_max
        v_1 = v_1 if v_1 is not None else self.v_max

        return Block(x=x, v_0=v_0, v_1=v_1, joint=self)

    @property
    def dx_max(self):
        return (self.v_max ** 2) / (self.a_max)

    @property
    def dt(self):
        dt = self.dx_max / j.v_max


@dataclass
class Block:
    x: float = 0
    t: float = 0
    v_m: float = 0
    v_0: float = 0
    v_1: float = 0
    d: int = 0  # direction, -1 or 1

    joint: Joint = None
    segment: 'Segment' = None
    next: 'Block' = None
    prior: 'Block' = None

    flag: str = None
    recalcs: int = 0

    _param_names = ['x', 't', 'v_0', 'v_c', 'v_1']

    @property
    def id(self):
        return f"{self.segment.n}/{self.joint.n}"

    def __post_init__(self):
        self.d = sign(self.x)
        self.x = abs(self.x)

    def asdict(self):
        return asdict(self)

    def replace(self, **kwds):
        return replace(self, **kwds)

    @property
    def area(self):
        """Calculate the distance x as the area of the profile. Ought to
        always match .x """

        return 0

    def str(self):
        from colors import color, bold

        v0 = color(f"{int(self.v_0):<5d}", fg='green')
        x = bold(f"{int(self.d * self.x):>6d}")
        vm = color(f"{int(self.v_m):<6d}", fg='blue')
        v1 = color(f"{int(self.v_1):>5d}", fg='red')

        return f"[{v0} {x + '@' + vm} {v1}]"

    def _repr_pretty_(self, p, cycle):
        p.text(self.str() if not cycle else '...')


class AxisSegment:
    n: int = 0
    segment: 'Segment' = None
    joint: 'Joint' = None
    split_info: 'SplitInfo' = None
    blocks: List[Block] = None
    slices: List[float] = None

    dt: float = 0

    x: int = 0
    v_0: float = 0
    v_1: float = 0

    def __init__(self, axis_num: int, segment: "Segment", joint: "Joint",
                 split_info: SplitInfo, x: int, v_0: float, v_1: float) -> None:
        self.axis_num = axis_num
        self.segment = segment
        self.joint = joint
        self.split_info = split_info
        self.v_0 = v_0 if v_0 else 0
        self.v_1 = v_1 if v_1 else 0

        # THe following are all redundant with split_info now

        self.x = x

    def solve(self):
        from .isolver import ipsolve, triplets, with_limits
        self.slices = ipsolve(self.split_info, self.x, self.v_0, self.v_1)

        lwl = with_limits(self.slices, self.v_0, self.v_1, self.split_info.dt)

        self.blocks = [self.make_block(p, c, n, self.split_info.dt, self.joint) for (p, c, n) in triplets(lwl)]

        return self

    @property
    def v_m(self):
        return self.x / self.split_info.t

    @staticmethod
    def make_block(px, cx, nx, dt, joint):
        pv = px / dt
        cv = cx / dt
        nv = nx / dt
        return Block(x=cx, v_m=cv, v_0=(pv + cv) / 2, v_1=(cv + nv) / 2, t=dt, joint=joint)

    def _repr_pretty_(self, p, cycle):
        p.text(str(self) if not cycle else '...')

    def __str__(self):
        return '\n'.join(b.str() for b in self.blocks)


class Segment(object):
    """One segment, for all joints"""

    n: int = None
    next: "Segment" = None
    prior: "Segment" = None
    joints: List["Joint"] = None
    split_info: SplitInfo = None

    n_updates: int = 0
    move: List[int] = None
    axes: List[AxisSegment] = None

    boundary_velocities_0 = None
    boundary_velocities_1 = None

    def __init__(self, n, move: List[int], joints: List["Joint"], prior=None):

        self.n = n
        self.move = move
        self.joints = joints

        assert len(self.joints) == len(self.move)

        if prior:
            Segment.link(prior, self)

        self.boundary_velocities_0 = [0] * len(joints)
        self.boundary_velocities_1 = [0] * len(joints)

    @classmethod
    def link(cls, prior: "Segment", current: "Segment"):

        prior.next = current
        current.prior = prior

        # for p, c in zip(prior.axes, current.axes):
        #    p.next = c
        #    c.prior = p

    def solve(self, bv_0=None, bv_1=None):
        """Solve all of the axes"""
        from .isolver import ipsplit, longest_index
        from .gsolver import ACDBlock

        if bv_0 is None:
            if self.prior:
                bv_0 = self.prior.boundary_velocities_1
            else:
                bv_0 = [0] * len(self.joints)

        if bv_1 is None:
            if self.next:
                bv_1 = self.next.boundary_velocities_0
            else:
                bv_1 = [0] * len(self.joints)

        lng_idx = longest_index(self.move)
        lng_move = max(self.move)
        lng_joint = self.joints[lng_idx]

        b = ACDBlock(lng_move, v_0=0, v_1=0, joint=lng_joint).init()

        # Split the longest axis to get the number of  divisions
        self.split_info = ipsplit(lng_move, t=b.t, j=lng_joint)

        self.axes = [AxisSegment(i, self, joint, self.split_info, x, v_0, v_1).solve()
                     for i, (x, joint, v_0, v_1) in enumerate(zip(self.move, self.joints, bv_0, bv_1))]

        self.boundary_velocities_0 = [a.blocks[0].v_0 for a in self.axes]
        self.boundary_velocities_1 = [a.blocks[-1].v_1 for a in self.axes]

    @property
    def t(self):
        return self.split_info.t

    def __iter__(self):
        return iter(self.blocks)

    def __getitem__(self, item):
        return self.axes[item]

    def __str__(self):

        o = []

        for blocks in zip(*[a.blocks for a in self.axes]):
            o.append(f"{self.t:>3.4f}|" + ' '.join(b.str() for b in blocks))

        return '\n'.join(o)

    def _repr_pretty_(self, p, cycle):
        p.text(str(self) if not cycle else '...')

    def plot(self, ax=None):
        from .plot import plot_trajectory
        plot_trajectory(self.dataframe, ax=ax)

    @property
    def dataframe(self):
        from .plot import plot_params_df
        return plot_params_df(*list(zip(*[a.blocks for a in self.axes])))

    def sim(self, t0=0):
        o = [[ss.sim(t0) for ss in b.subsegments] for b in self.blocks]

        return list(zip(*o))  # Transpose


@dataclass
class SubSegment:
    """A sub segment is a portion of the trajectory with a constant
    acceleration,  one of the aceleration, constant (cruise) or decleration portions. """

    id: str
    segment_n: int
    joint: "Joint"
    js: "Block"

    t: float
    v_i: float
    v_f: float
    x: float
    ss: float
    direction: int = 1

    def __post_init__(self):
        assert self.v_i == 0 or self.v_f == 0 or sign(self.v_i) == sign(self.v_f), \
            f"Inconsistent directions {self.v_i} {self.v_f} for {self.id}{self.ss} "

    def set_direction(self, sign) -> None:
        self.direction = sign
        self.v_i *= sign
        self.v_f *= sign
        self.x *= sign

    def __repr__(self):
        return f"<{self.joint.n} {self.ss} {self.t:2.5f} {int(self.x):5d} {self.v_i:5.0f}->{self.v_f:5.0f}> "

    @property
    def row(self):
        return [self.segment_n, self.joint.n, self.x,
                self.direction * self.v_i, self.direction * self.v_f,
                self.ss, self.t]

    def sim(self, t0=0):
        from .sim import SimSegment

        return SimSegment(self.v_i, self.v_f, x=self.x, t=self.t, t0=0)


class SegmentList(object):
    positions: List[float]  # Positions after last movement addition
    segments: deque[Segment]
    all_segments: List[Segment]  # All of the segments, unprocessed
    planr_calls = 0

    def __init__(self, joints: List[Joint]):

        self.joints = [Joint(j.v_max, j.a_max, i) for i, j in enumerate(joints)]

        self.directions = [0] * len(self.joints)

        for i, j in enumerate(self.joints):
            j.n = i

        self.segments = deque()
        self.positions = [0] * len(self.joints)

    def move(self, x: List[int]):
        """Add a new segment, with joints expressing joint distance
        :type x: object
        """

        sl = len(self.segments)
        prior = self.segments[-1] if sl > 0 else None

        s = Segment(sl, x, self.joints, prior)

        bv_max = [j.v_max for j in self.joints]
        bv_zero = [0 for _ in self.joints]

        if prior:
            # solve the prior with v_mac for v_1, but small axes may end up with lower values
            # so, we solve the tail with the same values
            pbv0 = prior.boundary_velocities_0
            sbv1 = bv_zero

            prior.solve(bv_0=pbv0, bv_1=bv_max)

            for i in range(4):
                s.solve(bv_0=prior.boundary_velocities_1, bv_1=bv_zero)
                prior.solve(bv_0=pbv0, bv_1=s.boundary_velocities_0)


        else:
            s.solve(bv_0=bv_zero, bv_1=bv_zero)

        self.segments.append(s)



    @property
    def dataframe(self):

        df = pd.concat([s.dataframe for s in self.segments])

        df['t'] = df.groupby('axis').del_t.cumsum()
        df['calc_x'] = (df.v_i + df.v_f) / 2 * df.del_t
        df['err'] = df.x - df.calc_x

        return df

    def joint_pairs(self, index=0):
        """Yield all valid adjacent segment"""

        for s in list(self.segments)[index:]:
            for b in s.blocks:
                if b.next:
                    yield (b, b.next)

    def discontinuities(self, index=0):
        """Yield segment pairs with velocity discontinuities"""
        for c, n in self.joint_pairs(index):

            if round(c.v_1, 2) != round(n.v_0, 2):
                yield c, n

    def has_discontinuity(self, s1, s2):

        for js1, js2 in zip(s1, s2):
            if js1.v_1 != js2.v_0:
                return True

    @property
    def dataframe_stacked(self):
        return self.dataframe.set_index(['t', 'axis']).stack().unstack(-2).unstack(-1)

    @property
    def params_df(self):
        """
        :return: a dataframe of paramaters
        :rtype:
        """

        frames = []
        for i, s in enumerate(self.segments):
            frames.append(s.params_df)

        df = pd.concat(frames)

        return df

    def __getitem__(self, item):
        """Return a joint segment by the id"""
        try:
            s, js = item
            return self.segments[s].blocks[js]
        except TypeError:
            return self.segments[item]

    def __iter__(self):
        return iter(self.segments)

    def __len__(self):
        return len(self.segments)

    def __str__(self):
        return '\n'.join(str(s) for s in self.segments)

    def _repr_pretty_(self, p, cycle):
        p.text(str(self) if not cycle else '...')

    def debug_str(self):
        return '\n'.join(s.debug_str() for s in self.segments)

    def plot(self, ax=None, axis=None):
        from .plot import plot_trajectory

        df = self.dataframe

        if axis is not None:
            df = df[df.axis == axis]

        plot_trajectory(df, ax=ax)
