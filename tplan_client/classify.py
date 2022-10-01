from enum import Enum


class JSClass(Enum):
    """

    === Convex

    ╱▔▔ U Acel. V0 fixed, V1 is maxed
          Should be a typical case.

    ╱╲  A Triangle. Fixed V0, V1, too short for vc
          x is non zero but is too short got velocity to reach v_max

    ╱▔╲ T Trapezoid.  V0 & V1 == 0, max vc
          Normal case where v0 and v1 are both below Vc, and the segment isn't short.

    ╱▔╲ P Pentagon.  Fixed V0, V1, but not zero, max vc
          Normal case where v0 and v1 are both below Vc, and the segment isn't short.

    ▔▔╲ L Clif. Next segment is very small x.
          Next segment is very small x or zero



    === Flat

    ▔▔▔ C Constant V. Prior and next have high V0, V1
          V0 and V1 are both at vmax, so v_c should be v_max

    ▁▁▁ Z Zero distance
          Zero x means that there must bt v=0 through whole segment

    === Concave

    ╲▁╱ V Trough. Like a pentagon, but Vc is below either V0 or V1

    ╲▁▁ R Ramp down. Very small x relative to other joints.
          X is so short relative to other joints that we hit v=0
          during accel phase.


    """
    ACEL = 'U'
    TRIANGLE = 'A'
    PENTAGON = 'P'
    HEXAGON = 'H'
    AT = 'AT'  # A or T, pinned to Zero at start and finish
    TRAPZEZOID = 'T'
    TROUGH = 'V'
    CONSTANT = 'C'
    RAMP = 'R'
    DECEL = 'D'
    CLIFF = 'L'
    DL = 'DL'  # D or L
    ZERO = 'Z'
    XXXXX = 'X'
    UNK = '?'


kind_icon_map = {

    JSClass.TRIANGLE: '╱╲',
    JSClass.TRAPZEZOID: '╱-╲',
    JSClass.PENTAGON: '╱▔╲',
    JSClass.HEXAGON: '╱▔╲',
    JSClass.TROUGH: '╲▁╱',
    JSClass.CONSTANT: '▔▔▔ ',
    JSClass.ZERO: '▁▁▁',
    JSClass.ACEL: '╱▔▔',
    JSClass.DECEL: '╲',
    JSClass.RAMP: '╲▁▁',
    JSClass.CLIFF: '▔▔╲',
    JSClass.XXXXX: '▁▁╱',
    JSClass.UNK: '?'
}


def classify(p):
    """Assign a class type to a planner block"""
    from operator import attrgetter
    ag = attrgetter(*'x v_0 v_c t_c v_1 v_max a_max'.split())

    x, v_0, v_c, t_c, v_1, v_max, a_max = ag(p)

    if x == 0:
        return JSClass.ZERO
    elif v_c == v_1 and v_1 == v_0:
        return JSClass.CONSTANT

    elif v_0 == 0 and v_1 == 0:
        if t_c == 0:
            return JSClass.TRIANGLE
        else:
            return JSClass.TRAPZEZOID

    elif v_0 != 0 and v_1 == 0 and v_c == 0:
        return JSClass.DECEL
    elif v_0 == v_c and v_1 > v_0:
        return JSClass.RAMP  # Probably should have a new one, it's reversed from other ram
    elif v_0 == v_c and v_1 < v_0:
        return JSClass.CLIFF
    elif v_1 == v_c and v_0 > v_1:
        return JSClass.RAMP
    elif v_1 == v_c and v_0 < v_1:
        return JSClass.ACEL

    elif (v_0 != 0 or v_1 != 0) and v_0 < v_max and v_1 < v_max:
        if t_c == 0:
            return JSClass.PENTAGON
        elif v_c > max(v_0, v_1):
            return JSClass.HEXAGON
        elif v_c < min(v_0, v_1):
            return JSClass.TROUGH
        else:
            return JSClass.UNK
    elif v_c < v_0 and v_c < v_1:
        return JSClass.TROUGH


    else:
        return JSClass.UNK