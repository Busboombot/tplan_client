from dataclasses import dataclass
from math import ceil
from typing import Callable


def num_blocks_of_size(x, block_size):
    pass


def split_by_size(x, group_size):
    """Return a list of group sizes of approximately the same, but less than group_size"""

    n = ceil(x / group_size)
    g = round(x / n)
    return [g] * (n - 1) + [x - (g * (n - 1))]


def split_by_number(x, n):
    g = round(x / n)
    l = [g] * (n - 1) + [x - (g * (n - 1))]
    # The last element ends up smaller than the rest. Move it to the middle for
    # symetry
    m = len(l) // 2
    l = l[m:] + l[:m]
    return l


def split_moves(moves, joints):
    lng = max(moves)
    small_x = min([(j.v_max ** 2) / (j.a_max) for j in joints])
    group_size = min(small_x, lng / 2)

    l = split_by_size(lng, group_size)

    return [split_by_number(move, len(l)) for move in moves]


def acceleration(j, l):
    return [((c / j.dt) - (p / j.dt)) / j.dt for p, c in pairs(l)]


def velocity(j, l):
    return [e / j.dt for e in l]


def x_to_v(x, dt):
    return x / dt


def v_to_x(v, dt):
    return v * dt


def with_limits(l, v_0, v_1, dt):
    return [round(v_to_x(v_0, dt))] + l + [round(v_to_x(v_1, dt))]


def pairs(l):
    return list(zip(l, l[1:]))


def triplets(l):
    return list(zip(l, l[1:], l[2:]))


def diffs(l):
    return [a - b for a, b in pairs(l)]


def excess(l):
    return [e for e in diffs(l)]


def mean(l):
    """Return the value in this cell that connects a line between the
    values in adjacent cells. Requires with-limits"""
    return [(p + n) / 2 for p, c, n in triplets(l)]


def minv(j, lwl):
    return [round(max((p + n) / 2 - j.dx_max, 0)) for p, c, n in triplets(lwl)]


def maxv(j, lwl):
    # return [round(min((p + n) / 2 + j.dx_max, v_to_x(j.v_max, j.dt))) for p, c, n in triplets(lwl)]
    return [round(min((p + n) / 2 + j.dx_max, j.x_max)) for p, c, n in triplets(lwl)]


def bv_err(lwl):
    """Boundary value error"""
    return [lwl[1]-lwl[0]] + ([0]*(len(lwl)-4)) + [lwl[-2]-lwl[-1]]

def mean_err(lwl):
    """Difference between each cell. Requires with-limits, returns without limits."""
    return [c - m for c, m in zip(lwl[1:-1], mean(lwl))]


def valuere(l):
    """Return value in a reverse enumerated list"""
    return [e[0] for e in l]


def indexre(l):
    """Return index in a reverse enumerated list"""
    return [e[1] for e in l]


def normre(l):
    """Normalize a reverse-enumerated list"""
    s = sum(e[0] for e in l)
    return [((v / s) if s != 0 else 0, i) for v, i in l]


def rms_err(l):
    from math import sqrt
    return sqrt(sum(v ** 2 for v in mean_err(l)))


def rms_pos_err(l):
    from math import sqrt
    return sqrt(sum(v ** 2 for v in mean_err(l) if v >= 0))


def rms_neg_err(l):
    from math import sqrt
    return sqrt(sum(v ** 2 for v in mean_err(l) if v <= 0))


def renumerate(l):
    """Enumerate(), but with the index second"""
    return [(v, i) for i, v in enumerate(l)]


def can_recieve(j, lwl, norm=False):
    """How many steps each bin can recieve without going out of spec.
    Takes a list with limits and returns without limits"""

    def cr(x, m):
        cr1 = m - x  # x limit due to accelleration
        cr2 = j.x_max - x  # limit due to v_max (expressed in x as x_max)
        return max(min(cr1, cr2), 0)

    l_ = [cr(x, m) for x, m in zip(lwl[1:-1], maxv(j, lwl))]
    if norm:
        s = sum(l_)
        l_ = [v / s for v in l_]
    return l_


def can_donate(j, lwl, norm=False):
    """How many steps can be given to the bin without going out of spec.
    Takes a list with limits and returns without limits"""
    l_ = [v - m for v, m in zip(lwl[1:-1], minv(j, lwl))]
    if norm:
        s = sum(l_)
        l_ = [v / s for v in l_]
    return l_


def apply_dx(l, dx):
    return [v + dx_ for v, dx_ in zip(l, dx)]


def value_mult(l, v):
    """Multiply each value in an re-format list"""
    return [(e[0] * v, e[1]) for e in l]


def longest_index(l):
    """Return the index of the longest item in the list"""
    return next(reversed(sorted(renumerate(l))))[1]


def sl(lwl):
    """Strip limits from a list with limits"""
    return lwl[1:-1]


@dataclass
class SplitInfo:
    v_max: float
    a_max: float
    dt: float
    x_max: float
    dx_max: float
    n: int
    t: float
    make_add_limits: Callable


def dataframe(jd, l, v_0, v_1, err_f=None):
    import pandas as pd
    import numpy as np
    al = jd.make_add_limits(v_0, v_1)

    d = {
        't': np.linspace(0, jd.t, num=len(l)),
        'v': velocity(jd, l),
        'max_v': velocity(jd, maxv(jd, al(l))),
    }

    if err_f is not None:
        d['err'] = velocity(jd, err_f(al(l)))

    df = pd.DataFrame(d).set_index('t')

    return df


def plot_isolve(jd, l, v_0, v_1, err_f=None, ax=None):
    return dataframe(jd, l, v_0, v_1, err_f).plot(ax=ax, marker='+')


def ipdataframe(jd, lwl):
    import pandas as pd
    rows = []
    for p, c, n in triplets(lwl):
        pv = p / jd.dt
        cv = c / jd.dt
        nv = n / jd.dt
        v_0 = (pv + cv) / 2
        v_1 = (cv + nv) / 2

        rows.append({'t': None, 'seg': 0, 'axis': 0,
                     'x': c, 'v_i': v_0, 'v_f': v_1, 'del_t': jd.dt})

    df = pd.DataFrame(rows)
    df['area'] = ((df.v_i + df.v_f) / 2 * df.del_t).round().astype(int)
    df['v_i'] = df.v_i.round().astype(int)
    df['v_f'] = df.v_f.round().astype(int)
    df['x'] = df.x.round().astype(int)

    return df


def update_ne(j, lwl, dxf=.3, err_f=None, parts=False):
    """
    Distribute area to remove negative error
    :param lwl: a list of area, with limits
    :type lwl:
    :param dxf:
    :type dxf:
    :return: a list of areas, with no limits.
    :rtype:
    """

    l = lwl[1:-1]

    if err_f is None:
        err_f = mean_err

    ne = [e for e in renumerate(err_f(lwl)) if e[0] < 0]  # Negative error
    cd = [(v, i) for v, i in renumerate(can_donate(j, lwl)) if i not in indexre(ne)]
    dx = min(abs(sum(valuere(ne))), sum(valuere(cd))) * dxf

    ne_dx = value_mult(normre(ne), dx)
    cd_dx = value_mult(normre(cd), -dx)

    if parts:
        return [(v, dx) for v, (dx, i) in zip(l, sorted(ne_dx + cd_dx, key=lambda e: e[1]))]
    else:
        l = l[:]

        for (dx, i) in sorted(ne_dx + cd_dx, key=lambda e: e[1]):
            l[i] += dx

        return l


def update_pe(j, lwl, dxf=.3, err_f=None, parts=False):
    """
    Distribute area to remove positive error
    :param lwl: a list of area, with limits
    :type lwl:
    :param dxf:
    :type dxf:
    :return: a list of areas, with no limits.
    :rtype:
    """

    l = lwl[1:-1]

    if err_f is None:
        err_f = mean_err

    pe = [e for e in renumerate(err_f(lwl)) if e[0] > 0]  # positive error
    cr = [(v, i) for v, i in renumerate(can_recieve(j, lwl)) if i not in indexre(pe) and v > 0]
    dx = min(abs(sum(valuere(pe))), sum(valuere(cr))) * dxf

    pe_dx = value_mult(normre(pe), -dx)
    ct_dx = value_mult(normre(cr), dx)

    if parts:
        return [(v, dx) for v, (dx, i) in zip(l, sorted(pe_dx + ct_dx, key=lambda e: e[1]))]
    else:
        l = l[:]

        for (dx, i) in sorted(pe_dx + ct_dx, key=lambda e: e[1]):
            l[i] += dx

        return l

def err_f(j, lwl):
    me = mean_err(lwl)

    xe = [(x - j.x_max) if x > j.x_max else 0 for x in sl(lwl)]

    bv = bv_err(lwl)

    return [max(me_, xe_, bv_) for me_, xe_, bv_ in zip(me, xe, bv)]


def dist_error(jd, lwl, d, err_f=err_f):
    import numpy as np
    l = sl(lwl)
    ce = [e for e in renumerate(err_f(jd, lwl)) if d * e[0] > 0]  # Error calculated from error function
    oe = [e for e in renumerate(err_f(jd, lwl)) if d * e[0] <= 0]  # The opposite error, or no error
    idx = indexre(ce)  # indexes of cells with error

    # One of the error buckets was empty, so splt above and below the mean
    if len(oe) == 0 or len(ce) == 0:
        x = np.array(list(renumerate(err_f(jd, lwl))))
        ce = x[x[:, 0] > x[:, 0].mean()].tolist()
        oe = x[x[:, 0] < x[:, 0].mean()].tolist()

    if d < 0:
        ce, oe = oe, ce

    for v, i in ce:
        x = v * .3  # Only take a portion each iteration
        l[int(i)] -= x  # Remove all error from the cell with error
        dx = x / len(oe)  # divide error up among cells with opposite error
        for v, i in oe:
            l[int(i)] += dx

    return l

def ipsplit(x, t=None, n=None, j=None):
    dx_max = ceil((j.v_max ** 2) / (j.a_max) / 1 )

    if t and not n:
        n = ceil(j.v_max * t / dx_max)
        dt = t / n
    elif t and n:
        dt = t / n
        dx_max = ceil(j.v_max * dt)
    elif not t and not n:
        dt = dx_max / j.v_max
        n = ceil((x + dx_max) / dx_max)
        t = n * dt
    elif n and not t:
        dt = dx_max / j.v_max
        t = dt * n

    assert x <= dx_max * n

    def make_add_limits(v_0, v_1):
        def _f(l):
            return with_limits(l,
                               v_0 if v_0 else 0,
                               v_1 if v_1 else 0,
                               dt)

        return _f

    return SplitInfo(
        v_max=j.v_max,
        a_max=j.a_max,
        dt=dt,
        x_max=j.v_max * dt,
        dx_max=dx_max,
        n=n,
        t=dt * n,
        make_add_limits=make_add_limits)


def ipsolve(jd, x, v_0=None, v_1=None, err_f=err_f):
    # from IPython.display import display
    # import matplotlib.pylab as plt
    import numpy as np

    l = split_by_number(x, jd.n)

    al = jd.make_add_limits(v_0, v_1)

    l = split_by_number(x, jd.n)

    last_rmse = 0
    for i in range(30):
        l = dist_error(jd, al(l), 1)
        l = dist_error(jd, al(l), -1)
        rmse = np.sqrt(np.sum(np.square(err_f(jd, al(l)))))
        if rmse == 0 or abs((last_rmse - rmse) / rmse) < .001:
            break

        last_rmse = rmse

    return l



#
# Extra functions
#


def pos_err(l):
    return [e for e in renumerate(mean_err(l)) if e[0] > 0]


def neg_err(l):
    return [e for e in renumerate(mean_err(l)) if e[0] < 0]
