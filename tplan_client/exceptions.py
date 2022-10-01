class SegmentError(Exception):
    pass


class ConvergenceError(SegmentError):
    pass


class BoundaryError(SegmentError):
    pass


class ConstraintError(SegmentError):
    pass


class ValidationError(SegmentError):
    pass


class ShortSegment(SegmentError):
    """Segment is too short"""
    pass


class TimeRecalc(SegmentError):
    """We need to recalc segment time"""
    pass


class TrapMathError(Exception):
    pass


class LimitError(TrapMathError):
    pass


class ShortTimeError(TrapMathError):

    def __init__(self, t, *args: object) -> None:
        super().__init__(*args)
        self.t = t


class ParameterError(TrapMathError):

    def __init__(self, params, *args: object) -> None:
        super().__init__(*args)
        self.params = params


