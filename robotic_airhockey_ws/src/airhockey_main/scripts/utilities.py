# This file contains various utility functions with generic uses.

class Struct(object):
    """
    Generic class to store any variables, preferred over dictionary because of
    tab-completion capabilities.
    """
    pass


class NoSolutionError(Exception):
    """Error when trying to solve for intersection between puck trajectory
    and perimeter of arm reach."""
    pass


class MissingParamError(Exception):
    """
    Error when default parameter for a function is missing.
    """
    pass


def distance(x1, y1, x2, y2):
    return ((x1 - x2)**2 + (y1 - y2)**2)**0.5


def transform(value, is_pos, y_bound=None):
    """
    Converts between graphics and cartesian coordinate frame. In graphics frame,
    y = 0 at the top, but in cartesian, y = 0 at bottom. Velocities are
    reversed.

    :param value: value to be transformed
    :type: float

    :param y_bound: max value of y, used to transform position y values
    :type: float or int

    :param is_pos: is the value to be converted a position or velocity. If
    position, flip frame by subtracting from max y value. If velocity, simply
    reverse direction.

    """
    if is_pos:
        try:
            return y_bound - value
        except TypeError:
            raise MissingParamError('Missing y_bound for position transform')
    else: return -1 * value

