import numpy as np
from trajectory_synthesis_exceptions import InvalidInput

def get_mask(shape, side, dof_idx, n):
    mask = np.zeros(n)
    if shape in ["sine", "spline", "trapezoid", "sine+trapezoid"]:
        mask[dof_idx] = 1.0
    elif shape in ["positive_right_trapezoid", "positive_triangle"]:
        if n != 12:
            raise InvalidInput(f"{shape} currently only supports 12 dofs not {n}")
        if side == "right":
            mask[8] = 1.0
            mask[9] = 2.0
            mask[10] = -1.0
        elif side == "left":
            mask[2] = -1.0
            mask[3] = -2.0
            mask[4] = 1.0
        else:
            raise InvalidInput(f"side should be left or right not {side}")
    else:
        raise InvalidInput(f"unsupported wave shape: {shape}")

    return mask