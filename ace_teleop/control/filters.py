import numpy as np
from pytransform3d import rotations


class LPFilter:
    def __init__(self, alpha: float) -> None:
        self.alpha = alpha
        self.y = None
        self.is_init = False

    def next(self, x: np.ndarray) -> np.ndarray:
        if not self.is_init:
            self.y = x
            self.is_init = True
            return self.y.copy()
        self.y = self.y + self.alpha * (x - self.y)
        return self.y.copy()

    def reset(self) -> None:
        self.y = None
        self.is_init = False


class LPRotationFilter:
    def __init__(self, alpha) -> None:
        self.alpha = alpha
        self.is_init = False

        self.y = None

    def next(self, x: np.ndarray) -> np.ndarray:
        assert x.shape == (4,)

        # assuming dealing with w, x, y, z quaternions

        if not self.is_init:
            self.y = x
            self.is_init = True
            return self.y.copy()

        self.y = rotations.quaternion_slerp(self.y, x, self.alpha, shortest_path=True)
        return self.y.copy()

    def reset(self) -> None:
        self.y = None
        self.is_init = False
