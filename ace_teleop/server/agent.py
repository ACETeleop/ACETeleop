from typing import Any, Dict, Protocol

import numpy as np


class Agent(Protocol):
    
    def get_ee(self) -> np.ndarray:
        raise NotImplementedError

    def get_joint(self) -> np.ndarray:
        raise NotImplementedError
