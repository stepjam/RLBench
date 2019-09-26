from typing import Tuple

import numpy as np


class NoiseModel(object):

    def apply(self, val: np.ndarray) -> np.ndarray:
        raise NotImplementedError()


class Identity(NoiseModel):
    def apply(self, val: np.ndarray):
        return val


class GaussianNoise(NoiseModel):

    def __init__(self, variance, min_max_clip: Tuple[float, float]=None):
        self._variance = variance
        self._min_max_clip = min_max_clip

    def apply(self, val: np.ndarray):
        val += np.random.normal(0.0, scale=self._variance, size=val.shape)
        if self._min_max_clip is not None:
            val = np.clip(val, *self._min_max_clip)
        return val

