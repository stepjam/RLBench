from enum import Enum
from typing import List
import numpy as np
import os
import glob


class RandomizeEvery(Enum):
    EPISODE = 0
    VARIATION = 1
    TRANSITION = 1


class Distributions(object):

    def apply(self, val: np.ndarray) -> np.ndarray:
        pass


class Gaussian(Distributions):

    def __init__(self, variance):
        self._variance = variance

    def apply(self, val: np.ndarray):
        return np.random.normal(val, self._variance)


class Uniform(Distributions):

    def __init__(self, min, max):
        self._min = min
        self._max = max

    def apply(self, val: np.ndarray):
        return np.random.uniform(self._min, self._max, val.shape)


EXTENSIONS = ['*.jpg', '*.png']


class RandomizationConfig(object):

    def __init__(self,
                 whitelist: List[str]=None,
                 blacklist: List[str]=None):
        self.whitelist = whitelist
        self.blacklist = [] if blacklist is None else blacklist

    def should_randomize(self, obj_name: str):
        return ((self.whitelist is None and len(self.blacklist) == 0) or
                (self.whitelist is not None and obj_name in self.whitelist) or
                (obj_name not in self.blacklist))


class DynamicsRandomizationConfig(RandomizationConfig):
    pass


class VisualRandomizationConfig(RandomizationConfig):

    def __init__(self,
                 image_directory: str,
                 whitelist: List[str]=None,
                 blacklist: List[str]=None):
        super().__init__(whitelist, blacklist)
        self._image_directory = image_directory
        if not os.path.exists(image_directory):
            raise NotADirectoryError(
                'The supplied image directory (%s) does not exist!' %
                image_directory)
        self._imgs = np.array([glob.glob(
            os.path.join(image_directory, e)) for e in EXTENSIONS])
        self._imgs = np.concatenate(self._imgs)
        if len(self._imgs) == 0:
            raise RuntimeError(
                'The supplied image directory (%s) does not have any images!' %
                image_directory)

    def sample(self, samples: int) -> np.ndarray:
        return np.random.choice(self._imgs, samples)
