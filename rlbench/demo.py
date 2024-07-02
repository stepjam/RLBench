import numpy as np
from typing import List
from rlbench.backend.observation import Observation


class Demo(object):

    def __init__(self, observations: List[Observation], random_seed=None, num_reset_attempts=None):
        self._observations = observations
        self.random_seed = random_seed
        self.num_reset_attempts = num_reset_attempts

    def __len__(self):
        return len(self._observations)

    def __getitem__(self, i) -> Observation:
        return self._observations[i]

    def restore_state(self):
        np.random.set_state(self.random_seed)
