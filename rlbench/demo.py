import numpy as np


class Demo(object):

    def __init__(self, observations, random_seed=None, num_reset_attempts = None):
        self._observations = observations
        self.random_seed = random_seed
        self.num_reset_attempts = num_reset_attempts

    def __len__(self):
        return len(self._observations)

    def __getitem__(self, i):
        return self._observations[i]

    def restore_state(self):
        np.random.set_state(self.random_seed)
