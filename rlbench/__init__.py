__version__ = "1.2.0"

import os

from gymnasium import register

import rlbench.backend.task as task
from rlbench.action_modes.action_mode import (
    ActionMode,
    ArmActionMode,
    GripperActionMode,
)
from rlbench.environment import Environment
from rlbench.observation_config import CameraConfig, ObservationConfig
from rlbench.sim2real.domain_randomization import (
    RandomizeEvery,
    VisualRandomizationConfig,
)
from rlbench.utils import name_to_task_class

__all__ = [
    "ActionMode",
    "ArmActionMode",
    "GripperActionMode",
    "CameraConfig",
    "Environment",
    "ObservationConfig",
    "RandomizeEvery",
    "VisualRandomizationConfig",
]

TASKS = [
    t for t in os.listdir(task.TASKS_PATH) if t != "__init__.py" and t.endswith(".py")
]

for task_file in TASKS:
    task_name = task_file.split(".py")[0]
    task_class = name_to_task_class(task_name)
    for obs_mode in ["state", "vision"]:
        register(
            id=f"rlbench/{task_name}-{obs_mode}-v0",
            entry_point="rlbench.gym:RLBenchEnv",
            kwargs={
                "task_class": task_class,
                "observation_mode": obs_mode,
            },
            nondeterministic=True,
        )
