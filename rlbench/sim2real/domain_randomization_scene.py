from typing import List
from pyrep import PyRep
from pyrep.const import ObjectType, TextureMappingMode
from pyrep.objects.shape import Shape

from rlbench.backend.scene import Scene
from rlbench.observation_config import ObservationConfig
from rlbench.backend.robot import Robot
from rlbench.sim2real.domain_randomization import RandomizeEvery

SCENE_OBJECTS = ['Floor', 'Roof', 'Wall1', 'Wall2', 'Wall3', 'Wall4',
                 'diningTable_visible']

TEX_KWARGS = {
    'mapping_mode': TextureMappingMode.PLANE,
    'repeat_along_u': True,
    'repeat_along_v': True,
    'uv_scaling': [4., 4.]
}


class DomainRandomizationScene(Scene):

    def __init__(self,
                 pyrep: PyRep,
                 robot: Robot,
                 obs_config: ObservationConfig = ObservationConfig(),
                 robot_setup: str = 'Panda',
                 randomize_every: RandomizeEvery=RandomizeEvery.EPISODE,
                 frequency: int=1,
                 visual_randomization_config=None,
                 dynamics_randomization_config=None):
        super().__init__(pyrep, robot, obs_config, robot_setup)
        self._randomize_every = randomize_every
        self._frequency = frequency
        self._visual_rand_config = visual_randomization_config
        self._dynamics_rand_config = dynamics_randomization_config
        self._previous_index = -1
        self._count = 0

        if self._dynamics_rand_config is not None:
            raise NotImplementedError(
                'Dynamics randomization coming soon! '
                'Only visual randomization available.')

        self._scene_objects = [Shape(name) for name in SCENE_OBJECTS]
        self._scene_objects += self.robot.arm.get_visuals()
        self._scene_objects += self.robot.gripper.get_visuals()
        if self._visual_rand_config is not None:
            # Make the floor plane renderable (to cover old floor)
            self._scene_objects[0].set_position([0, 0, 0.01])
            self._scene_objects[0].set_renderable(True)

    def _should_randomize_episode(self, index: int):
        rand = self._count % self._frequency == 0 or self._count == 0
        if self._randomize_every == RandomizeEvery.VARIATION:
            if self._previous_index != index:
                self._previous_index = index
                self._count += 1
        elif self._randomize_every == RandomizeEvery.EPISODE:
            self._count += 1
        return rand

    def _randomize(self):
        tree = self.task.get_base().get_objects_in_tree(
            ObjectType.SHAPE)
        tree = [Shape(obj.get_handle()) for obj in tree + self._scene_objects]
        if self._visual_rand_config is not None:
            files = self._visual_rand_config.sample(len(tree))
            for file, obj in zip(files, tree):
                if self._visual_rand_config.should_randomize(obj.get_name()):
                    text_ob, texture = self.pyrep.create_texture(file)
                    try:
                        obj.set_texture(texture, **TEX_KWARGS)
                    except RuntimeError:
                        ungrouped = obj.ungroup()
                        for o in ungrouped:
                            o.set_texture(texture, **TEX_KWARGS)
                        self.pyrep.group_objects(ungrouped)
                    text_ob.remove()

    def init_task(self) -> None:
        super().init_task()

    def init_episode(self, index: int, *args, **kwargs) -> List[str]:
        ret = super().init_episode(index, *args, **kwargs)
        if (self._randomize_every != RandomizeEvery.TRANSITION and
                self._should_randomize_episode(index)):
            self._randomize()
            self.pyrep.step()  # Need to step to apply textures
        return ret

    def step(self):
        if self._randomize_every == RandomizeEvery.TRANSITION:
            if self._count % self._frequency == 0 or self._count == 0 :
                self._randomize()
            self._count += 1
        super().step()

    def reset(self) -> None:
        return super().reset()
