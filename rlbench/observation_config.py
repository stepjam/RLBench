from pyrep.const import RenderMode
from rlbench.noise_model import NoiseModel, Identity


class CameraConfig(object):
    def __init__(self,
                 rgb=True,
                 rgb_noise: NoiseModel=Identity(),
                 depth=True,
                 depth_noise: NoiseModel=Identity(),
                 mask=True,
                 image_size=(128, 128),
                 render_mode=RenderMode.OPENGL3):
        self.rgb = rgb
        self.rgb_noise = rgb_noise
        self.depth = depth
        self.depth_noise = depth_noise
        self.mask = mask
        self.image_size = image_size
        self.render_mode = render_mode

    def set_all(self, value: bool):
        self.rgb = value
        self.depth = value
        self.mask = value


class ObservationConfig(object):
    def __init__(self,
                 left_shoulder_camera: CameraConfig = None,
                 right_shoulder_camera: CameraConfig = None,
                 wrist_camera: CameraConfig = None,
                 joint_velocities=True,
                 joint_velocities_noise: NoiseModel=Identity(),
                 joint_positions=True,
                 joint_positions_noise: NoiseModel=Identity(),
                 joint_forces=True,
                 joint_forces_noise: NoiseModel=Identity(),
                 gripper_pose=True,
                 gripper_touch_forces=False,
                 record_gripper_closing=False,
                 task_low_dim_state=False,
                 ):
        self.left_shoulder_camera = (
            CameraConfig() if left_shoulder_camera is None
            else left_shoulder_camera)
        self.right_shoulder_camera = (
            CameraConfig() if right_shoulder_camera is None
            else right_shoulder_camera)
        self.wrist_camera = (
            CameraConfig() if wrist_camera is None
            else wrist_camera)
        self.joint_velocities = joint_velocities
        self.joint_velocities_noise = joint_velocities_noise
        self.joint_positions = joint_positions
        self.joint_positions_noise = joint_positions_noise
        self.joint_forces = joint_forces
        self.joint_forces_noise = joint_forces_noise
        self.gripper_pose = gripper_pose
        self.gripper_touch_forces = gripper_touch_forces
        self.record_gripper_closing = record_gripper_closing
        self.task_low_dim_state = task_low_dim_state

    def set_all(self, value: bool):
        self.set_all_high_dim(value)
        self.set_all_low_dim(value)

    def set_all_high_dim(self, value: bool):
        self.left_shoulder_camera.set_all(value)
        self.right_shoulder_camera.set_all(value)
        self.wrist_camera.set_all(value)

    def set_all_low_dim(self, value: bool):
        self.joint_velocities = value
        self.joint_positions = value
        self.joint_forces = value
        self.gripper_pose = value
        self.gripper_touch_forces = value
        self.task_low_dim_state = value
