from rlbench.environment import Environment
from rlbench.action_modes import ArmActionMode, ActionMode
from rlbench.observation_config import ObservationConfig
from rlbench.tasks import PickAndLift, PlayJenga, StackBlocks
import numpy as np
import scipy.misc
import os

# Path to demonstation storage directory
destination_dir = os.path.abspath(__file__ + "/../../../Demonstrations/")

# Path to demonstation storage directory
destination_dir = os.path.abspath(__file__ + "/../../../Demonstrations/")

def save_data(data, name, demo_id, demo_dir_path):
    data_tensor = np.concatenate(data, axis=0)

    tensor_file = name + '_%04d.npy' % (demo_id)
    save_path = os.path.join(demo_dir_path, tensor_file)
    np.save(save_path, data_tensor)

# To use 'saved' demos, set the path below, and set live_demos=False
live_demos = True
DATASET = '' if live_demos else 'PATH/TO/YOUR/DATASET'

obs_config = ObservationConfig()
obs_config.set_all(False)
obs_config.left_shoulder_camera.rgb = True
obs_config.right_shoulder_camera.rgb = True

# run headless
headless = True

action_mode = ActionMode(ArmActionMode.ABS_JOINT_VELOCITY)
env = Environment(
    action_mode, DATASET, obs_config, headless)
env.launch()

task = env.get_task(PickAndLift)

demos = task.get_demos(2, live_demos=live_demos)  #robotrobot -> List[List[Observation]]

for demo_id, demo in enumerate(demos):
    # create directory of demo
    demo_dir_path = os.path.join(destination_dir, 'demo_%04d/' % (demo_id))
    
    try:
        # Create target Directory
        os.mkdir(demo_dir_path)
        print(demo_dir_path)

    except FileExistsError:
        print(demo_dir_path ,  " already exists")

    right_shoulder_imgs = [np.expand_dims(obs.right_shoulder_rgb, axis=0) for obs in demo]
    save_data(right_shoulder_imgs, 'right_shoulder_imgs', demo_id, demo_dir_path)


print('Done')
env.shutdown()