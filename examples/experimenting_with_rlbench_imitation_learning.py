from rlbench.environment import Environment
from rlbench.action_modes import ArmActionMode, ActionMode
from rlbench.observation_config import ObservationConfig
from rlbench.tasks import PickAndLift, PlayJenga, StackBlocks
import numpy as np
import scipy.misc
import os
import csv
import json

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
obs_config.set_all(True)

# # set image resolution to 256x256
# ''' WHY DOES THIS WORK??? HOW CAN I MODIFY A CLASS VARIABLE LIKE THIS '''
# obs_config.left_shoulder_camera.image_size = (256,256)
# obs_config.right_shoulder_camera.image_size = (256,256)
# obs_config.wrist_camera.image_size = (256,256)

# run headless
headless = True

action_mode = ActionMode(ArmActionMode.ABS_JOINT_VELOCITY)
env = Environment(
    action_mode, DATASET, obs_config, headless)
env.launch()

task = env.get_task(PickAndLift)

demos = task.get_demos(1, live_demos=live_demos)  #robotrobot -> List[List[Observation]]

print('Done')
env.shutdown()

# save data to files
for demo_id, demo in enumerate(demos):
    # create directory of demo
    demo_dir_path = os.path.join(destination_dir, 'demo_%04d/' % (demo_id))
    
    try:
        # Create target Directory
        os.mkdir(demo_dir_path)
        print(demo_dir_path)

    except FileExistsError:
        print(demo_dir_path ,  " already exists")

    # # iterate over each observation in this demo
    # for obs_id, obs in enumerate(demo):

    #     #### get image from each observation and save it ####
    #     img_left = obs.left_shoulder_rgb
    #     img_right = obs.right_shoulder_rgb

    #     # specify path to save image
    #     img_path_left = os.path.join(demo_dir_path, 'img_left_%04d.npy' % (obs_id)) # need to specify path
    #     img_path_right = os.path.join(demo_dir_path, 'img_right_%04d.npy' % (obs_id)) # need to specify path

    #     # save the image in a file

    #     # to save the images as png images, use scipy.misc
    #     scipy.misc.imsave(img_path_left, img_left)
    #     scipy.misc.imsave(img_path_right, img_right)

    ###############################################################
    # save low dim state dictionary
    # with open('data.p', 'wb') as fp:
    # pickle.dump(data, fp, protocol=pickle.HIGHEST_PROTOCOL)

    ###############################################################
    # save images in one .npy file
    right_shoulder_imgs = [np.expand_dims(obs.right_shoulder_rgb, axis=0) for obs in demo]
    save_data(right_shoulder_imgs, 'right_shoulder_imgs', demo_id, demo_dir_path)

    left_shoulder_imgs = [np.expand_dims(obs.left_shoulder_rgb, axis=0) for obs in demo]
    save_data(left_shoulder_imgs, 'left_shoulder_imgs', demo_id, demo_dir_path)
    
    wrist_imgs = [np.expand_dims(obs.wrist_rgb, axis=0) for obs in demo]
    save_data(wrist_imgs, 'wrist_imgs', demo_id, demo_dir_path)

    ################################################################
    # save depths in one .npy file
    right_shoulder_depths = [np.expand_dims(obs.right_shoulder_depth, axis=0) for obs in demo]
    save_data(right_shoulder_depths, 'right_shoulder_depths', demo_id, demo_dir_path)

    left_shoulder_depths = [np.expand_dims(obs.left_shoulder_depth, axis=0) for obs in demo]
    save_data(left_shoulder_depths, 'left_shoulder_depths', demo_id, demo_dir_path)

    wrist_depths = [np.expand_dims(obs.wrist_depth, axis=0) for obs in demo]
    save_data(wrist_depths, 'wrist_depths', demo_id, demo_dir_path)

    ################################################################
    # save masks in one .npy file, and convert to uint8 type

    right_shoulder_masks = [np.expand_dims(obs.right_shoulder_mask.astype(np.uint8), axis=0) for obs in demo]
    save_data(right_shoulder_masks, 'right_shoulder_masks', demo_id, demo_dir_path)

    left_shoulder_masks = [np.expand_dims(obs.left_shoulder_mask.astype(np.uint8), axis=0) for obs in demo]
    save_data(left_shoulder_masks, 'left_shoulder_masks', demo_id, demo_dir_path)

    wrist_masks = [np.expand_dims(obs.wrist_mask.astype(np.uint8), axis=0) for obs in demo]
    save_data(wrist_masks, 'wrist_masks', demo_id, demo_dir_path)

    ###############################################################
    # save lowdim data such as velocities

    # concat all states into tensor
    # joint_velocities
    jnt_vel_list = [np.expand_dims(obs.joint_velocities, axis=0) for obs in demo]
    save_data(jnt_vel_list, 'jnt_vel_list', demo_id, demo_dir_path)

    # joint_positions
    jnt_pos_list = [np.expand_dims(obs.joint_positions, axis=0) for obs in demo]
    save_data(jnt_pos_list, 'jnt_pos_list', demo_id, demo_dir_path)
    
    # gripper_poses
    gripper_poses = [np.expand_dims(obs.gripper_pose, axis=0) for obs in demo]
    save_data(gripper_poses, 'gripper_pose_list', demo_id, demo_dir_path)

    # gripper_joint_positions
    gripper_joint_positions = [np.expand_dims(obs.gripper_joint_positions, axis=0) for obs in demo]
    save_data(gripper_joint_positions, 'gripper_joint_position_list', demo_id, demo_dir_path)
    
    # task_low_dim_state

    save_path = os.path.join(demo_dir_path, 'low_dim_state.csv')
    # if previous version of csv file exists, remove it
    if os.path.exists(save_path):
        os.remove(save_path)
    f = csv.writer(open(save_path, "w"))

    # Write CSV Header, If you dont need that, remove this line
    f.writerow(["waypoint1", "waypoint2", "success_visual", "waypoint0", "stack_blocks_distractor1", "pick_and_lift_target", "pick_and_lift_boundary", "stack_blocks_distractor0", "distractors", "pick_and_lift_success"])

    
    for obs_id, obs in enumerate(demo):
        if "waypoint0" in obs.task_low_dim_state and "waypoint0" in obs.task_low_dim_state and "pick_and_lift_target" in obs.task_low_dim_state:
            f.writerow([obs.task_low_dim_state["waypoint1"],
                        obs.task_low_dim_state["waypoint2"],
                        obs.task_low_dim_state["success_visual"],
                        obs.task_low_dim_state["waypoint0"],
                        obs.task_low_dim_state["stack_blocks_distractor1"],
                        obs.task_low_dim_state["pick_and_lift_target"],
                        obs.task_low_dim_state["pick_and_lift_boundary"],
                        obs.task_low_dim_state["stack_blocks_distractor0"],
                        obs.task_low_dim_state["distractors"],
                        obs.task_low_dim_state["pick_and_lift_success"],
                        ])
        else:
            f.writerow([
                -1.0,
                obs.task_low_dim_state["waypoint2"],
                obs.task_low_dim_state["success_visual"],
                -1.0,
                obs.task_low_dim_state["stack_blocks_distractor1"],
                -1.0,
                obs.task_low_dim_state["pick_and_lift_boundary"],
                obs.task_low_dim_state["stack_blocks_distractor0"],
                obs.task_low_dim_state["distractors"],
                obs.task_low_dim_state["pick_and_lift_success"],
            ])



# different file for each demonstation
# save depth and masks
# create .npy files for one demo, containing rgb.npy (all the RGB images stacked along a new axis 0),
# masks.npy (all mask images stacked along a new axis 0), depth.npy (all depth images stacked along new axis 0)
# joint_states.npy (all joint states stacked along a new axis 0)
# joint_velocities.npy (all joint velocities stacked along a new axis 0)