from grutopia.core.config import SimulatorConfig
from grutopia.core.env import BaseEnv
from grutopia.core.util.container import is_in_container

file_path = '/home/percy/princeton/GRUtopia/demo/configs/h1_locomotion.yaml'
sim_config = SimulatorConfig(file_path)
print("sim_config: ", sim_config.config.tasks[0].robots[0].position)
headless = False
webrtc = False

if is_in_container():
    headless = True
    webrtc = True

env = BaseEnv(sim_config, headless=headless, webrtc=webrtc)

import numpy as np
from omni.isaac.core.utils.rotations import euler_angles_to_quat, quat_to_euler_angles

from grutopia.core.util import log

task_name = env.config.tasks[0].name
robot_name = env.config.tasks[0].robots[0].name

path = [(1.0, 0.0, 0.0)] # , (1.0, 1.0, 0.0), (3.0, 4.0, 0.0)
i = 0

move_action = {'move_along_path': [path]}
# move_action = {'move_to_point': [(5.5, -0.77, 1.05)]}
# move_action = {'move_by_speed': (0.01, 0.1, 0.5)}
rotate_action = {'rotate': [euler_angles_to_quat(np.array([0, 0, 1]))]}
path_finished = False
actions = {'h1': move_action}

while env.simulation_app.is_running():
    i += 1
    env_actions = []
    env_actions.append(actions)
    obs = env.step(actions=env_actions)
    robot_position = tuple(float(x) for x in obs[task_name][robot_name]['position'])
    robot_orientation = obs[task_name][robot_name]['orientation']
    rotate_finished = obs[task_name][robot_name]['rotate'].get('finished', False)
    # print(rotate_finished)
    if not path_finished:
        path_finished = obs[task_name][robot_name]['move_along_path'].get('finished', False)
        if path_finished:
            log.info('start rotate')
            # rotate_action = {'rotate': [robot_orientation]}
            actions['h1'] = rotate_action
            start_rotate = True

    if i % 100 == 0:
        print(i)
        print('available observations for h1: {}'.format(obs[task_name][robot_name].keys()))
        print('current position of h1:{}'.format(obs[task_name][robot_name]['position']))
        print('current orientation of h1: {}'.format(quat_to_euler_angles(obs[task_name][robot_name]['orientation'])))

env.simulation_app.close()
