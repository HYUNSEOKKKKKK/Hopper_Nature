from ruamel.yaml import YAML, dump, RoundTripDumper
from raisimGymTorch.env.bin import hound
from raisimGymTorch.env.RaisimGymVecEnv import RaisimGymVecEnv as VecEnv
import raisimGymTorch.algo.ppo.module as ppo_module
import os
import math
import time
import torch
import argparse
import numpy as np
import pygame

joystick = 1
if (joystick == 1) :
    pygame.display.init()
    pygame.joystick.init()
    pygame.joystick.Joystick(0).init()

# configuration
parser = argparse.ArgumentParser()
parser.add_argument('-w', '--weight', help='trained weight path', type=str, default='')
args = parser.parse_args()

# directories
task_path = os.path.dirname(os.path.realpath(__file__))
home_path = task_path + "/../../../../.."

# config
cfg = YAML().load(open(task_path + "/cfg.yaml", 'r'))

# create environment from the configuration file
cfg['environment']['num_envs'] = 1

env = VecEnv(hound.RaisimGymEnv(home_path + "/rsc", dump(cfg['environment'], Dumper=RoundTripDumper)), cfg['environment'])

# shortcuts
ob_dim = env.num_obs
act_dim = env.num_acts
est_dim = env.num_est

# weight_path = "/home/gijeong/workspace/raisimLib/hound/raisimGymTorch/data/dhal_one_leg/2.1th_closed_loop_highPD_2025-02-20-00-42-22/full_2000.pt"
weight_path = "/media/gijeong/T7/raisimGymTorch/data/dhal_one_leg/2025-02-21-13-19-02/full_500.pt"
iteration_number = weight_path.rsplit('/', 1)[1].split('_', 1)[1].rsplit('.', 1)[0]
weight_dir = weight_path.rsplit('/', 1)[0] + '/'


if weight_path == "":
    print("Can't find trained weight, please provide a trained weight with --weight switch\n")
else:
    print("Loaded weight from {}\n".format(weight_path))
    start = time.time()

    env.set_terrain(0,0.0,1.0)
    env.set_initial(0)

    env.reset()
    # env.set_terrain(4,4.7,1.0)
    env.set_command(1.0,0.0,0.0)

    reward_ll_sum = 0
    done_sum = 0
    average_dones = 0.
    n_steps = math.floor(cfg['environment']['max_time'] / cfg['environment']['control_dt'])
    total_steps = n_steps * 1
    start_step_id = 0

    print("Visualizing and evaluating the policy: ", weight_path)
    loaded_graph = ppo_module.MLP(cfg['architecture']['policy_net'], torch.nn.LeakyReLU, ob_dim+est_dim, act_dim)
    loaded_graph.load_state_dict(torch.load(weight_path)['actor_architecture_state_dict'])
    loaded_graph_est = ppo_module.MLP(cfg['architecture']['estimator_net'], torch.nn.LeakyReLU, ob_dim, est_dim)
    loaded_graph_est.load_state_dict(torch.load(weight_path)['estimator_architecture_state_dict'])

    env.load_scaling(weight_dir, int(iteration_number))
    env.turn_on_visualization()

    # max_steps = 1000000
    max_steps = 10000 ## 10 secs

    for step in range(max_steps):
        if joystick == 1 :
            if (step % 10 == 0):
                command_x = max(-0.8, min(-pygame.joystick.Joystick(0).get_axis(4)*1.5, 1.2))
                command_y = max(-0.6, min(-pygame.joystick.Joystick(0).get_axis(3), 0.6))
                command_yaw = max(-0.6, min(-pygame.joystick.Joystick(0).get_axis(1), 0.6))
                env.set_command(command_x,command_y,command_yaw)
            else:
                if step % 200 == 0:
                    env.set_command(np.random.uniform(-1.0, 1.0, 1),
                                    np.random.uniform(-0.6, 0.6, 1),
                                    np.random.uniform(-0.6, 0.6, 1))

        time.sleep(0.012)
        with torch.no_grad():
            obs = env.observe(False)
            est_out = loaded_graph_est.architecture(torch.from_numpy(obs).cpu())
            value_obs = env.value_observe(False) # obs + true state
            # print("===========")
            # print(est_out)
            # print(value_obs[:,-est_dim:])
            action_ll = loaded_graph.architecture(torch.from_numpy(np.hstack((obs,est_out))).cpu())
        print(action_ll)
        reward_ll, dones, _= env.step(action_ll.cpu().detach().numpy())
        reward_ll_sum = reward_ll_sum + reward_ll[0]
        if dones or step == max_steps - 1:
            print('----------------------------------------------------')
            print('{:<40} {:>6}'.format("average ll reward: ", '{:0.10f}'.format(reward_ll_sum / (step + 1 - start_step_id))))
            print('{:<40} {:>6}'.format("time elapsed [sec]: ", '{:6.4f}'.format((step + 1 - start_step_id) * 0.01)))
            print('----------------------------------------------------\n')
            start_step_id = step + 1
            reward_ll_sum = 0.0

    env.turn_off_visualization()
    env.reset()
    print("Finished at the maximum visualization steps")
