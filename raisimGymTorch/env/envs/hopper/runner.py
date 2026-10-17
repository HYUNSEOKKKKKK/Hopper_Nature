import argparse
from pathlib import Path
import sys

if __package__ in (None, ''):
    sys.path.insert(0, str(Path(__file__).resolve().parents[4]))

from raisimGymTorch.helper.hopper_config import REPO_ROOT, TASK_DIR, load_config, positive_int
from raisimGymTorch.helper.hopper_viewer import check_viewer, start_viewer, stop_viewer, viewer_argument

parser = argparse.ArgumentParser(description="Train the Hopper policy with PPO.")
parser.add_argument('--config', type=Path, help='YAML configuration (default: task cfg.yaml)')
parser.add_argument('--log-dir', type=Path, default=REPO_ROOT / 'raisimGymTorch/data/hopper')
parser.add_argument('--iterations', type=positive_int, default=10001, help='number of PPO updates')
parser.add_argument('--num-envs', type=positive_int, help='override parallel environment count')
parser.add_argument('--num-threads', type=positive_int, help='override OpenMP thread count')
parser.add_argument('--seed', type=int, help='override the configuration seed')
viewer_argument(parser)
parser.add_argument('--no-tensorboard', action='store_true', help='do not launch a browser/server')
args = parser.parse_args()
viewer_executable = check_viewer(parser, args.viewer)

from ruamel.yaml import YAML, dump, RoundTripDumper
from raisimGymTorch.env.RaisimGymVecEnv import RaisimGymVecEnv as VecEnv
from raisimGymTorch.helper.raisim_gym_helper import ConfigurationSaver, tensorboard_launcher
from raisimGymTorch.env.bin.hopper import NormalSampler
from raisimGymTorch.env.bin.hopper import RaisimGymEnv
from raisimGymTorch.env.RewardAnalyzer import RewardAnalyzer
import math
import time
import raisimGymTorch.algo.ppo.module as ppo_module
import raisimGymTorch.algo.ppo.ppo as PPO
import torch.nn as nn
import numpy as np
import torch
import datetime


# check if gpu is available
device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')

# directories
task_path = str(TASK_DIR)

# config
cfg = load_config(args.config, num_envs=args.num_envs, num_threads=args.num_threads,
                  headless=args.headless, seed=args.seed)
if args.viewer:
    cfg['environment']['render'] = True

# create environment from the configuration file
env = VecEnv(RaisimGymEnv(str(REPO_ROOT / 'rsc'), dump(cfg['environment'], Dumper=RoundTripDumper)))
env.seed(cfg['seed'])

# shortcuts
ob_dim = env.num_obs
est_dim = env.num_est
act_dim = env.num_acts
num_threads = cfg['environment']['num_threads']

# Training
n_steps = math.floor(cfg['environment']['max_time'] / cfg['environment']['control_dt'])
total_steps = n_steps * env.num_envs

avg_rewards = []

actor = ppo_module.Actor(ppo_module.MLP(cfg['architecture']['policy_net'], nn.LeakyReLU, ob_dim+est_dim, act_dim),
# actor = ppo_module.Actor(ppo_module.CustomMLP(cfg['architecture']['policy_net'], nn.LeakyReLU, ob_dim+est_dim, act_dim),
                         ppo_module.MultivariateGaussianDiagonalCovariance(act_dim,
                                                                           env.num_envs,
                                                                           1.1,
                                                                           NormalSampler(act_dim),
                                                                           cfg['seed']),
                         device)
critic = ppo_module.Critic(ppo_module.MLP(cfg['architecture']['value_net'], nn.LeakyReLU, ob_dim+est_dim, 1),
                           device)

barrier_critic = ppo_module.Critic(ppo_module.MLP(cfg['architecture']['barrier_value_net'], nn.LeakyReLU, ob_dim+est_dim, 1),
                           device)

estimator = ppo_module.Estimator(ppo_module.MLP(cfg['architecture']['estimator_net'], nn.LeakyReLU, ob_dim,est_dim),
                                 device)

saver = ConfigurationSaver(log_dir=str(args.log_dir.expanduser().resolve()),
                           save_items=[task_path + "/Environment.hpp"])
with open(saver.data_dir + '/cfg.yaml', 'w') as stream:
    YAML().dump(cfg, stream)
print('Training output:', saver.data_dir)
print('Device:', device)
if not args.no_tensorboard:
    tensorboard_launcher(saver.data_dir+"/..")

ppo = PPO.PPO(actor=actor,
              critic=critic,
              barrier_critic = barrier_critic,
              estimator=estimator,
              num_envs=cfg['environment']['num_envs'],
              num_transitions_per_env=n_steps,
              num_learning_epochs=4,
              gamma=0.99,
              lam=0.95,
              # learning_rate = 9e-5,
              learning_rate = 3.0e-4,
              num_mini_batches=4,
              entropy_coef=0.01,
              device=device,
              log_dir=saver.data_dir,
              shuffle_batch=False,
              )

reward_analyzer = RewardAnalyzer(env, ppo.writer)
viewer = start_viewer(env, viewer_executable)
# scheduler = torch.optim.lr_scheduler.MultiStepLR(ppo.optimizer, milestones=[2000], gamma=0.5)
scheduler = torch.optim.lr_scheduler.MultiStepLR(ppo.optimizer, milestones=[1000], gamma=0.75)

for update in range(args.iterations):
    start = time.time()
    env.reset()
    reward_sum = 0
    done_sum = 0
    average_dones = 0.

    if update % cfg['environment']['eval_every_n'] == 0:
        print("Visualizing and evaluating the current policy")
        torch.save({
            'actor_architecture_state_dict': actor.architecture.state_dict(),
            'actor_distribution_state_dict': actor.distribution.state_dict(),
            'critic_architecture_state_dict': critic.architecture.state_dict(),
            'barrier_critic_architecture_state_dict': barrier_critic.architecture.state_dict(),
            'estimator_architecture_state_dict': estimator.architecture.state_dict(),  # added
            'optimizer_state_dict': ppo.optimizer.state_dict(),
        }, saver.data_dir+"/full_"+str(update)+'.pt')
        # we create another graph just to demonstrate the save/load method
        loaded_graph = ppo_module.MLP(cfg['architecture']['policy_net'], nn.LeakyReLU, ob_dim + est_dim, act_dim)
        loaded_graph.load_state_dict(torch.load(saver.data_dir+"/full_"+str(update)+'.pt', map_location='cpu', weights_only=True)['actor_architecture_state_dict'])
        loaded_graph_est = ppo_module.MLP(cfg['architecture']['estimator_net'], nn.LeakyReLU, ob_dim, est_dim)
        loaded_graph_est.load_state_dict(torch.load(saver.data_dir+"/full_"+str(update)+'.pt', map_location='cpu', weights_only=True)['estimator_architecture_state_dict'])

        env.turn_on_visualization()
        if cfg.get('record_video', False) and cfg['environment']['render']:
            env.start_video_recording(datetime.datetime.now().strftime("%Y-%m-%d-%H-%M-%S") + "policy_"+str(update)+'.mp4')

        for step in range(n_steps):
            with torch.no_grad():
                frame_start = time.time()
                obs = env.observe(False)
                est_out = loaded_graph_est.architecture(torch.from_numpy(obs).cpu())
                action = loaded_graph.architecture(torch.from_numpy(np.hstack((obs,est_out))).cpu())
                reward, dones, barrier_reward = env.step(action.cpu().detach().numpy())
                frame_end = time.time()
                wait_time = cfg['environment']['control_dt'] - (frame_end-frame_start)
                if cfg['environment']['render'] and wait_time > 0.:
                    time.sleep(wait_time)

        if cfg.get('record_video', False) and cfg['environment']['render']:
            env.stop_video_recording()
        env.turn_off_visualization()

        env.reset()
        env.save_scaling(saver.data_dir, str(update))  # WITH Obs Normalization

    # actual training
    for step in range(n_steps):
        # obs = env.observe(False)  # WITHOUT Obs Normalization
        obs = env.observe()     # WITH Obs Normalization (better) -> cause, value obs also use this one
        est_out = estimator.predict(torch.from_numpy(obs).to(device)).cpu().numpy()
        true_state = env.get_state() # obs (currently, not used) + true state
        action = ppo.act(np.hstack((obs,est_out)))
        reward, dones, barrier_reward = env.step(action)
        # ppo.step(value_obs=np.hstack((obs,value_obs[:,-est_dim:])), est_obs = obs,true_state=value_obs[:,-est_dim:], rews=reward, dones=dones, bar_rews=barrier_reward)
        ppo.step(value_obs=np.hstack((obs,true_state)), est_obs = obs,true_state=true_state, rews=reward, dones=dones, bar_rews=barrier_reward)
        done_sum = done_sum + np.sum(dones)
        reward_sum = reward_sum + np.sum(reward)
        if (update % 100 == 0): # 평지, stair
            reward_analyzer.add_reward_info(env.get_reward_info())

    # take st step to get value obs
    # obs = env.observe(False)  # WITHOUT Obs Normalization
    obs = env.observe()     # WITH Obs Normalization
    est_out = estimator.predict(torch.from_numpy(obs).to(device)).cpu().numpy()
    true_state = env.get_state()
    # ppo.update(actor_obs=np.hstack((obs,est_out)), value_obs=np.hstack((obs,value_obs[:,-est_dim:])),log_this_iteration=update % 10 == 0, update=update)
    ppo.update(actor_obs=np.hstack((obs,est_out)), value_obs=np.hstack((obs,true_state)),log_this_iteration=update % 10 == 0, update=update)
    average_ll_performance = reward_sum / total_steps
    average_dones = done_sum / total_steps
    avg_rewards.append(average_ll_performance)

    actor.update()

    min_std_factor = max(0.5, 0.5 + 0.6 * (3000 - update) / 3000)
    min_std = torch.ones(act_dim) *min_std_factor
    actor.distribution.enforce_minimum_std((min_std).to(device))
    actor.distribution.enforce_maximum_std((torch.ones(act_dim)*1.5).to(device))

    # curriculum update. Implement it in Environment.hpp
    env.curriculum_callback()

    end = time.time()
    scheduler.step()

    if (update % 100 == 0): # 평지, stair
        reward_analyzer.analyze_and_plot(update)

    print('----------------------------------------------------')
    print('{:>6}th iteration'.format(update))
    print('{:<40} {:>6}'.format("average ll reward: ", '{:0.10f}'.format(average_ll_performance)))
    print('{:<40} {:>6}'.format("dones: ", '{:0.6f}'.format(average_dones)))
    print('{:<40} {:>6}'.format("time elapsed in this iteration: ", '{:6.4f}'.format(end - start)))
    print('{:<40} {:>6}'.format("fps: ", '{:6.0f}'.format(total_steps / (end - start))))
    print('{:<40} {:>6}'.format("real time factor: ", '{:6.0f}'.format(total_steps / (end - start)
                                                                       * cfg['environment']['control_dt'])))
    print(actor.distribution.std)
    print('----------------------------------------------------\n')

env.close()
stop_viewer(viewer)
