from ruamel.yaml import YAML, dump, RoundTripDumper
from raisimGymTorch.env.RaisimGymVecEnv import RaisimGymVecEnv as VecEnv
from raisimGymTorch.helper.raisim_gym_helper import ConfigurationSaver, load_param, tensorboard_launcher
from raisimGymTorch.env.bin.hound import NormalSampler
from raisimGymTorch.env.bin.hound import RaisimGymEnv
from raisimGymTorch.env.RewardAnalyzer import RewardAnalyzer
import os
import math
import time
import raisimGymTorch.algo.ppo.module as ppo_module
import raisimGymTorch.algo.ppo.ppo as PPO
import torch.nn as nn
import numpy as np
import torch
import datetime
import argparse


# task specification
task_name = "hound"

# configuration
parser = argparse.ArgumentParser()
parser.add_argument('-m', '--mode', help='set mode either train or test', type=str, default='train')
parser.add_argument('-w', '--weight', help='pre-trained weight path', type=str, default='')
args = parser.parse_args()
mode = args.mode
weight_path = args.weight

# check if gpu is available
device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')

# directories
task_path = os.path.dirname(os.path.realpath(__file__))
home_path = task_path + "/../../../../.."

# config
cfg = YAML().load(open(task_path + "/cfg.yaml", 'r'))

# create environment from the configuration file
env = VecEnv(RaisimGymEnv(home_path + "/rsc", dump(cfg['environment'], Dumper=RoundTripDumper)))
env.seed(cfg['seed'])

# shortcuts
ob_dim = env.num_obs
value_ob_dim = env.num_value_obs
est_dim = env.num_est
act_dim = env.num_acts
num_threads = cfg['environment']['num_threads']

# Training
n_steps = math.floor(cfg['environment']['max_time'] / cfg['environment']['control_dt'])
total_steps = n_steps * env.num_envs

avg_rewards = []

actor = ppo_module.Actor(ppo_module.MLP(cfg['architecture']['policy_net'], nn.LeakyReLU, ob_dim+est_dim   , act_dim),
                         ppo_module.MultivariateGaussianDiagonalCovariance(act_dim,
                                                                           env.num_envs,
                                                                           1.5,
                                                                           NormalSampler(act_dim),
                                                                           cfg['seed']),
                         device)
critic = ppo_module.Critic(ppo_module.MLP(cfg['architecture']['value_net'], nn.LeakyReLU, value_ob_dim, 1),
                           device)

barrier_critic = ppo_module.Critic(ppo_module.MLP(cfg['architecture']['barrier_value_net'], nn.LeakyReLU, value_ob_dim, 1),
                           device)

estimator = ppo_module.Estimator(ppo_module.MLP(cfg['architecture']['estimator_net'], nn.LeakyReLU, ob_dim,est_dim),
                                 device)

saver = ConfigurationSaver(log_dir=home_path + "/hound/raisimGymTorch/data/"+task_name,
                           save_items=[task_path + "/cfg.yaml", task_path + "/Environment.hpp"])
tensorboard_launcher(saver.data_dir+"/..")  # press refresh (F5) after the first ppo update

ppo = PPO.PPO(actor=actor,
              critic=critic,
              barrier_critic = barrier_critic,
              estimator=estimator,
              num_envs=cfg['environment']['num_envs'],
              num_transitions_per_env=n_steps,
              num_learning_epochs=4,
              gamma=0.99,
              lam=0.95,
              learning_rate = 2.5e-4,
              num_mini_batches=8,
              entropy_coef=0.01,
              device=device,
              log_dir=saver.data_dir,
              shuffle_batch=False,
              )

reward_analyzer = RewardAnalyzer(env, ppo.writer)
# scheduler = torch.optim.lr_scheduler.MultiStepLR(ppo.optimizer, milestones=[2000], gamma=0.333333)
# scheduler = torch.optim.lr_scheduler.MultiStepLR(ppo.optimizer, milestones=[1800], gamma=0.5)
scheduler = torch.optim.lr_scheduler.MultiStepLR(ppo.optimizer, milestones=[2000], gamma=0.5)


# if mode == 'retrain':
#     load_param(weight_path, env, actor, critic, ppo.optimizer, saver.data_dir)

for update in range(16002):
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
        loaded_graph.load_state_dict(torch.load(saver.data_dir+"/full_"+str(update)+'.pt')['actor_architecture_state_dict'])
        loaded_graph_est = ppo_module.MLP(cfg['architecture']['estimator_net'], nn.LeakyReLU, ob_dim, est_dim)
        loaded_graph_est.load_state_dict(torch.load(saver.data_dir+"/full_"+str(update)+'.pt')['estimator_architecture_state_dict'])

        env.turn_on_visualization()
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
                if wait_time > 0.:
                    time.sleep(wait_time)

        env.stop_video_recording()
        env.turn_off_visualization()

        env.reset()
        env.save_scaling(saver.data_dir, str(update))

    # actual training
    for step in range(n_steps):
        obs = env.observe()                   # obs
        est_out = estimator.predict(torch.from_numpy(obs).to(device)).cpu().numpy()
        value_obs = env.value_observe(False) # obs + true state
        action = ppo.act(np.hstack((obs,est_out)))
        reward, dones, barrier_reward = env.step(action)
        ppo.step(value_obs=value_obs, est_obs = obs,true_state=value_obs[:,-est_dim:], rews=reward, dones=dones, bar_rews=barrier_reward)
        done_sum = done_sum + np.sum(dones)
        reward_sum = reward_sum + np.sum(reward)
        if (update % 200 == 0): # 평지, stair
        # if (update % 200 == 0): # 평지, stair
            reward_analyzer.add_reward_info(env.get_reward_info())

    # take st step to get value obs
    obs = env.observe()
    est_out = estimator.predict(torch.from_numpy(obs).to(device)).cpu().numpy()
    value_obs = env.value_observe(False)
    ppo.update(actor_obs=np.hstack((obs,est_out)), value_obs=value_obs,log_this_iteration=update % 10 == 0, update=update)
    average_ll_performance = reward_sum / total_steps
    average_dones = done_sum / total_steps
    avg_rewards.append(average_ll_performance)

    actor.update()
    if update<3000:
        actor.distribution.enforce_minimum_std((torch.ones(12) * 0.8).to(device))
    elif update<10000:
        actor.distribution.enforce_minimum_std((torch.ones(12) * 0.55).to(device))
    else:
        actor.distribution.enforce_minimum_std((torch.ones(12) * 0.5).to(device))

    actor.distribution.enforce_maximum_std((torch.ones(12)*1.5).to(device))

    # curriculum update. Implement it in Environment.hpp
    env.curriculum_callback()

    end = time.time()
    scheduler.step()

    if (update % 200 == 0): # 평지, stair
    # if (update % 200 == 0): # 평지, stair
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
