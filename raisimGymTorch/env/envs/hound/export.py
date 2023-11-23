###             ###
### From RaiLab ###
###             ###
from ruamel.yaml import YAML, dump, RoundTripDumper
from raisimGymTorch.env.bin import hound
from raisimGymTorch.env.RaisimGymVecEnv import RaisimGymVecEnv as VecEnv
import raisimGymTorch.algo.ppo.module as ppo_module
import os
import shutil
import torch
import torch.nn as nn
import argparse


# weight argument handler
parser = argparse.ArgumentParser()
parser.add_argument('-w', '--weight', help='pre-trained weight path', type=str, default='')
args = parser.parse_args()

# directories
task_path = os.path.dirname(os.path.realpath(__file__))
home_path = task_path + "/../../../../.."

# weight path
full_path = "/home/gijeong/workspace/raisimLib/hound/raisimGymTorch/data/hound/2023-11-22-21-13-21/full_7839.pt"
full = torch.load(full_path)
iteration = full_path.rsplit('/')[-1].rsplit('_')[-1].rsplit('.')[0]
scale_path = '/'.join(full_path.rsplit('/')[:-1])
# scale_path = '/'.join(full_path.rsplit('/')[:-1]) + "/scale"
dir_path = '/'.join(full_path.rsplit('/')[:-1]) + "/network_" + iteration

weight_dir = full_path.rsplit('/', 1)[0] + '/'


# config
cfg = YAML().load(open(weight_dir + "/cfg.yaml", 'r'))
cfg['environment']['num_envs'] = 1

# create environment from the configuration file
env = VecEnv(hound.RaisimGymEnv(home_path + "/rsc", dump(cfg['environment'], Dumper=RoundTripDumper)), cfg['environment'])

# shortcuts
ob_dim = env.num_obs
act_dim = env.num_acts
device = 'cpu'

# make directory where the text weights will be saved
os.makedirs(dir_path, exist_ok=True)

# networks
actor = full['actor_architecture_state_dict']

loaded_actor = ppo_module.MLP(shape=cfg['architecture']['policy_net'],
                              actionvation_fn=nn.LeakyReLU,
                              input_size=ob_dim,
                              output_size=act_dim)
loaded_actor.load_state_dict(actor)

# actor txt
print("/// actor")
actor_txt = open(dir_path + "/actor.txt", 'w')
content = ''
for w in actor.items():
    if w[0].startswith('architecture'):
        print(w[0])
        w = w[1]
        if w.ndim == 2:
            w = w.transpose(0, 1).flatten()
        for i in w:
            content += str(i.item()) + ", "

actor_txt.write(content[:-2])

scale_files = {scale_path + "/mean" + iteration + ".csv": dir_path + "/mean" + iteration + ".csv",
               scale_path + "/var" + iteration + ".csv": dir_path + "/var" + iteration + ".csv"}
for source_file, target_file in scale_files.items():
    shutil.copy(source_file, target_file)