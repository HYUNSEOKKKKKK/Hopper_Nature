###             ###
### From RaiLab ###
###             ###
import os
import shutil
import torch

# weight path
full_path = "/home/gijeong/workspace/raisimLib/hound/raisimGymTorch/data/hound/2023-12-15-16-23-38/full_8000.pt"
full = torch.load(full_path)
iteration = full_path.rsplit('/')[-1].rsplit('_')[-1].rsplit('.')[0]
scale_path = '/'.join(full_path.rsplit('/')[:-1])
dir_path = '/'.join(full_path.rsplit('/')[:-1]) + "/network_" + iteration
weight_dir = full_path.rsplit('/', 1)[0] + '/'

# make directory where the text weights will be saved
os.makedirs(dir_path, exist_ok=True)

# networks
actor = full['actor_architecture_state_dict']
estimator = full['estimator_architecture_state_dict']

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

# estimator txt
print("/// estimator")
estimator_txt = open(dir_path + "/estimator.txt", 'w')
content = ''
for w in estimator.items():
    if w[0].startswith('architecture'):
        print(w[0])
        w = w[1]
        if w.ndim == 2:
            w = w.transpose(0, 1).flatten()
        for i in w:
            content += str(i.item()) + ", "

estimator_txt.write(content[:-2])

# for obs
scale_files = {scale_path + "/mean" + iteration + ".csv": dir_path + "/mean" + iteration + ".csv",
               scale_path + "/var" + iteration + ".csv": dir_path + "/var" + iteration + ".csv"}
for source_file, target_file in scale_files.items():
    shutil.copy(source_file, target_file)