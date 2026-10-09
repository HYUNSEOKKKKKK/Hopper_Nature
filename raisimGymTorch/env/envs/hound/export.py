###             ###
### From RaiLab ###
###             ###
import os
import shutil
import torch

# directories
task_path = os.path.dirname(os.path.realpath(__file__))
home_path = task_path + "/../../../../.."

# weight path (runner.py 가 체크포인트를 쓰는 위치. run 폴더 이름만 바꿔 쓰면 됨)
full_path = home_path + "/hound/raisimGymTorch/data/dhal_one_leg/2026-10-08-12-50-29/full_10000.pt"
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