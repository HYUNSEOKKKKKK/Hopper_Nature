"""Replay a Hopper checkpoint on a fixed flat-ground command schedule."""

import argparse
import math
from pathlib import Path
import sys
import time

if __package__ in (None, ''):
    sys.path.insert(0, str(Path(__file__).resolve().parents[4]))

from raisimGymTorch.helper.hopper_config import REPO_ROOT, checkpoint_path, load_config

COMMAND_SCHEDULE = [
    (2.0, 0.00, 0.0, 0.0),
    (2.0, -0.25, 0.0, 0.0),
    (2.0, 0.00, 0.0, 0.0),
    (2.0, -0.30, 0.0, 0.0),
    (2.0, 0.00, 0.0, 0.0),
    (2.0, -0.40, 0.0, 0.0),
    (2.0, 0.00, 0.0, 0.0),
    (2.0, -0.50, 0.0, 0.0),
    (2.0, 0.00, 0.0, 0.0),
    (2.0, -0.60, 0.0, 0.0),
    (2.0, 0.00, 0.0, 0.0),
]


def command_at(t):
    phase = t % sum(item[0] for item in COMMAND_SCHEDULE)
    for duration, vx, vy, yaw in COMMAND_SCHEDULE:
        if phase < duration:
            return vx, vy, yaw
        phase -= duration
    return 0.0, 0.0, 0.0


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('-w', '--weight', type=checkpoint_path, required=True)
    parser.add_argument('--config', type=Path, help='override the checkpoint cfg.yaml')
    parser.add_argument('--duration', type=float, default=22.0, help='simulation seconds')
    parser.add_argument('--headless', action='store_true')
    parser.add_argument('--no-realtime', action='store_true', help='run without playback pacing')
    args = parser.parse_args()
    if not math.isfinite(args.duration) or args.duration <= 0:
        parser.error('--duration must be a finite positive number')

    import numpy as np
    import torch
    from ruamel.yaml import dump, RoundTripDumper
    from raisimGymTorch.env.bin import hopper
    from raisimGymTorch.env.RaisimGymVecEnv import RaisimGymVecEnv
    from raisimGymTorch.algo.ppo.module import MLP

    cfg = load_config(args.config or args.weight.parent / 'cfg.yaml',
                      num_envs=1, num_threads=1, headless=args.headless)
    if not args.headless:
        cfg['environment']['render'] = True
    env = RaisimGymVecEnv(hopper.RaisimGymEnv(
        str(REPO_ROOT / 'rsc'), dump(cfg['environment'], Dumper=RoundTripDumper)))
    try:
        env.seed(cfg['seed'])
        checkpoint = torch.load(args.weight, map_location='cpu', weights_only=True)
        actor = MLP(cfg['architecture']['policy_net'], torch.nn.LeakyReLU,
                    env.num_obs + env.num_est, env.num_acts)
        estimator = MLP(cfg['architecture']['estimator_net'], torch.nn.LeakyReLU,
                        env.num_obs, env.num_est)
        actor.load_state_dict(checkpoint['actor_architecture_state_dict'])
        estimator.load_state_dict(checkpoint['estimator_architecture_state_dict'])
        actor.eval()
        estimator.eval()
        iteration = args.weight.stem.split('_')[1]
        env.load_scaling(str(args.weight.parent), iteration)
        env.set_terrain(0, 0.0, 0.7)
        env.set_initial(0)
        env.reset()
        env.turn_on_visualization()
        dt = cfg['environment']['control_dt']
        reward_sum, terminations = 0.0, 0
        steps = math.ceil(args.duration / dt)
        print('Loaded checkpoint:', args.weight)
        for step in range(steps):
            started = time.perf_counter()
            command = command_at(step * dt)
            env.set_command(*command)
            with torch.no_grad():
                obs = env.observe(False)
                estimate = estimator.architecture(torch.from_numpy(obs)).numpy()
                action = actor.architecture(torch.from_numpy(np.hstack((obs, estimate)))).numpy()
            reward, done, _ = env.step(action)
            reward_sum += float(reward[0])
            terminations += int(done[0])
            if step % 100 == 0:
                print(f't={step * dt:6.2f}s command={command} terminated={bool(done[0])}')
            if not args.no_realtime:
                time.sleep(max(0, dt - (time.perf_counter() - started)))
        print(f'Completed {steps} steps; mean reward={reward_sum / steps:.6f}; '
              f'episode terminations={terminations}')
    finally:
        env.turn_off_visualization()
        env.close()


if __name__ == '__main__':
    main()
