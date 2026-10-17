# Training and replay

After [installation](installation.md), activate the environment and enter the Hopper task directory:

```bash
conda activate hopper_nature
cd ~/Hopper_Nature/raisimGymTorch/env/envs/hopper
```

Run the commands below from this directory.

## First run

```bash
python runner.py \
  --num-envs 4 --num-threads 2 --iterations 2 \
  --headless --no-tensorboard
```

Two updates exercise native simulation, estimator inference, rollout collection, PPO optimization and checkpoint writing. This is a software check, not a trained hopping policy. At iteration 0 the checkpoint is written before the first update.

## Train a policy

```bash
python runner.py \
  --num-envs 400 --num-threads 16 --headless --no-tensorboard
```

| Option | Default / behavior |
| --- | --- |
| `--config PATH` | Task `cfg.yaml`; override it with your own YAML |
| `--num-envs N` | YAML default: 400 |
| `--num-threads N` | YAML default: 60; select a value suitable for your CPU |
| `--iterations N` | 10001 updates, numbered 0–10000 |
| `--seed N` | YAML default: 1 |
| `--log-dir PATH` | `raisimGymTorch/data/hopper` in this checkout |
| `--headless` | Disable the viewer server, recording and evaluation pacing |
| `--viewer` | Start the Unity inside this checkout, wait for connection, and close it on exit |
| `--no-tensorboard` | Skip automatic TensorBoard server/browser; event logging still runs |

The configuration uses 0.002 s physics steps, 0.01 s control steps and a 4 s rollout: 400 transitions per environment per PPO update. With 400 environments, that is 160,000 transitions per update. PPO uses four epochs, four minibatches, discount 0.99 and GAE lambda 0.95. The AdamP learning rate starts at 0.0003 and is multiplied by 0.75 at update 1000. These are configuration values, not measured performance.

For visual training use `python runner.py --viewer`; it opens the project's own client. The original `python runner.py` command also works with a manually started local viewer. Automatic TensorBoard uses port 6700; `--no-tensorboard` avoids conflicts when running several experiments.

```bash
tensorboard --logdir ~/Hopper_Nature/raisimGymTorch/data/hopper --port 6700
```

## Saved artifacts

Each run writes its **effective** configuration, including CLI overrides:

```text
raisimGymTorch/data/hopper/<timestamp>/
├── cfg.yaml
├── Environment.hpp
├── full_0.pt
├── mean0.csv
├── var0.csv
├── full_1000.pt
├── mean1000.csv
├── var1000.csv
└── events.out.tfevents...
```

Checkpoints contain actor, action distribution, task critic, barrier critic, estimator and optimizer state dictionaries. Saving occurs before rollout/optimization at updates divisible by `environment.eval_every_n` (1000 by default). The two-update check therefore produces `full_0.pt`, rather than a checkpoint after its second update.

Keep the checkpoint, its matching observation mean and variance, and `cfg.yaml` together. Archive `Environment.hpp`, source commit, dependency versions, seed and hardware with final experiments. Experiment directories and weight files are excluded from Git. A selected pretrained result can later be distributed as a release asset with a checksum.

With `record_video: yes` and rendering enabled, evaluation rollouts are also recorded by the connected Unity client. Start the project-local client with `--viewer`; its MP4s go to `.local/runtime/raisimLib/raisimUnity/linux/Screenshot/` inside this checkout, separate from checkpoint bundles. The default evaluation rollout is 4 simulated seconds, recorded at updates 0, 1000, 2000, and so on. `--headless` disables recording. Unity logs from automatic launches are under `.local/logs/`.

The public runner trains **from scratch**. The old `--mode retrain` branch was commented out; exact training resume is not implemented. Restoring network/optimizer weights alone would not restore curriculum, scheduler, observation/state running statistics, or random generators. Do not describe that as an exact continuation.

## Replay

```bash
python tester.py --viewer \
  --weight /path/to/run/full_10000.pt --duration 22
```

Replay reads `cfg.yaml` beside the checkpoint, creates one environment and one thread, loads matching observation statistics, then runs deterministic network inference. Simulation reset and physics still include environment randomization. Seed selection does not guarantee identical trajectories across parallel runs, hardware or dependency versions.

Use `--headless --no-realtime` for a check without a viewer. Use `--config PATH` only when deliberately overriding the checkpoint configuration. A headless training configuration is automatically rendered during visual replay unless `--headless` is supplied.

`--viewer` opens this checkout's Unity and waits for connection before replay begins. It is mutually exclusive with `--headless`. For manual startup use `python tester.py` without `--viewer` and open the project-local Unity in another terminal. The tester does not start automatic video recording.

The fixed 22 s flat-ground schedule is:

| Simulation time | Body-frame command `(vx, vy, yaw rate)` |
| --- | --- |
| 0–2 s | `(0, 0, 0)` |
| 2–4 s | `(-0.25, 0, 0)` |
| 4–6 s | `(0, 0, 0)` |
| 6–8 s | `(-0.30, 0, 0)` |
| 8–10 s | `(0, 0, 0)` |
| 10–12 s | `(-0.40, 0, 0)` |
| 12–14 s | `(0, 0, 0)` |
| 14–16 s | `(-0.50, 0, 0)` |
| 16–18 s | `(0, 0, 0)` |
| 18–20 s | `(-0.60, 0, 0)` |
| 20–22 s | `(0, 0, 0)` |

Longer replays repeat the schedule. Linear units are m/s; angular units are rad/s. The console reports mean reward and termination count. Vectorized simulation automatically resets a terminated episode, so a completed replay does not establish uninterrupted successful hopping.

## Export

```bash
python export.py --weight /path/to/run/full_10000.pt
```

The default destination is `network_10000/` beside the checkpoint. Override it with `--output /path/to/export`. The text format transposes each linear weight matrix before flattening it, then appends its bias in state dictionary order. CSVs are copied unchanged. This preserves the historical text layout; the task’s `export.py` now accepts an explicit checkpoint path.

Export is not a complete hardware controller. See the [method and interface](method.md) before downstream integration.
