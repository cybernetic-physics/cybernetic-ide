# G1 Yoga RL

`g1-yoga-rl` is the research package for the Cybernetic IDE Unitree G1
"yoga teacher" balance-policy track.

It is intentionally small right now. The goal is to turn the LocoMuJoCo audit
into runnable, reviewable artifacts before training a policy:

- project Cybernetic's 29-DOF yoga pose registry onto LocoMuJoCo's reduced
  Unitree G1 joint set;
- generate smooth target trajectories that can become LocoMuJoCo
  `Trajectory` objects;
- benchmark local CPU MuJoCo and MJX stepping speed on the Mac;
- keep all assumptions explicit so a later trained policy can be deployed back
  into the Dockerized Cybernetic IDE MuJoCo sim without joint-order surprises.

## Install

From the Cybernetic IDE repo root:

```sh
python3 -m pip install -e packages/g1-yoga-rl
```

For LocoMuJoCo-backed scripts, use a Python 3.12 environment and install
LocoMuJoCo from the local clone:

```sh
~/.local/bin/uv venv --python 3.12 .venv-rl
source .venv-rl/bin/activate
uv pip install -e ~/wagmi/loco-mujoco
uv pip install -e packages/g1-yoga-rl
```

## Commands

Project the Cybernetic simulator pose registry onto the LocoMuJoCo Unitree G1
joint set:

```sh
g1-yoga-project-poses --output .runtime/g1-yoga-rl/yoga_pose_projection.json
```

Generate a LocoMuJoCo trajectory NPZ from those projected poses:

```sh
g1-yoga-make-trajectory --output .runtime/g1-yoga-rl/yoga_trajectory.npz
```

Check that LocoMuJoCo's imitation factory accepts a generated trajectory:

```sh
g1-yoga-check-imitation --traj .runtime/g1-yoga-rl/yoga_trajectory.npz
```

Start an experimental PPOJax mimic-training run on the generated trajectory:

```sh
g1-yoga-train \
  --traj .runtime/g1-yoga-rl/yoga_trajectory.npz \
  --out .runtime/g1-yoga-rl/runs/smoke \
  --total-timesteps 1000000 \
  --num-envs 64
```

Export a trained PPOJax agent to a plain NumPy policy bundle for future
simulator deployment:

```sh
g1-yoga-export \
  --agent .runtime/g1-yoga-rl/runs/smoke/PPOJax_saved.pkl \
  --out .runtime/g1-yoga-rl/policies/yoga_policy.npz
```

Render one frame per pose hold for visual QA:

```sh
g1-yoga-render-frames \
  --traj .runtime/g1-yoga-rl/yoga_trajectory.npz \
  --out-dir .runtime/g1-yoga-rl/frames
```

Analyze the static support margin for each source pose before training:

```sh
g1-yoga-analyze-stability \
  --render-dir .runtime/g1-yoga-rl/stability-frames
```

Numerically solve planted-foot ankle pitch/roll and selected lunge reach joints
when tuning the simulator's `NAMED_POSES` registry:

```sh
g1-yoga-tune-feet
```

Benchmark local stepping speed:

```sh
g1-yoga-bench-env --mode both --num-envs 16 --num-steps 100
```

Generated `.npz` datasets are intentionally not committed. Store them under
`.runtime/g1-yoga-rl/` or regenerate them from the scripts.

## Why This Exists

The simulator can already apply yoga poses kinematically and hold easy poses
with a gravity-compensated PD loop. Hard poses still topple because there is no
whole-body balance controller. The policy track answers:

- What is the smallest controller that can hold those poses with physics on?
- Can we train and deploy in MuJoCo end-to-end to avoid Isaac-to-MuJoCo transfer
  issues?
- Can the resulting policy be exposed through the same lowcmd/named-joint
  surfaces already available to Cybernetic IDE agents?

This package is not yet the final trainer. It is the scaffolding that makes the
training path reproducible.
