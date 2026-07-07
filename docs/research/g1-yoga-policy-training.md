# Training a physics-on "yoga teacher" balance policy for the Unitree G1

**Date:** 2026-07-07
**Status:** Research synthesis + recommendation (pre-implementation)
**Scope:** How to train an RL policy that keeps our Unitree G1 (29-DOF) upright while
holding a set of target joint poses ("yoga poses"), given our existing MuJoCo-based
setup and an Apple-Silicon (M5) development machine.

> **Provenance.** Produced by the `deep-research` workflow (run `wf_6eaf20a7-a39`):
> 6 web-search agents → 23 sources fetched with extracted claims. The workflow's final
> adversarial-verify/synthesis step crashed mid-run; this document was synthesized by
> hand from the recovered per-source claims and cross-checked against first-hand
> knowledge. Individual claims are source-extracted, not independently re-verified —
> treat quantitative details (hyperparameters, wall-clock) as indicative.

---

## 1. Our setup (the constraints that drive everything)

- **Sim:** the official `unitree_mujoco` G1 **29-DOF** asset, MuJoCo 3.3.6, **torque
  (motor) actuators** (ctrlrange ±88–139 Nm), 0.003 s timestep, running in Docker
  behind an HTTP/WS protocol. We already have a gravity-compensated PD `hold`
  controller and a named-pose registry.
- **Compute:** MacBook Pro **M5, Apple Silicon (arm64), 15 cores, no CUDA.**
- **Also available:** an Isaac Sim + Eureka MCP (`cyberneticphysics`) — GPU-cloud, not local.

The task is **pose-conditioned whole-body balance**: track a target joint configuration
while keeping the pelvis upright and feet under the center of mass. This is the
DeepMimic / ExBody / OmniH2O / HuB family of "reference-tracking + balance" control,
*not* velocity locomotion.

---

## 2. Training-stack landscape — the fast G1 paths all assume NVIDIA

| Stack | Engine / RL algo | G1 asset | Local on M5? | Notes |
|---|---|---|---|---|
| [MuJoCo Playground](https://github.com/google-deepmind/mujoco_playground) ([arXiv 2502.08844](https://arxiv.org/pdf/2502.08844)) | MJX + Brax PPO (JAX) | ✅ ready | ⚠️ GPU-oriented | G1 locomotion in **<30 min on 2× RTX 4090**, 8192 envs, ONNX @ 50 Hz. JAX-Metal on Apple GPU is experimental; MJX-on-CPU is slow. |
| [`unitree_rl_mjlab`](https://github.com/unitreerobotics/unitree_rl_mjlab) | mjlab / **MuJoCo-Warp (CUDA)** + PPO | ✅ Flat + 23-DoF | ❌ needs NVIDIA | Native MuJoCo physics, 4096 envs, ONNX export, **motion-imitation tasks**, sim2sim via `unitree_mujoco`. |
| [`unitree_rl_gym`](https://github.com/unitreerobotics/unitree_rl_gym) | legged_gym + rsl_rl PPO (Isaac Gym) | ✅ `--task=g1` | ❌ trains on Isaac Gym | Built-in `deploy_mujoco` sim2sim, exports `.pt`, Isaac→MuJoCo→real pipeline. |
| [`unitree_rl_lab`](https://github.com/unitreerobotics/unitree_rl_lab) / [Isaac Lab](https://isaac-sim.github.io/IsaacLab/main/source/overview/reinforcement-learning/rl_frameworks.html) | Isaac Lab + rsl_rl PPO | ✅ 29-DoF Velocity | ❌ needs Isaac Sim | Whole-body tracking punted to external `whole_body_tracking`. SB3/RL-Games/RSL-RL selectable. |
| [`mujocolab/g1_spinkick_example`](https://github.com/mujocolab/g1_spinkick_example) | MuJoCo-Warp | ✅ | ❌ needs NVIDIA | Reference-motion imitation (spin kick), 4096 envs × 20k iters on 1 GPU, ONNX deploy. |

**Takeaway:** every turnkey G1 trainer is GPU-parallel (Isaac Gym / MJX / MuJoCo-Warp).
None trains locally on an M5 out of the box.

---

## 3. Reward & action formulation — strong cross-paper consensus

**Action space.** Universally **PD joint-position targets**, offset from the default
standing pose and per-joint scaled — *not* raw torques:
- `unitree_rl_mjlab`: per-joint scales (hips ≈0.55, wrists ≈0.07); legs Kp ≈40–99 / Kd ≈2.6–6.3, arms Kp ≈14–17 / Kd ≈0.9–1.1. ([DeepWiki](https://deepwiki.com/unitreerobotics/unitree_rl_mjlab/7.2-g1))
- [ExBody2](https://arxiv.org/abs/2412.13196), [HuB](https://arxiv.org/html/2505.07294): action = 23/29-D joint targets, **policy @ 50 Hz over a PD loop @ 500 Hz** (two-rate control).
- A torque-action alternative exists ([arXiv 2304.09434](https://arxiv.org/pdf/2304.09434), RA-L 2023): claims better sim-to-real compliance and suggests **pre-training upright via gravity compensation before task learning** — notable because we already have gravity comp.

**Observations.** Base angular velocity, projected gravity, joint pos/vel relative to
default, last action, **+ target-pose conditioning** (target joint angles + body-keypoint
positions), with a **5–25-step history**. Asymmetric actor-critic: the critic gets
privileged ground truth (base linear velocity, etc.). ([ExBody2], [HuB], `unitree_rl_lab`)

**Reward (the important part for our toppling poses):**
- Exponential tracking: DoF-position `exp(-0.7·|q_ref−q|)` (w≈3), keypoint `exp(-|p_ref−p|)` (w≈2). ([ExBody2](https://arxiv.org/abs/2412.13196))
- **Balance shaping** — [HuB](https://arxiv.org/html/2505.07294) is the most directly relevant: a **high-weight COM term keeping the COM projection inside the support polygon**, a **foot-contact-mismatch penalty**, a **close-feet penalty**, and **relaxed** position tracking (σ≈0.6) so the policy may sacrifice pose fidelity to stay upright.
- **[ExBody](https://arxiv.org/abs/2402.16796) decoupling** (the key idea for us): **upper body tracks the reference closely; the legs relax tracking and instead optimize for balance.** This is exactly what will rescue poses that currently topple.
- Regularization: joint-velocity, joint-acceleration, action-rate, torque penalties; alive bonus ≈0.15.

---

## 4. PPO hyperparameters & realistic wall-clock

- PPO: lr **1e-4–5e-4**, clip **0.2**, γ **0.99**, **5 epochs**, **4 minibatches**, batch **4096**. ([ExBody2](https://arxiv.org/abs/2412.13196), [unitree_g1_rl_lab](https://github.com/mintlabkorea/unitree_g1_rl_lab))
- Parallelism: **4096–8192 envs** on GPU.
- Wall-clock (GPU): locomotion in **minutes**; **whole-body motion tracking ~3 days (teacher) + ~1 day (student) on a single RTX 4090** ([GMT](https://arxiv.org/html/2506.14770v1)). Teacher–student (PPO teacher → DAgger student) is standard for sim-to-real but **not needed for a sim-only deployment**.
- Domain randomization (sim-to-real only): friction, link mass 0.7–1.3×, PD gains 0.75–1.25×, control delay 20–60 ms, torque RFI, IMU noise, pushes. ([HuB](https://arxiv.org/html/2505.07294))

---

## 5. Sim-to-sim transfer is a documented failure mode

An Isaac-Lab-trained G1 policy **collapses immediately when deployed into MuJoCo**
([IsaacLab issue #3751](https://github.com/isaac-sim/IsaacLab/issues/3751), Oct 2025):
joint-ordering, actuator-model, action-scaling, and observation-dimension mismatches
between the training env and MuJoCo. No maintainer fix.

**Implication:** **train in the same engine you deploy in.** Since we deploy in MuJoCo,
train in MuJoCo — this sidesteps the entire class of transfer bugs. (`unitree_rl_gym`,
`unitree_rl_mjlab`, and `unitree_rl_lab` all *validate* in `unitree_mujoco` before real
hardware precisely because this gap is real.)

---

## 6. Eureka (the available MCP) is not a reliable win here

[Eureka](https://arxiv.org/abs/2310.12931) (GPT-4 evolutionary reward-code search) beats
human-expert rewards on **83% of tasks** overall — but a 2025 study
([arXiv 2511.19355](https://arxiv.org/html/2511.19355)) found the **Humanoid task
specifically defeated both Eureka and newer LLM-reward methods**: LLM reward design
struggles on high-DOF humanoid morphologies, is **high-variance (~10 runs to find a good
candidate)**, and Eureka is built on Isaac Gym. A hand-crafted HuB/ExBody-style reward is
the safer, cheaper bet for our balance task.

---

## 7. Recommendation for our M5

**Train natively in MuJoCo on CPU; deploy into the same Docker sim → zero sim-to-sim gap.**

Rationale: no CUDA locally rules out the GPU trainers, and the Isaac→MuJoCo transfer gap
is a known trap regardless. Crucially, **our task is far easier than locomotion** — hold a
target pose, with each episode **initialized at that pose** — which collapses the
exploration problem and makes CPU PPO across the M5's cores tractable (order tens of
minutes to ~an hour per pose-set, not days).

**Proposed architecture:**
1. **`gymnasium` env** wrapping our exact G1 MJCF (`mujoco` Python, CPU):
   - *Action*: PD joint-position targets, offset from the default pose, per-joint scaled; internal PD @ ~500 Hz, policy @ 50 Hz.
   - *Obs*: base angular velocity, projected gravity, joint pos/vel vs default, last action, + target-pose conditioning (target joint angles).
   - *Reward*: ExBody-decoupled tracking (upper body tight, legs relaxed) + HuB balance (COM-in-support, upright, foot contact, alive) + action-rate/torque/accel regularization.
   - *Reset*: initialize at (or near) the target pose from our `NAMED_POSES`.
2. **PPO via [stable-baselines3](https://github.com/DLR-RM/stable-baselines3)**, CPU `SubprocVecEnv` (~8–15 envs); **curriculum** from statically-stable poses → harder balances.
3. **Export ONNX**; add a **policy-control mode** to the sim that replaces the PD `hold` at 50 Hz.
4. Reuse the existing `fallen`/`pelvis_height` signals for episode termination and eval.

**Environment note:** host Python is 3.14 (no torch/SB3 wheels yet). Training should run
in a dedicated **Python 3.12 venv** (via `uv`).

**Deferred (needs GPU):** if we later want the *hard* balance poses (deep squats,
one-legged tree) at locomotion-grade robustness, port the same env to `unitree_rl_mjlab`
(MuJoCo-Warp) or MuJoCo Playground and train on a cloud/Isaac GPU, then ONNX back into
our MuJoCo sim.

---

## Sources

- MuJoCo Playground — https://github.com/google-deepmind/mujoco_playground · [arXiv 2502.08844](https://arxiv.org/pdf/2502.08844) · [technical report](https://playground.mujoco.org/assets/playground_technical_report.pdf)
- `unitree_rl_gym` — https://github.com/unitreerobotics/unitree_rl_gym
- `unitree_rl_mjlab` — https://github.com/unitreerobotics/unitree_rl_mjlab · [DeepWiki G1](https://deepwiki.com/unitreerobotics/unitree_rl_mjlab/7.2-g1)
- `unitree_rl_lab` — https://github.com/unitreerobotics/unitree_rl_lab · [DeepWiki G1 29-DoF](https://deepwiki.com/unitreerobotics/unitree_rl_lab/6.3-g1-humanoid-(29-dof))
- `unitree_g1_rl_lab` — https://github.com/mintlabkorea/unitree_g1_rl_lab
- `mujocolab/g1_spinkick_example` — https://github.com/mujocolab/g1_spinkick_example
- G1-Playground — https://github.com/AlexandreBrown/G1-Playground
- IsaacLab issue #3751 (Isaac→MuJoCo collapse) — https://github.com/isaac-sim/IsaacLab/issues/3751
- Isaac Lab RL frameworks — https://isaac-sim.github.io/IsaacLab/main/source/overview/reinforcement-learning/rl_frameworks.html
- HuB: Learning Extreme Humanoid Balance — [arXiv 2505.07294](https://arxiv.org/html/2505.07294)
- ExBody — [arXiv 2402.16796](https://arxiv.org/abs/2402.16796)
- ExBody2 — [arXiv 2412.13196](https://arxiv.org/abs/2412.13196)
- GMT: General Motion Tracking — [arXiv 2506.14770](https://arxiv.org/html/2506.14770v1)
- KungfuBot — [arXiv 2506.12851](https://arxiv.org/html/2506.12851v1)
- Torque-based Deep RL for Bipedal Robots (RA-L 2023) — [arXiv 2304.09434](https://arxiv.org/pdf/2304.09434)
- Sim-to-Real via Joint-Torque-Space Perturbation — [arXiv 2504.06585](https://arxiv.org/pdf/2504.06585)
- Eureka — [arXiv 2310.12931](https://arxiv.org/abs/2310.12931) · https://github.com/eureka-research/Eureka
- LLM reward-design limits on Humanoid — [arXiv 2511.19355](https://arxiv.org/html/2511.19355)
