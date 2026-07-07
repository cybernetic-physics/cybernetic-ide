# Next-session prompt: train the G1 "yoga teacher" balance policy (LocoMuJoCo path)

> Copy-paste everything below this line into a fresh session as the opening prompt.

---

Continue the Unitree G1 "yoga teacher" work in `~/wagmi/cyber-ide`. Your goal this
session: **build and run the policy-training system that lets the G1 hold yoga poses
with physics on**, using LocoMuJoCo as the training stack, then deploy the trained
policy back into our Docker MuJoCo sim.

## Read these first (in order)

1. `docs/research/g1-yoga-policy-training.md` — the research synthesis: reward design
   (ExBody upper/lower decoupling, HuB COM-in-support balance shaping), PPO settings,
   why we train in MuJoCo and deploy in MuJoCo (Isaac→MuJoCo transfer is a documented
   failure mode), and why Eureka is skipped.
2. `todo.md` — the broader robotics roadmap from a parallel session; our work is the
   "Locomotion And Balance" research track ("What is the smallest controller that can
   hold a yoga pose without toppling?").
3. `docs/src/upstream-robotics-audit.md` — what upstream repos are cloned in `~/wagmi`
   and which SDK facades already exist.
4. Claude memory note `g1-sim-pose-registry` — sim pose registry mechanics and the
   Docker rebuild gotcha.

## Current state (committed and pushed on `main` through the current robotics
SDK/MCP slices)

- **Sim**: official `unitree_mujoco` G1 **29-DOF** (torque actuators), MuJoCo 3.3.6,
  Dockerized (`cyber/unitree-g1-mujoco-protocol:0.1.0`), HTTP :38383 / WS :8788.
  Source: `overlays/unitree-g1-mujoco-protocol/python/g1_protocol_sim.py`.
  - 9 yoga poses in `NAMED_POSES` (mountain, upward_salute, forward_fold, chair,
    warrior_one, warrior_two, goddess, tree, namaste), auto-grounding, smooth
    qpos-interpolated transitions (`animate_to_pose`).
  - Gravity-compensated PD `hold_pose` (physics on). Measured: mountain /
    upward_salute / namaste **hold**; chair / goddess / forward_fold / warriors / tree
    **topple** (no balance controller — this is the gap the policy fills).
  - Fall detection in `/status`: `fallen` (pelvis z < 0.45 or torso up-axis < 0.5),
    `pelvis_height`; `control_mode` field.
  - A `rt/lowcmd`-shaped command surface (per-motor `tau/kp/kd/q/dq`) — added by a
    parallel session; a natural actuation path for policy deployment. The current
    local approximation applies commanded joint targets as held MuJoCo frames and
    records `mode_pr`, `mode_machine`, CRC, accepted, clamped, and ignored command
    metadata for debugging. It is not yet official CycloneDDS transport or a
    whole-body low-level controller.
- **Client**: `packages/cybernetic-robotics` — `G1Robot.pose(smooth=)`, `.hold()`,
  `.is_fallen()`, snapshots, camera; Unitree SDK2-shaped shims (arm, loco, lowcmd).
- **Demo**: `examples/yoga_teacher.py` (physics mode with reset-on-fall; `--posed`
  kinematic mode).
- **Gotcha**: the sim script is baked into the Docker image. After editing it:
  `docker build --platform linux/arm64 -t cyber/unitree-g1-mujoco-protocol:0.1.0
  overlays/unitree-g1-mujoco-protocol` then `docker compose --env-file
  .runtime/unitree-g1-mujoco/compose.env -f
  overlays/unitree-g1-mujoco-container/compose.yaml up -d --force-recreate
  unitree-g1-mujoco`. A plain restart reuses the old image. The
  `mcp__cybernetic-robotics__*` MCP tools drive the sim (sim_status,
  python_control_run, viewer_snapshot, etc.).

## Why LocoMuJoCo (reviewed at `~/wagmi/loco-mujoco`)

It closes the biggest build-vs-buy gaps — we don't have to write the env, the
tracking reward, or the PPO harness:

- `loco_mujoco/environments/humanoids/unitreeG1.py` (+ `unitreeG1_mjx.py`): ready
  UnitreeG1 env, same joint naming as our sim. **Caveat: it is the reduced-DOF G1**
  (wrists roll-only, waist yaw-only) vs our 29-DOF sim — poses must be projected onto
  its joint set (ours barely use the extra DOFs; forward_fold's `waist_pitch_joint`
  is the main casualty), or pass our 29-DOF MJCF via the env's `spec` argument.
- **Custom trajectories**: `examples/tutorials/10_creating_custom_traj.py` +
  `CustomDatasetConf` + `ImitationFactory` — build a reference motion programmatically
  from qpos/qvel arrays. **Our yoga flow is exactly this**: synthesize the trajectory
  by smooth-interpolating through `NAMED_POSES` targets with holds (the same
  smoothstep math already in `animate_to_pose`), qvel by finite differences.
- **Mimic training**: `examples/training_examples/jax_rl_mimic/` — PPOJax +
  DeepMimic-style tracking reward, hydra `conf.yaml`, eval script. Also `jax_amp`,
  `jax_gail` if plain mimic tracking underperforms.
- Tutorials for PD control types (`07_changing_control_type.py`) and domain
  randomization (`08_domain_randomization.py`).
- Its `CLAUDE.md` documents install (`pip install -e .`), dataset CLIs, and
  architecture (CPU MuJoCo base + MJX/JAX for parallel).

## Hardware / environment constraints

- MacBook Pro **M5 (arm64), 15 cores, no CUDA**. JAX runs on the CPU backend
  (fine); MJX-on-CPU works but is slow — prefer moderate env counts; do NOT
  attempt jax-metal (experimental).
- Host python is **3.14 — torch/jax wheels likely missing. Create a Python 3.12
  venv with `uv`** (installed at `~/.local/bin/uv`), e.g.
  `uv venv --python 3.12 .venv-rl && uv pip install -e ~/wagmi/loco-mujoco`.
- Our task is much easier than locomotion: episodes can initialize AT the target
  pose (collapses exploration). Expect useful CPU results in minutes–hours, not days.

## Suggested plan (adapt as you learn)

1. Set up the 3.12 venv; install loco-mujoco (editable, from `~/wagmi/loco-mujoco`);
   verify `UnitreeG1` env steps on CPU.
2. Write a trajectory generator: `NAMED_POSES` (projected to the env's joint set) →
   smooth glide + hold sequence → `Trajectory` (tutorial 10 shape) → save as npz.
   This is now scaffolded in `packages/g1-yoga-rl/` via `g1-yoga-project-poses`
   and `g1-yoga-make-trajectory`; replay it in the env to visually verify
   before training.
3. Train mimic PPO on the trajectory (start from `jax_rl_mimic/conf.yaml`; wandb
   optional — can disable). Start with the statically-stable poses as a sanity
   curriculum, then the full flow. Watch: does it hold chair/goddess/tree?
4. Eval + export: run `eval.py`; export the policy (ONNX preferred, or a numpy-weights
   MLP forward pass) for 50 Hz inference.
5. Deploy back into our Docker sim: add a `policy` control mode to
   `g1_protocol_sim.py` (alongside `hold`/`lowcmd`) that runs the exported policy at
   50 Hz over the existing PD/torque path; wire a `yoga_policy` command; update
   `examples/yoga_teacher.py` with a `--policy` mode. Mind obs/action ordering and
   scaling — the Isaac issue #3751 failure mode (dimension/order mismatch) applies to
   ANY cross-runtime transfer, including LocoMuJoCo→our sim. Write the obs builder
   once, test it against recorded states from both sides.
6. Validate end-to-end with `yoga_teacher.py --policy`: count held vs fallen poses vs
   the PD baseline (3/9 held). Snapshot each pose via the MCP viewer tools.
7. Commit in slices (repo guidelines: no `mod.rs`-style noise, imperative
   titles). New training code lives in `packages/g1-yoga-rl/`; document in
   `docs/research/`.

## Honest expectations

- PD-hold baseline: 3/9 poses. A mimic policy should rescue chair/goddess/warriors
  (two-foot, CoM-shiftable). Tree (single-leg) is genuinely hard; treat it as stretch.
- If CPU training stalls, fall back to: fewer envs + shorter horizon + per-pose
  policies (one policy per pose is perfectly acceptable for the demo), or rent a GPU
  box later with the identical LocoMuJoCo config.
