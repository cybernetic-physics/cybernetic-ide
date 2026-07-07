# G1 yoga policy training — session working log

**Date:** 2026-07-07
**Status:** In progress (live working notes; updated as the session advances)
**Context:** Executes the plan in `g1-yoga-policy-handoff-prompt.md`, informed by
`g1-yoga-policy-training.md`. Goal: train a LocoMuJoCo mimic policy that lets the
G1 hold yoga poses with physics on, then deploy it into our Docker MuJoCo sim.

---

## Decisions made (and why)

### 1. Training stack: LocoMuJoCo PPOJax + MJX on CPU (confirmed viable)

Benchmarks on the M5 (15 cores, JAX 0.10.2 CPU backend, MuJoCo 3.10,
Python 3.12 venv at `.venv-rl/`, benchmark script
`packages/g1-yoga-rl/scripts/bench_env_speed.py`):

| Path | Throughput |
|---|---|
| CPU `UnitreeG1` env, single process | ~1,740 env-steps/s |
| MJX `MjxUnitreeG1`, jit+vmap, 16 envs | ~4,900 env-steps/s |
| MJX, 64 envs | ~4,500 env-steps/s |
| MJX, 256 envs | ~5,300 env-steps/s |

MJX-on-CPU plateaus around ~5k env-steps/s regardless of batch width (XLA CPU
doesn't parallelize the physics across cores well). That is still enough:
~10M steps ≈ 35 min inside the fully-jitted PPOJax loop, and our task
(initialize at the target pose, hold it) needs far fewer samples than
locomotion. Decision: use the handoff's `jax_rl_mimic` path (PPOJax +
MimicReward) with moderate env counts (~64). Fallback if training stalls:
LocoMuJoCo's `GymnasiumWrapper` + SB3 PPO with ~12 subprocess CPU envs
(~20k steps/s aggregate) — the env/reward/trajectory machinery is shared, only
the PPO harness would change.

Facts confirmed along the way:

- `loco_mujoco_models/unitree_g1/g1_23dof.xml` ships with the package locally
  and contains the full `*_mimic` site set MimicReward/GoalTrajMimic need.
- `MjxUnitreeG1`: dt = 0.01 (timestep 0.002 × n_substeps 5, policy at 100 Hz),
  and `_modify_spec_for_mjx` reduces contacts to **feet↔floor only** (foot
  collision capsules `left/right_foot_1..4_col`). The trained policy never
  feels any other contact — relevant for reference-pose design (nothing else
  may rest on the floor) and for deploy expectations.
- PPOJax steps the env inside `jax.lax.scan` → requires the MJX path;
  a custom trajectory loaded from npz needs `env.th.to_jax()` before training.
- `ImitationFactory` defaults: `TrajInitialStateHandler` (episodes start on
  random trajectory frames — gives us "initialize at the pose" for free) and
  `RootPoseTrajTerminalStateHandler` (terminate on root deviation from the
  reference).
- `ImitationFactory.get_custom_dataset` does NOT auto-extend a qpos/qvel-only
  trajectory with the body/site kinematics the mimic reward needs (that code
  path is commented out upstream), so our generator computes the extension
  itself via per-frame `mj_forward` (mirroring upstream's `ExtendTrajData`).

### 2. Trajectory: synthesized from the sim's NAMED_POSES

`packages/g1-yoga-rl/g1_yoga_rl/yoga_traj.py` +
`scripts/make_yoga_traj.py`:

- Parses `NAMED_POSES` out of `overlays/unitree-g1-mujoco-protocol/python/`
  `g1_protocol_sim.py` via `ast` (no heavy sim imports in the training venv).
- Projects each pose onto the 23-DOF training model (clamped to `jnt_range`;
  missing joints recorded — e.g. wrist pitch/yaw; `waist_pitch` handled
  specially, see below).
- Glide+hold sequence: 1 s settle at standing, then per pose 1.5 s smoothstep
  glide + 3 s hold (same easing as the sim's `animate_to_pose`), 100 Hz.
- Grounding per frame: binary search on root z until foot↔floor contact
  distance ≈ 0 (proper contact-based settle, replacing the sim's cruder
  lowest-geom-center heuristic).
- qvel via central differences; base angular velocity zero (identity/fixed
  base orientation per frame pair assumption).
- FK extension per frame: xpos/xquat/cvel/subtree_com/site_xpos/site_xmat for
  all bodies/sites + full `TrajectoryModel` constants → complete `Trajectory`
  npz that `GoalTrajMimic` can consume.

Verification so far (`scripts/check_imitation_env.py`): obs dim 450, action
dim 23; CPU and MJX (post `to_jax()`) paths both step with finite obs;
zero-action reward near the reference ≈ 1.3–1.6 vs the 1.5 weight-sum maximum
→ reward wiring is correct.

### 3. Pose physicality (user-flagged, in progress)

Rendering the first-cut trajectory exposed that several registry poses are
mannequin poses, not physically holdable configurations:

- **forward_fold** grounded as *sitting on the floor with legs stretched out*
  (pelvis 0.29 m, below the sim's own 0.45 "fallen" threshold). Root cause:
  `hip_pitch 1.4` with an upright pelvis swings the legs forward instead of
  folding the torso; the fold direction (pelvis pitch) is not expressible as
  joint targets at all. Worse, in the training model only feet collide, so a
  seated reference could never be supported.
- **chair** keeps the torso bolt upright with arms overhead — CoM drifts to
  the heels with no forward-lean compensation; matches the measured PD-hold
  topple.
- **warrior_one/two**: with a level pelvis and asymmetric knee bends, the bent
  leg's foot floats mid-air — the "two-foot lunge" is actually a one-leg
  balance as defined.

Agreed fix (option "hybrid"): make the registry poses physically meaningful
and let the projection translate what the 23-DOF model can't express:

- Added a `base_pitch` pseudo-key to the pose registry, honored by the sim's
  `_target_qpos_locked` (pitches the floating base forward) — needed because
  a standing fold requires pelvis pitch.
- Projection translates `waist_pitch` (absent in the 23-DOF model) into
  `base_pitch += waist_pitch` with hip-pitch compensation so the legs keep
  their world orientation and only the torso pitches.
- Retuned chair/fold/warrior entries in `NAMED_POSES`; goddess got ankle
  compensation.

**Sign-convention findings (verified numerically, probe scripts, not yet fully
resolved):** on the 23-DOF model, `hip_pitch`, `knee`, and `ankle_pitch` all
rotate the foot's sole about +y in the *same* direction and add linearly
(foot pitch ≈ hip + knee + ankle). Consequences:

- Flat feet require hip + knee + ankle ≈ 0 (plus base pitch). With
  `ankle_pitch` limited to −0.87, leg configurations with hip+knee beyond
  ~0.87 in the same direction cannot have flat feet — the first-cut "deep
  chair" (hip 0.85, knee 0.95) is geometrically impossible for this robot
  with heels down. Chair must be shallower or heels-up.
- My first hip-compensation sign for the waist translation was inverted
  (caught because grounding found feet *above* the pelvis); fixed.
- Still to resolve: which world direction is the robot's "forward"
  (+x vs −x) — determines the `base_pitch` sign and the anatomical reading of
  hip flexion. Next probe: toe-geom offsets in neutral pose.

Tooling written for this loop (all under `packages/g1-yoga-rl/scripts/`):

- `analyze_pose_stability.py` — per pose: grounded pelvis z, per-foot floor
  clearance and sole tilt, CoM ground-projection margin to the support polygon
  (monotone-chain hull; >0 = statically holdable), plus renders.
- `tune_pose_feet.py` — Newton-solves ankle pitch/roll per planted foot
  (zeroing measured sole tilt) and one reach joint for lunges (zeroing the
  clearance gap between feet), using numeric derivatives so sign conventions
  come from the model, not assumptions. First run: fold solved exactly
  (ankles −0.12); warrior_two close (16° residual roll at the ±0.26
  ankle-roll limit — acceptable); chair/goddess hit the −0.87 ankle-pitch
  limit with ~19–53° residual tilt → their leg geometry must be retuned per
  the flat-feet constraint above.

## Current state of the working tree (uncommitted)

- `.venv-rl/` — Python 3.12 venv with editable loco-mujoco (untracked, not to
  be committed).
- `packages/g1-yoga-rl/` — new training package (traj generator, benchmarks,
  analysis/tuning scripts, `datasets/yoga_full.npz` first-cut trajectory —
  needs regeneration after the pose retune).
- `overlays/unitree-g1-mujoco-protocol/python/g1_protocol_sim.py` —
  `base_pitch` support + retuned chair/fold/warrior/goddess poses (values
  still being iterated; Docker image NOT rebuilt yet).

## Remaining plan

1. **Finish pose retune** (in progress): resolve forward-direction sign, pick
   feasible chair/goddess leg geometry (flat-feet constraint), rerun tuner,
   copy solved values into `NAMED_POSES`, regenerate trajectory, verify with
   the stability table (CoM margin > 0 for two-foot poses) AND rendered
   images of every pose until they look right.
2. **Train** mimic PPO (task 3): adapt `jax_rl_mimic` conf — MjxUnitreeG1,
   `use_mjwarp: false`, wandb disabled, custom dataset, ~64 envs; curriculum:
   stable poses first as sanity, then the full flow. Watch chair/goddess/
   warriors.
3. **Eval + export** (task 4): export policy weights (numpy npz MLP forward
   pass preferred — the Docker sim image has numpy but not JAX) + obs
   normalization stats; document obs layout (450 = plain obs + GoalTrajMimic
   goal: current-state site rpos/rangles/rvel + reference qpos/qvel/site
   quantities) and action scaling.
4. **Deploy** (task 5): `policy` control mode in `g1_protocol_sim.py` running
   the exported MLP at the trajectory cadence over the existing torque path;
   inject the mimic sites into the 29-DOF model at load (MjSpec) so the obs
   builder can read site kinematics; ship the reference trajectory npz into
   the image; wire a `yoga_policy` command; `--policy` mode in
   `examples/yoga_teacher.py`. Known issues to handle: the sim's fall check
   (`torso up-axis < 0.5`) false-positives on the folded pose; obs/action
   ordering must be tested against recorded states from both sides (the
   cross-runtime transfer trap from the research doc).
5. **Validate + commit in slices** (task 6): `--policy` vs the 3/9 PD-hold
   baseline; MCP viewer snapshots per pose; commits: (a) pose retune + sim
   `base_pitch`, (b) g1-yoga-rl package, (c) policy deploy + example, (d)
   docs. Docker rebuild required for (a)/(c) (`docker build` +
   `compose up -d --force-recreate`; plain restart reuses the old image).

## Open risks

- Warrior lunges remain quasi-single-leg if reach equalization can't close the
  clearance gap within joint limits; acceptable fallback: shallower stances.
- Tree is genuinely single-leg — stretch goal, expected to be hard for the
  policy (matches the handoff's honest expectations).
- MJX-on-CPU wall-clock: if the full-flow run crawls, fall back to per-pose
  policies or the SB3 path.
- Isaac-style obs/action mismatch at deploy is the biggest correctness risk;
  mitigated by writing one obs builder and unit-testing it against recorded
  states from both runtimes before wiring the control loop.
