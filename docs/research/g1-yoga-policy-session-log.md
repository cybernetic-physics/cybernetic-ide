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

**Sign conventions (RESOLVED, verified numerically against both MJCFs):**
forward is +x (toe geoms at +0.12 from the ankle); `hip_pitch`, `knee`, and
`ankle_pitch` all rotate about +y and add linearly, with POSITIVE values
swinging the segment BACKWARD. Hip flexion (thigh forward) is therefore
negative `hip_pitch`, and a flat foot requires
base_pitch + hip + knee + ankle ≈ 0. The original registry used positive hips
for forward bends — the legs swung backward instead, one reason those poses
toppled. The earlier "impossible chair" conclusion was an artifact of
same-sign hips: with hip −0.75 / knee +0.85 the terms cancel and chair is
easy. `base_pitch` quat [cos θ/2, 0, sin θ/2, 0] pitches the torso forward ✓.

**Final retuned registry (all margins from analyze_pose_stability, 23-DOF
projected + grounded):** mountain +6.5 cm, upward_salute +6.0, forward_fold
+3.8 (base 0.35 + waist 0.52 ≈ 50° fold, feet ~14 cm ahead of hips, stays
under the torso-up fall threshold), chair +2.4 (waist 0.5 forward lean, CoM
moved off the heel edge), warrior_one +4.5 (true lunge, both feet planted,
rear knee/ankle solved by the Newton tuner), warrior_two +6.3, goddess +1.1,
namaste +7.2; tree remains single-support by design (CoM 11 cm off the stance
foot — the policy must deviate to balance; stretch goal). Rendered frames of
every pose visually verified (proper standing fold, chair with hips back +
torso lean, planted lunges). Kinematic application in the 29-DOF sim verified
in-process (base_pitch quat applied, no false "fallen").

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

### 4. Training runs

- **run1** (20M steps, lr 3e-4, single monolithic train_fn): learned to
  return ~114 / episode length ~220 by update ~1250, then **collapsed
  catastrophically** (~update 1700, return ~10) and never recovered; the
  intermediate state was unrecoverable because PPOJax only returns the final
  agent. End-to-end throughput 3,768 steps/s (88.5 min).
- Trainer rewritten to **chunked training**: one jitted resume function, N
  chunks (default 10), agent saved after every chunk, best tail-return kept —
  a late collapse can no longer destroy the run. lr default dropped to 1e-4.
- **run2** (20M steps, 10×2M chunks, lr 1e-4): chunk 1 tail return 96.5,
  chunk 2 tail return 336 / length 423. Interim eval of the chunk-2 agent
  through the FULL deploy pipeline: **4/9 poses held** (mountain,
  upward_salute, warrior_one, goddess) — already beats the 3/9 PD baseline
  and rescues two previous topplers, at 4M steps.

### 5. Export / deploy pipeline (built and validated with interim agents)

- `g1-yoga-export`: PPOJax pickle → numpy npz (actor MLP 450→512→256→23 tanh,
  frozen RunningMeanStd, DefaultControl act_mean/act_delta over ctrlrange) +
  parity test vs the flax network (3.7e-07; the flax RunningMeanStd
  normalizes with batch-updated stats, so the test emulates that update).
- `g1-yoga-pack`: deploy bundle npz — policy + **precomputed reference half
  of the goal obs** (225 dims/frame, computed with loco_mujoco's own
  functions) + raw ref qpos/qvel for teleport resets + mimic-site table +
  actuator→joint mapping (mapped by driven joint name across models).
- `overlays/.../python/g1_policy_runtime.py`: sim-side runtime — injects the
  15 mimic sites into the 29-DOF MjSpec (hand sites remapped from the 23-DOF
  lumped wrist body to `*_wrist_yaw_link` −0.084 x), numpy obs builder
  (plain 57 + current-site 168 via loco-mujoco-mirroring math with mujoco's
  own quat helpers) + MLP + actuator mapping + gravity-comp PD for the six
  29-DOF-only actuators + teleport reset.
- `g1-yoga-validate-deploy`: obs parity 29-DOF-deploy vs training truth at
  sampled frames: **plain 0.0, current-site ≤4e-16, reference-goal ≤7e-8** —
  machine precision. (Found along the way: loco_mujoco's CPU `reset()`
  computes the obs before `mj_forward` sees the trajectory state, so
  reset-obs site kinematics are stale upstream; validation compares against
  freshly-forwarded `_create_observation` instead.)
- `g1-yoga-sim2sim`: closed-loop policy rollout on the 29-DOF scene with the
  exact deploy control loop (policy 100 Hz over 0.002 s physics). Smoke-agent
  survival ticks match the training env (34–79 vs 34–87) → the sim2sim gap is
  small.

### 6. Sim policy mode (implemented, locally verified end-to-end)

- `g1_protocol_sim.py`: loads the bundle if present (default
  `/opt/unitree-g1-mujoco-protocol/policy/g1_yoga_policy.npz`, override via
  `UNITREE_G1_POLICY_BUNDLE`), injects mimic sites at model load; new
  `yoga_policy` command (`start`/`stop`/`status`) drives control_mode
  "policy": physics timestep switches to 0.002, a new action every 5 steps,
  loop-wrapping reference frames, fall (< 0.35 m pelvis) teleports back onto
  the reference and counts `falls`; `/status` reports frame / cycles / falls
  / pose label.
- **Physics pump decoupled from rendering**: the old render loop advanced
  physics at 8 Hz with a 0.08 s cap (≈64% real time at best). New dedicated
  200 Hz physics thread; render loop only refreshes JPEGs; `UNITREE_G1_RENDER_HZ=0`
  disables rendering (macOS offscreen GL wedges in background threads — a
  local-testing artifact only, Docker/osmesa unaffected). HTTP/WS ports now
  env-configurable (`UNITREE_G1_HTTP_PORT`/`UNITREE_G1_WS_PORT`).
- Local end-to-end run (host python, render off): `yoga_policy start` →
  frames advance ~83/s wall, falls counted + auto-recovery works with the
  weak smoke policy.
- `examples/yoga_teacher.py --policy`: starts the loop, narrates pose-by-pose
  from `/status` policy frames, snapshots each hold, tallies falls per pose
  window.

### 7. run2 final + the sim2sim transfer failure (the day's big lesson)

run2 finished cleanly (10 chunks, monotonic improvement, final tail return 729
/ episode length 767 of 1000). In the training env the exported policy held
**6/9 poses** (mountain, salute, fold, chair, warrior_one, goddess) — double
the PD baseline. But in the 29-DOF deploy sim it scored **0/9**, despite obs
matched to machine precision.

Diagnosis: pure dynamics gap. Foot geometry is actually identical (both
models use 4 contact spheres per foot, same friction/solref), and matching
the MJX solver settings (iterations 2/4, EULERDAMP off vs 100/50) helped only
marginally. The residual is structural — the 23-DOF model welds the wrist
chain and lumps its mass, waist roll/pitch are absent vs PD-held, torso mass
differs 0.24 kg, total mass 34.1 vs 35.1 kg. The weak smoke policy transferred
fine (survival ticks matched 34–79 vs 34–87), but the trained policy
overfits precise closed-loop dynamics — a ~3% model mismatch was enough to
destroy it. Lesson recorded: **smoke-policy sim2sim parity does NOT certify
transfer; only a trained policy exposes the gap.**

### 8. Pivot: train on the deploy model itself (29-DOF spec)

Implemented the handoff's alternative path (`spec=` argument):
`g1_yoga_rl/cyber_env.py` builds the training env from OUR
`scene_29dof.xml` — mimic sites injected (site table lifted from the 23-DOF
model, hand sites remapped), foot contact spheres named, free joint renamed
"root", obs/actuation specs generated from the model (obs 474, action 29);
the env's own `_modify_spec_for_mjx` applies the same foot-only contact
reduction used before. The whole pipeline (trajectory regen, export, pack
with dynamic goal dims, validate, sim2sim) was re-pointed and re-verified:

- obs parity 29-DOF deploy vs training truth: still machine precision
- **transfer parity gate with a smoke policy: in-env eval vs sim2sim now
  agree within 2–3 ticks on every pose** — training and deploy dynamics are
  the same by construction
- sim policy mode now applies `reduce_robot_contacts` at model load (bundle
  present) and toggles the training solver options around policy mode

run3 (20M steps, 10 chunks, lr 1e-4, 29-DOF) is training now.

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
