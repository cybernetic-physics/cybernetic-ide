# Lecture 07 — Case Study: Every Concept in This Course, Located in Your Own Repos

*Series: Robot Policies (see [syllabus](robot-policies-00-syllabus.md)). This lecture maps Lectures 1–6 onto the code in `~/wagmi/loco-mujoco` and `~/wagmi/cyber-ide` — file paths verified against the actual checkouts (July 2026).*

The fastest way to make this course concrete: the sibling repos of this document already implement most of Lecture 2's taxonomy. Read them side-by-side with the lectures.

---

## 1. loco-mujoco = Lectures 1–3 in executable form

[LocoMuJoCo](https://github.com/robfiras/loco-mujoco) (checked out at `~/wagmi/loco-mujoco`) is an imitation-learning benchmark for whole-body locomotion — 12 humanoids (incl. **UnitreeG1**), 4 quadrupeds, 22K+ retargeted mocap trajectories.

### The policy object (Lecture 1)

- **Observation space** — `loco_mujoco/environments/humanoids/unitreeG1.py::_get_observation_specification`: the G1's default observation is exactly Lecture 1 §3's proprioception: `FreeJointPosNoXY` (root pose, 5-dim — note x,y excluded: position-invariance), 23 joint positions, `FreeJointVel` (6), 23 joint velocities → 57-dim. No cameras. A locomotion policy in its natural habitat.
- **Action space** — two registered `ControlFunction`s (Lecture 1 §4 made flesh):
  - `core/control_functions/default.py::DefaultControl` — normalize [-1,1] → actuator (torque) range: the *direct-torque exception*.
  - `core/control_functions/pd.py::PDControl` — the *dominant pattern*: policy emits normalized joint-position targets; `generate_action` computes `ctrl = p_gain·(target_q − q) − d_gain·q̇`, and `run_with_simulation_frequency = True` makes the PD math run at the **simulation** rate (1 kHz at default `timestep=0.001`) while the *agent* acts every `n_substeps=10` steps (10 ms → 100 Hz). That is precisely the policy-over-PD frequency hierarchy of Lecture 1 §5, one flag away.
- **The carry as POMDP machinery**: `AdditionalCarry` (flax dataclass in `core/mujoco_base.py`) threads stateful observation/reward/randomizer state through a functionally-pure step — the JAX-native answer to "policies need memory."

### The training substrate (Lectures 2–3)

- **Massively parallel sim RL**: `core/mujoco_mjx.py::Mjx` — vmapped/jitted MJX stepping with in-step asynchronous resets (`jax.lax.cond(state.done, self._mjx_reset_in_step, ...)`), plus optional **MjWarp**. This is the same GPU-parallelism thesis as Isaac Gym's "walking in minutes" **[V]**, in JAX. `MjxUnitreeG1._modify_spec_for_mjx` shows the cost of GPU contact: foot meshes → capsules, all contacts pruned except feet-floor — remember this when Lecture 6 says contact fidelity is the sim-to-real crux.
- **PPO dominance**: `algorithms/ppo_jax.py::PPOJax` — a single-file JAX PPO where env and training compile into one jitted function. Lecture 2 §1.1's consensus, instantiated.
- **Adversarial IL**: `algorithms/gail_jax.py::GAILJax` (discriminator over transitions, policy rewarded for fooling it — Lecture 2 §4's GAIL) and `algorithms/amp_jax.py::AMPJax` — a ~30-line subclass swapping in the least-squares discriminator loss and the `max(0, 1 − 0.25(D−1)²)` reward: **AMP is literally GAIL with a different loss**, and the code makes that lineage visible.
- **DeepMimic tracking**: `core/reward/trajectory_based.py::MimicReward` — exponential kernels `exp(−w·dist²)` over qpos (with proper quaternion angular distance), qvel, and *relative site* positions/orientations/velocities, plus torque/action-rate/out-of-bounds penalty terms. Compare term-for-term with ExBody2/HOVER's tracking objectives **[V]** — same family, and the reward-shaping zoo of Lecture 2 §1.1 is visible in its ~12 weight kwargs.
- **Mocap as pretraining data**: `task_factories/imitation_factory.py` pulls AMASS/LAFAN1/native datasets (HuggingFace-hosted), retargeted per robot; `trajectory/handler.py::TrajectoryHandler` filters/reorders/interpolates trajectories to match the model — the unglamorous plumbing behind "trained on retargeted human MoCap" claims in every humanoid paper.
- **Domain randomization & terrain**: `core/domain_randomizer/` (`DefaultRandomizer`; note `PDControlState.p_gain_noise/pos_offset/ctrl_mult` — gain/offset randomization hooks straight into the PD controller) and `core/terrain/RoughTerrain` — Lecture 2 §6's toolkit.
- **Init/termination**: `TrajInitialStateHandler` = DeepMimic's reference-state initialization; `RootPoseTrajTerminalStateHandler` = early termination on tracking divergence — two under-cited tricks that make mocap-tracking RL converge [K].

### What loco-mujoco does *not* contain (gap-spotting as a learning exercise)

No teacher–student distillation pipeline (Lecture 2 §5 — the sim-to-real workhorse **[V]**), no privileged-observation split, no actuator modeling, no hardware deployment path. It is a *benchmark for policy learning research*, not a deployment stack — which is exactly why its policies stay in MuJoCo. To get an ExBody2-style G1 result you would add: privileged teacher obs → DAgger student on observation histories → export to the real-robot SDK.

---

## 2. cyber-ide = what a robotics stack looks like *below* and *around* the policy

The Cybernetic IDE repo (this repo) contains **no learned policy at all** — and that makes it the perfect foil. Its Dockerized G1 MuJoCo harness (`overlays/unitree-g1-mujoco-protocol/python/g1_protocol_sim.py`) implements the *classical* layer of Lecture 1 §2:

- **A PD + gravity-compensation controller, not a policy**: `apply_hold_control_locked` computes `tau = qfrc_bias + kp·(target − q) − kd·q̇` per actuator — a hand-tuned controller *tracking a given target*. It "decides" nothing; per Lecture 1's distinction, it's a controller. Its documented failure mode ("poses whose center of mass falls outside the support will topple — by design, since there is no whole-body balance controller") is precisely the hole a learned whole-body policy (HOVER/ExBody2-style, or a loco-mujoco-trained tracker) would fill.
- **Kinematic locomotion**: `apply_loco_velocity_locked` teleports the floating base — the simulator honestly documents that real velocity commands would require the balance/locomotion policy it doesn't have.
- **The SDK boundary as the future policy seam**: the `unitree_sdk2py`-shaped facade (`LocoClient.SetVelocity`, `rt/lowcmd` joint targets with kp/kd) mirrors the real G1 interface — which is exactly where a trained policy's 50 Hz joint-target stream would plug in (ExBody2 drives the same 23 joints through the same kind of interface, on the same robot **[V]**).
- The repo's own `todo.md` states the gap in course vocabulary: "Whole-body balance control for low-level commands" — not complete.

**The synthesis opportunity** (and the reason these two repos share a filesystem): train a G1 tracking policy in loco-mujoco (MjxUnitreeG1 + `MimicReward` or AMP + PPO on LAFAN1/AMASS clips), then serve its 50–100 Hz joint-position targets through cyber-ide's `rt/lowcmd` path in place of the static held poses. That single pipe would touch every lecture: mocap data (L2 §4), parallel-sim PPO (L3 §1), PD-target actions (L1 §4), the sim-to-sim gap between MJX's pruned contacts and the harness's full-mesh 29-DOF scene (L6 §2 — note also the 23-DOF vs 29-DOF joint-mapping problem), and eventually a VLA or agent issuing the commands up top (L6 §1). The neighboring docs `g1-yoga-policy-training.md` / `g1-yoga-policy-handoff-prompt.md` in this directory sketch exactly such an effort.

---

## 3. Suggested exercises (PhD-problem-set style)

1. **Feel the action space** (L1): train `MjxUnitreeG1` twice with identical PPO configs — `control_type="DefaultControl"` (torque) vs `"PDControl"` (position targets). Compare sample efficiency, final gait quality, and torque smoothness. Predict the winner from Lecture 1 §4.1 before running.
2. **BC vs. tracking-RL** (L2): take one LAFAN1 walking clip. (a) Behavior-clone qpos targets directly; (b) train `MimicReward` PPO. Measure survival time under a 20 N lateral push. Explain the gap via covariate shift and the embodiment-mismatch argument (L2 §4).
3. **AMP ablation** (L2): diff `gail_jax.py` and `amp_jax.py` (it's ~30 lines). Swap the AMP reward back to the GAIL log-loss reward and observe discriminator saturation effects on gait naturalness.
4. **Contact fidelity audit** (L6): run the same trained policy in MjxUnitreeG1 (capsule feet, pruned contacts) and CPU UnitreeG1 (full meshes). Quantify the sim-to-sim performance drop; relate it to the ManipulationNet claim that contact approximation inflates capability **[S]**.
5. **Close the loop** (L7): stream a loco-mujoco-trained policy's joint targets into cyber-ide's `/command lowcmd` endpoint at 50 Hz and watch Lecture 1 §5's hierarchy — your policy over the harness's PD loop — render live in the Robot Viewer.

---

*End of series. The syllabus's one-paragraph thesis, re-read now, should feel like a summary of things you know rather than a preview of things you don't.*
