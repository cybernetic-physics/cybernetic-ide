# Lecture 03 — Locomotion: The "Solved" Problem That Isn't

*Series: Robot Policies (see [syllabus](robot-policies-00-syllabus.md) for the [V]/[S]/[K] evidence legend)*

People keep telling you locomotion is "more or less solved." This lecture gives you the strongest honest version of that claim, the evidence behind it, and then the expert-sourced case against it. Short version: **the recipe is solved; the problem is not.**

---

## 1. The recipe that changed everything: massively parallel sim RL

Before 2021 [K], learned locomotion meant CPU farms and days-to-weeks of training (ETH's 2019-era Science Robotics results ran on distributed CPU simulation), or careful model-based control (MPC/WBC — the classical Boston Dynamics lineage).

**The verified inflection point** — Rudin et al., CoRL 2021, "Learning to Walk in Minutes" **[V]**:

- **4096 parallel simulated ANYmal robots** in Isaac Gym on a single RTX A6000 (+ i9-11900k), batch size **98,304**, **1500 PPO updates**.
- **Flat-terrain walking in under 4 minutes; rough terrain in ~20 minutes** of wall clock.
- Versus a comparable prior fully-learned perceptive approach needing **120 hours**.
- GPU-simulator throughput: **~0.5–0.7 M state transitions/second** (the survey's "~1M" is a generous rounding; and note it's *Isaac Gym*, Makoviychuk et al. 2021 — the survey misnames it "Isaac Sim") **[V]**.
- Training curriculum: "game-inspired" terrain promotion/demotion based on whether the robot walks off its terrain tile or fails to cover half its commanded distance **[S]**.

Wall-clock minutes ≈ tens of simulated hours. Once policy iteration costs minutes, reward shaping, curricula, and architecture search become interactive — that *engineering-loop compression*, more than any algorithmic insight, is what made the last four years of legged robotics.

The stack that emerged (all of it verified across multiple papers in Lectures 1–2): tiny MLP policy **[V]** → PPO **[V]** → privileged teacher, DAgger student **[V]** → joint-position targets into 0.5–1 kHz PD **[V]** → zero-shot hardware deployment **[V]**.

---

## 2. Quadrupeds: the strongest case for "solved"

### 2.1 Field evidence — ANYmal in the wild **[V]**

Miki et al. 2022 (Science Robotics) is the canonical robustness result:

- 50 Hz policy: leg phase offsets + residual joint-position targets; **recurrent belief-state encoder** fuses proprioception with *noisy* height-map samples, learning when to distrust exteroception **[V]**.
- Teacher trained with privileged contact states/forces/normals, friction, external wrenches; student distilled, deployed **zero-shot, no fine-tuning** **[V]**.
- **The hike:** ANYmal completed a 2.2 km Alpine route (Mount Etzel, Switzerland; 120 m elevation gain), summiting in **31 min vs. the 35 min posted for human hikers**, finishing in 78 min vs. a 76-min planner estimate, **no falls** **[V]**. *Caveats:* single run, and a human supplied directional commands — the policy handled locomotion, not navigation **[V]**.
- Deployment record: at the DARPA Subterranean Challenge, the controller carried four ANYmals over 1,700 m of underground terrain **without a single fall**, staying functional under exteroceptive degradation (reflective floors, snow, transparent obstacles), and walked at 1.2 m/s vs. 0.6 m/s for the blind baseline **[S]**.

### 2.2 Beyond walking [K]

Parkour policies (Zhuang et al. 2023; Cheng et al. 2023 — climbing, leaping, squeezing on cheap quadrupeds), ANYmal parkour (Hoeller et al., Science Robotics 2024), wheeled-legged hybrids, and commercial adoption (Boston Dynamics shipping an RL controller for Spot, 2024; Unitree's product line) all reinforce the maturity story for quadrupeds. *(Background-flagged.)*

---

## 3. Humanoids: the recipe generalizes upward

Humanoid whole-body control (WBC) is where locomotion research now concentrates. Two verified flagships:

### 3.1 ExBody2 (UCSD, arXiv:2412.13196) **[V]**

- Two-stage: **PPO teacher on privileged info → DAgger student** from a longer, non-privileged observation history.
- Deployed on **Unitree G1** (23-dim joint targets, 50 Hz policy, 500 Hz low-level, onboard Jetson Orin NX).
- Small networks: teacher actor MLP [512,256,128]; student MLP [1024,1024,512]; Isaac Gym parallel training **[S]**.
- Trained to *track* human mocap (CMU), with the teacher used to filter dynamically infeasible clips; the student walks, crouches, dances — DeepMimic-style tracking made real **[S]**. Tracking error (mean per-joint, real robot): 0.1074 rad vs. 0.1465 (ExBody) and 0.1396 (OmniH2O*) **[S]**.

### 3.2 HOVER (NVIDIA GEAR/CMU, ICRA 2025) **[V, with flagged caveats]**

The "generalist controller" result:

- One policy **unifies three command families** — kinematic keypoint tracking, local joint-angle tracking, root velocity/height/orientation tracking — via command-space masking (binary mode + sparsity masks; the paper's "one-hot" wording is loose **[V]**), yielding **15+ usable control modes** on a 19-DOF Unitree H1 without per-mode retraining.
- Common abstraction: full-body kinematic motion imitation — an oracle trained on retargeted AMASS mocap, distilled by DAgger.
- Policy: 3-layer MLP [512,256,128] → 19 PD targets **[V]**; real-robot student sees only proprioception + 25-step action history **[S]**.
- Self-reported: beats specialist controllers in ≥7/12 tracking metrics in every mode, and a from-scratch multi-mode RL baseline on 32/32 metrics. ⚠️ **Present with attribution, not as settled fact**: comparisons are in Isaac Gym with author-reimplemented baselines, and an independent re-evaluation (BumbleBee, arXiv:2506.12779) reports much lower HOVER success rates in its own setup (63.21% Isaac Gym / 16.12% MuJoCo vs. 89.58%/66.84%), citing gradient conflicts in full-AMASS multi-mode distillation **[V-caveat]**.

The humanoid pattern to remember: **mocap tracking is the pretraining objective of whole-body control** — human motion data plays the role that internet text plays for LLMs, and the tracking policy is then *commanded* by higher layers (teleop, planners, VLAs — Lecture 6).

---

## 4. So *is* locomotion solved? The audit

### 4.1 The case for "more or less"

Within the settled envelope — velocity-commanded walking over moderate terrain on well-modeled robots — the problem is commodity: minutes of training **[V]**, open-source pipelines (legged_gym/RSL-RL, Isaac Lab, loco-mujoco next door), field-proven robustness **[V]**, products shipping [K]. If your goal is "make a G1 walk," you are executing a known recipe, not doing research. Your acquaintances are *right about this envelope*.

### 4.2 The expert-sourced case against **[V]**

The leading 2024 survey (Ha, Lee, van de Panne, Xie, Yu, Khadiv — arXiv:2406.01152) **explicitly treats legged locomotion as not solved**, devoting its final section to open problems, item-for-item **[V]**:

1. **Unsupervised skill discovery** — beyond reward-engineered gaits.
2. **Differentiable simulators** — gradient-based policy optimization through contact.
3. **Traversing challenging environments** — the contact-rich long tail (loose rock, mud, ice, vegetation, degraded perception).
4. **Safety** — certification and guarantees for learned controllers; RL policies offer no formal envelopes.
5. **Hybrid wheeled-legged locomotion.**
6. **Loco-manipulation** — locomotion *in service of* manipulation: carrying, pushing, opening doors while balancing; the seam between Lectures 3 and 4, largely open.
7. **Foundation models for locomotion** — cross-embodiment generality (the embodiment-scaling study of Lecture 6 distills ~1,000 procedurally-generated embodiments into one zero-shot-transferring policy, and *self-describes its scaling evidence as preliminary* **[S]**).

Fresh corroboration that the basics still yield: ETH, Sept 2025 — a physics-grounded actuator-energy framework cut ANYmal's full Cost of Transport by **32%** (to 1.27) over state-of-the-art methods, while stating plainly that "controllers trained in simulation often fail to transfer reliably" and that most approaches neglect actuator-specific energy losses **[S]**. A 32% efficiency improvement remaining on the table in 2025 is not the signature of a solved field. Sim-to-real robustness, energy, and safety are all live.

### 4.3 The synthesis to carry forward

"Locomotion is solved" is true the way "image classification is solved" was true in 2017: **the benchmark regime is saturated; the deployment regime is not.** What *is* genuinely settled is the *method* — parallel-sim PPO + distillation + PD-target actions is to legged robots what the transformer recipe is to language. The open problems have migrated up the stack (loco-manipulation, foundation policies, safety) and down into the physics long tail. Meanwhile manipulation (next lecture) lacks even the settled recipe — *that* asymmetry, verified on both sides **[V]**, is the real content of your acquaintances' claim.

---

## References (this lecture)

- Rudin et al., *Learning to Walk in Minutes* — arXiv:2109.11978 **[V]**
- Makoviychuk et al., *Isaac Gym* — arXiv:2108.10470 [K]
- Lee et al. 2020 — Science Robotics (blind locomotion, teacher–student) **[V]**
- Miki et al. 2022 — Science Robotics abk2822 (perceptive locomotion; Alpine hike; SubT) **[V]/[S]**
- Ha et al., locomotion survey — arXiv:2406.01152 (Section 8 open problems) **[V]**
- HOVER — arXiv:2410.21229 **[V]**; BumbleBee — arXiv:2506.12779 (independent re-evaluation) **[V-caveat]**
- ExBody2 — arXiv:2412.13196 **[V]/[S]**
- ETH energy-efficient locomotion, Sept 2025 — arXiv:2509.06342 **[S]**
- Embodiment scaling laws — CoRL 2025 (PMLR v305) **[S]**
- [K]: Zhuang/Cheng parkour 2023; Hoeller et al. 2024; Boston Dynamics Spot RL blog 2024
