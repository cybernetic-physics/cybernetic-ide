# Lecture 02 — The Taxonomy: How Robot Policies Are Trained

*Series: Robot Policies (see [syllabus](robot-policies-00-syllabus.md) for the [V]/[S]/[K] evidence legend)*

The function π(a|o) can be represented by anything differentiable. What actually distinguishes the families you'll meet in papers is **where the training signal comes from**. There are exactly three primal sources — *reward* (RL), *demonstrations* (imitation), and *predictions* (world models) — and everything in the ecosystem is one of these or a hybrid.

---

## 1. Model-free reinforcement learning: policies from reward

**Idea** [K]: roll the policy in an environment, score trajectories with a reward function, ascend the expected return. No dynamics model is learned; the simulator *is* the model.

### 1.1 PPO and why it owns locomotion

**PPO (Proximal Policy Optimization)** [K, mechanics]: an on-policy actor-critic method that takes conservative policy-gradient steps by clipping the importance ratio between new and old policy. Simple, stable, embarrassingly parallel.

The verified field consensus: *"PPO has been a particularly popular choice in the legged locomotion community due to its excellent convergence and adaptability to diverse policy architectures"* — it is the most popular deep-RL algorithm for legged locomotion and the default in Isaac Lab / RSL-RL pipelines **[V]**. Even pro-SAC sources treat PPO as the de-facto standard **[V]**.

Why on-policy wins *in simulation* [K]: PPO's weakness is sample hunger — but massively parallel GPU simulation made samples nearly free. Rudin et al.: 4096 simulated ANYmals, batch size 98,304, 1500 PPO updates → flat-terrain walking in **under 4 minutes**, rough terrain in ~20 minutes, on one RTX A6000 **[V]**. GPU simulators (Isaac Gym — note: the survey misnames it "Isaac Sim") collect roughly **0.5–0.7 M state transitions per second** **[V]**. Twenty wall-clock minutes ≈ ~80 hours of simulated robot experience. Off-policy methods (SAC) appear where sample efficiency is the true bottleneck, i.e., learning directly on hardware **[V]**.

Reward design is the real art [K]: locomotion rewards are sums of ~10–20 shaped terms (velocity tracking, torque penalties, action-rate penalties, foot air-time, orientation penalties...). This fragility is exactly what motivates the imitation-flavored methods in §3–4. Curricula matter too — Rudin et al.'s "game-inspired" terrain curriculum promotes a robot to harder terrain when it walks off its tile and demotes it when it covers less than half its commanded distance **[S]**.

### 1.2 Where model-free RL fails

RL needs (a) a resettable, fast environment and (b) a specifiable reward. Locomotion has both. Tabletop manipulation usually has neither at scale — contact simulation is unreliable (Lecture 6 **[S]**), resets are physical labor, and "fold the shirt nicely" defies reward engineering. Hence the imitation-learning takeover of manipulation (§3).

---

## 2. Model-based RL: policies from learned dynamics [K]

*(Flagged: this section is background — the deep-research verification pass returned no surviving claims here; numbers below are from the primary papers as retained knowledge.)*

- **Dreamer family (Hafner et al.)**: learn a latent world model (RSSM), then train actor and critic entirely inside its "imagination" — rollouts of the latent model. DreamerV3 (2023) solved diverse benchmark domains with one configuration; DayDreamer (2022) trained a physical quadruped to walk in about an hour *on hardware, without a simulator* — the canonical demonstration that world models buy real-robot sample efficiency.
- **TD-MPC2 (Hansen et al., 2023)**: hybrid — learn a latent dynamics model, plan at inference time with short-horizon sampling (MPPI) guided by a learned value function; one set of hyperparameters across 100+ continuous-control tasks.
- Relationship to your world: this is the "policies from world models" cell — the robotics-native ancestor of the Cosmos/GR00T thesis that predictive models of the world are a substrate for action. In deformables, the model-based pattern re-emerges strongly (Lecture 5: GNN dynamics + planning **[S]**).

Why it hasn't displaced PPO-in-sim for locomotion [K]: when a GPU simulator gives you 700K real-physics steps per second, a learned approximation of physics has to justify its bias. Model-based methods shine when the environment is the real world (slow, unresettable) or when the dynamics must be learned anyway (deformables).

---

## 3. Imitation learning: policies from demonstrations

**Behavior cloning (BC)** [K]: collect (o, a) pairs from an expert (usually human teleoperation), fit π by supervised learning. It inherits everything you know from supervised generative modeling — and one problem you don't have: **covariate shift**. The policy's own small errors carry it into states the demonstrator never visited, where it errs worse (compounding). DAgger [K] (Ross et al., 2011) fixes this by iteratively querying the expert *on the learner's own states* — impractical with humans, but perfect when the "expert" is another neural network (see §5, distillation).

The last three years of manipulation are, at core, three escalating answers to "how do we make BC actually work?":

### 3.1 ACT (2023): chunking + a CVAE transformer

Zhao et al.'s **Action Chunking with Transformers**, built for the $20K-class ALOHA bimanual teleoperation rig [K, hardware], predicts *chunks* of ~100 future joint-position targets (50 Hz control) in one DETR-style forward pass, trained as a conditional VAE. Chunking shortens the effective horizon k-fold, mitigating compounding errors, and was critical in ablations; ~50 demonstrations sufficed for 80–90% success on fine bimanual tasks **[V]**.

### 3.2 Diffusion Policy (2023): Stable Diffusion for actions — stated carefully

Chi et al. represent *"a robot's visuomotor policy as a conditional denoising diffusion process"* **[S, verbatim]**: instead of regressing one action, start from Gaussian noise over an action *sequence* and iteratively denoise it, conditioned on visual observations — mechanically the same conditional denoising you know from image diffusion, applied to a (horizon × action-dim) array. Inference "learns the gradient of the action-distribution score function and iteratively optimizes via stochastic Langevin dynamics steps" **[S]**.

⚠️ *Refuted framing to avoid:* "Diffusion Policy pioneered denoising for control" failed verification 0-3 (diffusion for decision-making predates it — e.g., Janner et al.'s Diffuser [K]). Its verified contribution is making diffusion work *as a real-time robot policy*: receding-horizon control, visual conditioning, and a time-series diffusion transformer **[S]**.

Why diffusion at all? The authors' three reasons: graceful handling of **multimodal action distributions** (a demonstrator sometimes goes left, sometimes right — a mean-regression policy splits the difference and hits the obstacle), suitability for **high-dimensional action spaces** (chunks), and **training stability** **[S]**. Result: +46.9% average relative improvement across 12 tasks / 4 benchmarks over prior methods **[S]**.

### 3.3 Flow matching & the VLA era (2024–): π0 and kin

Exactly as E2-TTS replaced diffusion's many-step sampling with flow matching's few-step ODE integration, π0 attaches a **flow-matching "action expert"** (~300M params, its own weights — *not* just a head) to a pretrained 3B PaliGemma VLM, 3.3B total, generating 50-step continuous action chunks at up to 50 Hz **[V]**. The survey's stated rationale for flow over diffusion: high-quality outputs in fewer inference steps **[S]**. The alternative VLA design tokenizes actions into 256 bins and does next-token classification (RT-2, OpenVLA) **[V]** — Lecture 4 treats both in depth.

The punchline for your mental model: **a VLA is behavior cloning at foundation-model scale** — pretrained VLM backbone, robot-demonstration corpus, generative action decoder. The paradigm is imitation; the novelty is scale and the language interface.

---

## 4. Adversarial imitation & motion tracking: reward *from* demonstrations

Between "reward engineering" (RL) and "supervised copying" (BC) sits a family that converts demonstrations into a reward and then runs RL. This family dominates *character-like whole-body motion* — and it is exactly what's implemented in the loco-mujoco repo sitting next to this one (Lecture 7).

- **GAIL** [K] (Ho & Ermon, 2016): a GAN over state transitions. Discriminator D learns to tell policy transitions from expert transitions; the policy is RL-trained with reward = fooling D. No reward engineering; no action labels needed beyond states.
- **AMP** [K] (Peng et al., 2021): *Adversarial Motion Priors* — GAIL's discriminator repurposed as a *style* reward, added to a simple task reward (e.g., "reach this velocity"). The policy accomplishes the task *in the style of* the mocap corpus. This is the standard recipe for natural-looking humanoid/quadruped motion; loco-mujoco's `AMPJax` implements it as a least-squares discriminator variant of its GAIL trainer.
- **DeepMimic** [K] (Peng et al., 2018): no discriminator — a *tracking* reward directly measures pose/velocity/end-effector distance to a retargeted reference motion, with exponential kernels (exp(−w·error²)), plus reference state initialization. This is the workhorse of humanoid whole-body control: HOVER's oracle is a motion imitator on retargeted AMASS mocap **[V]**; ExBody2 tracks CMU mocap with a teacher-filtered curriculum **[S]**; loco-mujoco's `MimicReward` is a faithful implementation (qpos/qvel/site-position exponential terms + torque/action-rate penalties).
- **ASAP** [K] (2025, NVIDIA/CMU): the real2sim2real refinement — learn a *delta action* model from real-robot rollouts that corrects the simulator's dynamics mismatch, then fine-tune the tracking policy inside the corrected sim; used for agile G1 whole-body motions. (Background-flagged; not in the verified set.)

Why adversarial/tracking beats plain BC for locomotion [K]: demonstrations (human mocap) come from a *different embodiment* — you cannot clone actions that were never recorded (mocap has no torques), so you must let RL discover actions whose *outcomes* match the reference states. That is precisely what tracking rewards and discriminators do.

---

## 5. Teacher–student distillation: the sim-to-real workhorse

**The verified standard recipe** for getting RL policies onto real legged robots **[V]**:

1. **Teacher**: train with RL in simulation with **privileged observations** — noiseless states, exact terrain, contact states/forces/normals, friction, external wrenches — things only a simulator can expose.
2. **Student**: distill the teacher into a policy that consumes only *deployable* observations (proprioception histories, noisy height samples), typically with **DAgger** — roll out the *student*, label its states with the teacher's actions, supervised-fit (an L2/action-matching loss).
3. **Deploy zero-shot** on hardware.

Verified instances **[V]**: Lee et al. 2020 (ANYmal blind locomotion — proprioceptive student from onboard-measurement history); Miki et al. 2022 (recurrent belief-state encoder fusing proprioception + noisy height samples; zero-shot transfer, no fine-tuning); HOVER (oracle imitator on AMASS → DAgger distillation into one multi-mode policy); ExBody2 (PPO teacher on privileged info → DAgger student from longer non-privileged history, deployed on Unitree G1).

Why it works [K]: the privileged teacher solves an easy, fully-observed RL problem; the student solves an easy supervised problem (with DAgger killing covariate shift, since the teacher-expert is queryable everywhere). The POMDP never has to be solved directly by RL. Distillation also *merges*: many specialists → one generalist (HOVER distills modes **[V]**; the embodiment-scaling study distills ~1,000 per-embodiment experts into one cross-embodiment policy **[S]**).

## 6. Sim-to-real transfer & domain randomization

- **Domain randomization** [K] (Tobin et al. 2017; OpenAI's Dactyl/Rubik's-cube automatic domain randomization): randomize physics (masses, friction, latencies, motor strengths) and visuals during training so the real world looks like just another sample from the training distribution.
- **Actuator modeling** [K]: ETH's actuator networks — learn the motor's real torque response from data and *put it in the simulator*.
- Verified status: sim-to-real is *standard working practice* for locomotion ("it is common practice to train the policies in simulation and then transfer them to the real world") **[V]** — yet for VLA manipulation it "remains a core obstacle," with dynamics discrepancies (friction, latency, actuation response) and perception discrepancies (illumination, textures, sensor noise) severely degrading transfer; randomization and fidelity enhancement are mitigations, not solutions **[V]**.
- Counterpoint from late 2025 **[S]**: ETH (Hutter's group) demonstrated reliable transfer across 13 legged platforms *without* randomizing dynamic parameters — using systematic dynamic-parameter identification and a physics-grounded actuator (PMSM) energy model instead, while noting that "controllers trained in simulation often fail to transfer reliably." Randomization is the default, not a law.

---

## 7. The size spectrum — one taxonomy-wide observation

**[V]** State-of-the-art humanoid whole-body policies are *tiny*: HOVER is a 3-layer MLP [512, 256, 128] emitting 19-dim PD targets; corroborating SONIC: "state-of-the-art humanoid control policies are often small neural networks, e.g., three-layer MLPs." Miki's field-proven ANYmal student: a 2×50-unit GRU + a {256,160,128} MLP **[S]**. Meanwhile π0 is 3.3B, OpenVLA 7B, GR00T-N1 2.2B **[V]/[S]** — roughly **five orders of magnitude** apart. (Caveat: some 2025 humanoid trackers use small transformers — still millions, not billions, of parameters **[V]**.)

The interpretation to internalize: **network size tracks the semantic bandwidth of the observation-action mapping, not the physical difficulty of the task.** Balancing 51 kg of humanoid is stupendously hard *physics* but a low-dimensional, fast *mapping* (proprioception → 19 joint targets); "clean the kitchen" is easy physics and an open-world *mapping* (pixels + language → any of a thousand behaviors). RL-in-sim compresses physics into small networks; open-world semantics demand pretrained internet-scale backbones.

---

## 8. Decision chart (which paradigm when) [K]

```
Reward specifiable + fast simulator?            → model-free RL (PPO) in parallel sim
  ...and deploy on hardware?                    → + teacher-student + DR / system-ID
Demonstrations exist, same embodiment?          → BC with chunking (ACT / Diffusion Policy / VLA fine-tune)
Demonstrations exist, different embodiment
  (human mocap → robot)?                        → tracking reward (DeepMimic) or AMP/GAIL + RL
No simulator, no demos, real robot only?        → model-based RL (Dreamer-style) or offline RL
Open-world language-conditioned generality?     → VLA (pretrained VLM + generative action decoder)
```

---

## References (this lecture)

- Locomotion survey — arXiv:2406.01152 **[V]** (PPO consensus, teacher–student, DR practice)
- Rudin et al. — arXiv:2109.11978 **[V]** (walking in minutes; curriculum)
- Lee et al. 2020, Science Robotics — **[V]** (privileged teacher–student)
- Miki et al. 2022, Science Robotics abk2822 — **[V]** (belief encoder, zero-shot transfer)
- HOVER — arXiv:2410.21229 **[V]**; ExBody2 — arXiv:2412.13196 **[V]**
- ACT — arXiv:2304.13705 **[V]**; Diffusion Policy — arXiv:2303.04137 **[S]**
- π0 — arXiv:2410.24164 **[V]**; VLA anatomy survey — arXiv:2512.11362 **[V]**
- IL-manipulation survey — arXiv:2508.17449 **[V]** (flow-vs-diffusion rationale **[S]**)
- ETH energy-efficient sim-to-real — arXiv (Sept 2025) **[S]** (32% CoT reduction, no-DR transfer)
- Embodiment scaling — CoRL 2025, PMLR v305 **[S]**
- [K] anchors: Ho & Ermon 2016 (GAIL); Peng et al. 2018 (DeepMimic), 2021 (AMP); Ross et al. 2011 (DAgger); Hafner et al. (Dreamer/DayDreamer); Hansen et al. (TD-MPC2); Tobin et al. 2017 & OpenAI 2019 (domain randomization); He et al. 2025 (ASAP)
