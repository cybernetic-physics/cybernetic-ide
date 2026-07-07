# Lecture 01 — What Is a Robot Policy?

*Series: Robot Policies (see [syllabus](robot-policies-00-syllabus.md) for the [V]/[S]/[K] evidence legend)*

---

## 1. The definition, stripped bare

A **policy** is a function that maps what the robot senses to what the robot does, executed in a loop:

```
π : observation → action        (deterministic form)
π(a | o)                        (stochastic form: a distribution over actions)
```

That's it. Everything else in this course is detail about (a) what goes into `o`, (b) what comes out as `a`, (c) what function class represents π, and (d) how you obtain π's parameters.

The 2025 imitation-learning survey states it exactly this way: *"A robotic manipulation policy (RMP) specifies how a robot selects and executes actions based on its sensory observations to achieve a manipulation goal"* **[V]**. Diffusion Policy frames policy learning as *"the supervised regression task of learning to map observations to actions"* **[V]** — which should sound instantly familiar: it is sequence modeling, where the sequence is a physical interaction rather than text.

### 1.1 Bridge from your world

You already know this object under other names:

| Your world | Robotics |
|---|---|
| An LLM maps a token context to a next-token distribution | A policy maps an observation (history) to an action distribution |
| Sampling temperature / nucleus | Stochastic policy π(a\|o); exploration noise |
| Autoregressive generation loop | The control loop: observe → act → world changes → observe... |
| A VLA like π0.5 | *Literally a policy.* "Vision-Language-Action model" = a policy whose observation includes images + a language instruction and whose output is robot actions **[V]** |
| Stable Diffusion: noise → image, conditioned on prompt | Diffusion Policy: noise → action sequence, conditioned on observation **[S]** (Lecture 2) |
| E2-TTS flow matching: noise → mel-spectrogram | π0's action expert: noise → 50-step action chunk via flow matching **[V]** |

The key *difference* from LLM generation: the policy's "decoding loop" passes through physics. The environment — not the model — computes the next "context." Errors compound in a way teacher-forced training never sees (this is the *covariate shift* problem of imitation learning, Lecture 2), and the loop must close in milliseconds, not seconds.

### 1.2 The formal wrapper: MDPs and POMDPs [K]

The standard formalism is the **Markov Decision Process (MDP)**: states s, actions a, transition dynamics p(s′|s,a), reward r(s,a), discount γ. A policy π(a|s) is evaluated by its expected discounted return E[Σ γᵗ rₜ]. Reinforcement learning is the search for π maximizing this.

Real robots never see the state. They see *observations* — joint encoders, IMU, cameras — which are partial, noisy, delayed views of s. That makes the problem a **POMDP** (partially observed MDP), and the practical consequence is everywhere in this course: policies condition on *histories* of observations (stacked frames, recurrent networks, transformers over past observations) to implicitly reconstruct the missing state. When you see "the student policy observes a history of onboard measurements" in a sim-to-real paper **[V]**, that's POMDP-coping machinery.

---

## 2. What a policy is *not*: the neighboring species

The robotics stack contains several observation-to-action-ish objects. Confusing them is the #1 newcomer error.

### 2.1 Controller (PD, MPC, WBC)

A **controller** is a hand-designed feedback law with a *given* target.

- **PD controller** [K]: `τ = kp·(q* − q) − kd·q̇` — a proportional-derivative spring-damper that produces motor torque τ to pull joint position q toward target q*. Two gains per joint. No learning, no perception, runs at 500 Hz–1 kHz. This is the workhorse *beneath* almost every learned policy (§4).
- **MPC (Model Predictive Control)** [K]: at every step, solve a short-horizon trajectory optimization problem using an explicit dynamics model, execute the first action, re-solve. Optimization *at inference time*, model-based, no learning required. Boston Dynamics' classical Atlas work is the iconic example.
- **WBC (Whole-Body Control)** [K]: a task-space optimizer (typically a quadratic program solved each control tick) that converts desired body/end-effector motions into joint torques while respecting contact and balance constraints, using the robot's kinematic/dynamic model.

**The distinction:** a controller *tracks* a reference it is given; a policy *decides*. In practice they compose: the policy decides (at 50 Hz) and PD controllers track (at 1 kHz) **[V]**. Note the boundary is a design choice, not a law of nature — an aggressive MPC that replans goals is policy-like, and a learned network trained only to track is controller-like. The field's working convention: *learned + closed-loop + decides ⇒ "policy."*

### 2.2 Planner

A **planner** [K] computes a *sequence* of states or actions ahead of time (A*, RRT for motion planning; task planners; an LLM decomposing "make coffee" into steps). Planners are open-loop until re-planned; policies are closed-loop by construction. Modern stacks put planners *above* policies: the planner emits subgoals, the policy handles the reactive execution (Lecture 6's System 2 / System 1).

### 2.3 World model

A **world model** [K] predicts *what happens next*: p(o′|o,a) or a latent version of it. It is a simulator distilled into a network. Your reference points — GR00T's video-prediction lineage, Cosmos — are world *foundation* models. A world model is not a policy, but it can be used to *get* one, three ways:

1. **Dream training** (Dreamer family): train the policy by RL inside the learned model's imagination (Lecture 2).
2. **Planning** (TD-MPC2, MPC-style): search action sequences against the model at inference time.
3. **Data generation**: use it to synthesize training trajectories (GR00T N1's "neural trajectories," Lecture 4 **[S]**).

Mnemonic: **world model = physics, reward model = desire, policy = behavior.** The policy is the only one that has to run in the real-time loop.

---

## 3. Observation spaces: what π gets to see

A typical modern robot policy consumes some subset of:

- **Proprioception** [K]: joint positions q, joint velocities q̇, base angular velocity and gravity direction from the IMU, previous action(s). Cheap (kHz-rate), low-dimensional (tens of floats), and the *sole* input to most deployed locomotion policies ("blind" locomotion). HOVER's real-robot student observes exactly this: joint pos/vel, base angular velocity, gravity vector, and an action history stacked over 25 steps **[S]**.
- **Exteroception** [K]: vision (RGB/depth cameras), LiDAR, elevation maps. In locomotion, "perceptive" policies fuse proprioception with terrain height samples (Miki et al. use a recurrent belief encoder over proprioception + *noisy* height samples, so the policy learns when to distrust its eyes **[V]**). In manipulation, vision is nearly always required — the object's pose is not in your joint encoders.
- **Language**: an instruction string. This is precisely what makes a VLA a VLA — the policy is conditioned on language the way Stable Diffusion is conditioned on a prompt **[V]**.
- **Tactile** [K]: fingertip force/vision-based touch sensors (GelSight-style optical tactile skins, Meta's Digit). Rich literature, but as of 2025 tactile input remains rare in flagship generalist policies — a known gap (Lecture 4).
- **Privileged observations (sim-only)**: ground-truth contact forces, friction coefficients, exact terrain geometry — visible in the simulator but not on hardware. Used by *teacher* policies that are then distilled into deployable *students* (Lecture 2; verified pipeline in Lee 2020, Miki 2022, HOVER, ExBody2 **[V]**).

Design principle [K]: every added observation is a sim-to-real liability (it must be simulated faithfully *and* measured reliably on hardware). The field's habit of proprioception-first minimalism in locomotion is a robustness decision, not a compute one.

---

## 4. Action spaces: what π gets to say

This is the most robotics-specific design axis, and the research verified a clear picture:

### 4.1 The dominant pattern: joint-position targets over a PD loop

**The policy outputs desired joint positions q\*; a PD controller turns them into torques.** The locomotion survey: *"Most works in quadrupedal learning use joint target position as the action space (PD policy)"* **[V]**. Concrete verified instances **[V]**:

- Rudin et al. 2022: "The actions are interpreted as desired joint positions sent to the motors. There, a PD controller produces motor torques." Policy at 50 Hz.
- Miki et al. 2022: 50 Hz policy emitting leg phase offsets + residual joint-position targets (via leg IK).
- ExBody2: 50 Hz policy, 23-dim joint-position targets, 500 Hz low-level interface, deployed on a Unitree G1 with an onboard Jetson Orin NX.
- HOVER: 19-dim joint-position targets into PD, on a Unitree H1.

Why this works [K]: the PD loop acts as a *learned-policy-friendly abstraction* — it is an impedance (spring-damper) around the target, so the policy's outputs are smooth, bounded, and forgiving; the high-rate stabilization burden is delegated to a 1 kHz loop the network never has to model. The action space itself embeds a prior.

**Direct-torque policies exist but self-identify as the exception** (Chen et al. 2023; Kim et al. 2023) **[V]**. The survey's argument is that PD-target policies can run slowly (~50 Hz) while torque is still produced at 1 kHz — whereas torque policies push the *policy itself* toward higher evaluation rates. (Note: the stronger claim "torque policies must run at 1 kHz" was refuted in verification — treat rate requirements as an empirical, per-paper question.)

### 4.2 The manipulation menu [K, structure; V where tagged]

- **End-effector deltas**: Δ(x, y, z, roll, pitch, yaw) + gripper open/close, with an inverse-kinematics layer mapping to joints. The lingua franca of tabletop manipulation datasets (Open X-Embodiment).
- **Absolute joint positions**: ACT predicts absolute joint targets for both ALOHA arms.
- **Discretized action tokens**: bin each action dimension into 256 buckets and emit them as *tokens* — action generation becomes next-token classification on a standard transformer stack. This is the RT-2/OpenVLA design: OpenVLA maps 256 bins onto the 256 least-used Llama-2 tokens and trains with cross-entropy **[V]**. Known costs: quantization error and slow autoregressive decoding at high control rates **[V]** (the FAST paper's motivation).
- **Continuous action chunks via generative heads**: a diffusion or flow-matching module generates an entire *chunk* of future actions conditioned on the observation embedding. π0: flow-matching action expert, chunks of H=50 actions, up to 50 Hz control **[V]**.

### 4.3 Action chunking — the concept your sequence-model intuition needs

**Action chunking** = predict a block of k future actions at once, execute some prefix, re-observe, repeat. ACT introduced this framing for imitation: it "packs high-frequency, low-level controls into discrete 'action chunks'," shortening the effective decision horizon k-fold and improving sample efficiency in low-demo regimes **[V]**.

The Dec-2025 VLA survey gives the exact formulation to remember: chunking is *"a practical compromise: the policy operates autoregressively over coarse time (emitting chunks), but non-autoregressively within each chunk"* **[V]** — the same pattern as ACT, Diffusion Policy, and π0/π0.5. Diffusion Policy pairs this with **receding-horizon control**: predict a horizon, execute a short prefix, re-plan **[S]**.

Why chunk? [K] (a) compounding-error mitigation — fewer decision points per episode; (b) temporal consistency — one coherent sample instead of k independently-noised decisions (compare: generating a whole image at once vs. pixel-by-pixel); (c) inference amortization — one billion-parameter forward pass buys k control steps. The cost: within-chunk open-loop-ness reduces reactivity in stochastic environments **[S]** (Bidirectional Decoding, arXiv:2408.17355).

---

## 5. The control-frequency hierarchy

Assemble §3 and §4 into the canonical deployed stack:

```
  ~0.1–10 Hz   Planner / VLM / "System 2"     (subgoals, language, task logic)
  ~10–50 Hz    POLICY (the learned π)          (obs → action / action chunk)
  ~500–1000 Hz PD / whole-body controller      (target → motor torque)
  ~10–40 kHz   Motor drivers / current control (electrical commutation)
```

Verified anchors: 50 Hz policy over 500 Hz low-level on the G1 (ExBody2) **[V]**; 50 Hz perceptive policy on ANYmal (Miki) **[V]**; GR00T N1's System 2 VLM at 10 Hz over a System 1 diffusion-transformer action module at 120 Hz **[S]**; π0 at up to 50 Hz **[V]**.

The general lesson, which you can port straight from systems thinking: **each layer is a rate adapter and an abstraction barrier.** The policy is the layer where *learning* currently pays off most — below it, physics is too fast and too safety-critical; above it, semantics were (until VLAs) too hard.

---

## 6. Where policies live in your generative-AI mental model — a summary table

| Question | Locomotion answer (typical) | Manipulation answer (typical, 2025) |
|---|---|---|
| Function class | 2–3 layer MLP (~10⁵–10⁶ params), sometimes GRU/small transformer **[V]** | Billion-param VLA (π0: 3.3B; OpenVLA: 7B; GR00T-N1: 2.2B) **[V]/[S]** |
| Observation | Proprioception (+ optional height map) **[V]** | Multi-camera RGB + proprioception + language **[V]** |
| Action | Joint-position targets → PD **[V]** | EE deltas / joint targets, as tokens or generative chunks **[V]** |
| Trained by | RL in massively parallel sim **[V]** | Imitation of teleoperated demos **[V]** |
| Rate | ~50 Hz over 0.5–1 kHz PD **[V]** | ~5–50 Hz chunked **[V]** |
| Status | Field-robust, not "solved" **[V]** | The open frontier; generalization unresolved **[V]** |

Hold this table in mind; Lectures 3 and 4 are essentially deep dives into its two columns, and Lecture 2 explains the training-paradigm row.

---

## References (this lecture)

- IL-manipulation survey — arXiv:2508.17449 (policy definition; ACT chunking)
- Legged locomotion survey — arXiv:2406.01152 (action spaces; PPO; teacher–student)
- Rudin et al. — arXiv:2109.11978 (50 Hz PD-target policy)
- Miki et al. — Science Robotics abk2822 (perceptive locomotion, belief encoder)
- ExBody2 — arXiv:2412.13196 (G1 deployment stack)
- HOVER — arXiv:2410.21229 (MLP size, observation stack)
- VLA anatomy survey — arXiv:2512.11362 (action tokenization; chunking formulation)
- OpenVLA — arXiv:2406.09246 (256-bin tokenization)
- π0 — arXiv:2410.24164 (flow-matching action expert, 50 Hz)
- Diffusion Policy — arXiv:2303.04137 (receding-horizon; "supervised regression" framing)
- GR00T N1 — arXiv:2503.14734 (10 Hz / 120 Hz dual rates)
- Sutton & Barto, *Reinforcement Learning: An Introduction* [K] (MDP/POMDP formalism)
