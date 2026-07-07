# Lecture 05 — Deformable & Soft-Body Manipulation

*Series: Robot Policies (see [syllabus](robot-policies-00-syllabus.md) for the [V]/[S]/[K] evidence legend)*

*Sourcing note: the adversarial-verification budget in the research pass did not reach this lecture's claims, so nothing here carries a **[V]** tag — but every **[S]** claim below was extracted from a primary paper with a verbatim quote. Treat confidence as one notch below Lectures 1–4.*

---

## 0. Two meanings of "soft," disambiguated first

- **Soft objects as manipulation *targets*** (this lecture's core): cloth, rope/cables, food, tissue — a rigid robot manipulating deformable stuff.
- **Soft robots as *actuators*** (§6): grippers and bodies made of compliant material — deformable stuff doing the manipulating.

They share continuum mechanics but are different research communities. When someone says "soft-body manipulation is unsolved," they nearly always mean the first.

---

## 1. Why deformables break everything you learned in Lecture 4

Rigid manipulation is hard; deformable manipulation removes the one mercy rigid objects grant you — a finite state.

1. **Infinite-dimensional state.** "Unlike rigid objects whose poses can be fully represented by low-dimensional vectors [6-DoF], deformable objects have an infinite configuration space that is prone to severe self-occlusion" **[S]** (deformables survey, arXiv:2312.10419). A shirt's "pose" is a continuum surface; a folded shirt hides most of itself from every camera.
2. **Perception, modeling, and control fail *simultaneously*.** "Deformable objects exhibit infinite dimensionality, dynamic shape changes, and complex interactions with their environment, posing significant hurdles for perception, modeling, and control" **[S]** (2026 DOM survey, arXiv:2602.22998). In rigid manipulation you can often solve perception (pose estimation) and control separately; here the factorization itself breaks.
3. **Analytical models don't rescue you.** Mass-spring, position-based dynamics, and FEM/continuum approaches "are not capable of accurately modeling deformable objects due to their infinite state dimensions and the difficulty in acquiring parameters in the real world" **[S]** — you can't system-identify a towel.
4. **Even the *goal* is ill-defined.** "What does it mean for a cloth to be 'folded,' water to be 'poured,' or fruit to be 'picked'? Defining these tasks in a generalizable way is an open problem at the heart of DOM research" **[S]**. Rigid tasks have pose goals; deformable tasks have vibes. This wrecks both reward design (RL) and evaluation (benchmarks).
5. **Maturity gap, stated plainly**: DOM "has historically been less studied than manipulation of rigid objects" given its compounded difficulty **[S]**, and current methods are narrowly task- and object-specific rather than generalizable **[S]**.

---

## 2. The benchmark reality check: SoftGym (CoRL 2020) **[S]**

SoftGym is the field's SoftGym-shaped mirror: 10 simulated deformable tasks (rope, cloth, fluids) on NVIDIA's FleX particle simulator with a Gym API, split Medium/Hard/Robot **[S]**. Its findings remain the cleanest statement of why the standard playbook fails:

- **Vision-based RL flops**: CURL-SAC, DrQ, and PlaNet perform "far below the optimal performance on many tasks," especially StraightenRope, SpreadCloth, FoldCloth — with learning curves suggesting more training won't close the gap **[S]**.
- **It's not (just) the perception**: a **Full-State Oracle** feeding ground-truth positions of *all particles* to an MLP+SAC policy "performs poorly on all tasks" — the high-dimensional state itself defeats model-free RL. A hand-designed **Reduced-State Oracle** (4 cloth corners; 10 rope keypoints) succeeds only when the reduction happens to capture task-relevant structure **[S]**. Moral: *representation, not algorithm, is the bottleneck* — the finding that motivated graph networks (§3).
- Practicalities: ~4× real-time with rendering on a 2080Ti; 1M sim steps = 6 h wall-clock vs ≥35 h of real-robot collection; 1000 pre-sampled task variations each **[S]**.

Successor evidence from DaXBench (differentiable-physics benchmark) **[S]**: gradient-through-physics methods dramatically beat PPO on deformables — Analytic Policy Gradients reach reward 1.00 vs PPO's 0.34 on Whip-Rope; Short-Horizon Actor-Critic 0.91 vs PPO's 0.32 on Pour-Water; demonstrations + differentiable physics (ILD) achieve 0.85 on sparse-reward Fold-T-shirt **[S]**. **The PPO recipe that owns locomotion does not carry over** — deformables reward methods that exploit structure (gradients, models, demonstrations) rather than brute sampling.

---

## 3. The method that worked: learned particle/graph dynamics + planning

The dominant successful pattern in deformables is *model-based*: represent the object as particles/mesh nodes, learn the dynamics with a **graph neural network** (message passing ≈ local physical interaction), then *plan* through the learned model (MPC/shooting) [K, pattern; instances below].

**Flagship instance — RoboCook (CoRL 2023 best-systems-paper line of work) [S]:**
- Task: long-horizon elasto-plastic manipulation — a Franka arm making **dumplings and alphabet-letter cookies from dough** with 15 3D-printed tools.
- Method: point-cloud scene → ~300 surface particles → **GNN dynamics model**, + PointNet++ tool classifier + self-supervised policy nets **[S]**.
- Data efficiency: **~20 minutes of real interaction data per tool** **[S]** — contrast π0's 10,000 hours; models-of-structure buy orders of magnitude.
- Results: human-evaluated 0.90 ± 0.11 vs 0.17–0.54 baselines on letter shaping; planning in 9.3 s vs 600–1900 s for baselines; robust to a human deforming/replacing the dough mid-task; generalizes across Play-Doh, clay, foam without retraining **[S]**.

Earlier landmarks in the same family [K]: NeRP/VCD for cloth, FlingBot (dynamic flinging to unfold), Ha & Song's smoothing lines, goal-conditioned Transporter Nets for rope/cloth rearrangement.

---

## 4. The 2025 turn: diffusion everything, transformers eat the GNN

Two sourced 2025 signals show your generative-AI world arriving here:

1. **UniClothDiff (arXiv:2503.11999)** **[S]** — cloth manipulation as *three diffusion models*: a Diffusion Perception Model (generative state estimation of the full cloth state from partial RGB-D — perception as conditional generation, exactly your inpainting intuition), a Diffusion **Dynamics** Model, and an MPC on top. Notably it **replaces GNN dynamics with transformers**, arguing graph locality throttles long-range propagation; reported ~10× lower long-horizon prediction error than GNN baselines (36-step MSE 0.05 vs 0.75 on cloth), and 9/10 real-robot success on square-cloth and T-shirt folding (vs 2–6/10 baselines) with zero-shot sim-to-real from ~200–500K SAPIEN samples **[S]**.
2. **Dynamics-Informed Diffusion Policy (arXiv:2505.17434)** **[S]** — diffusion policy for *dynamic* 3D deformable control (whip-like continuum object), learning inverse dynamics in a 20-DoF reduced-order Cosserat-rod space with physics-informed test-time adaptation; 55K simulated trajectories; **simulation-only**, with success falling from ~94% (10 cm tolerance) to ~62% (2 cm) — precise dynamic deformable control isn't solved even in sim **[S]**. The paper also documents that most prior deformable work restricts itself to 2D/planar settings for tractability **[S]**.

Pattern to notice: the rigid-manipulation stack (generative policies, big models) is diffusing into deformables with a lag of ~2 years, but *model-based structure survives* here (diffusion dynamics + MPC, reduced-order physics) because the state problem never went away.

**Cloth benchmark anchor** **[S]**: the deformables survey's cited SOTA bimanual garment-folding system (2022-era): 93% success folding randomly placed garments, ~120 s average, after learning from 4,300 human-labeled grasp actions.

---

## 5. Application domains, honestly assessed [K unless tagged]

- **Cloth/laundry**: the most-studied; folding from flattened states works (§4); crumpled-to-folded pipelines and garment diversity remain hard. π0's laundry-folding demos **[S]** show the *end-to-end BC* route can reach cloth without explicit deformable modeling — at 3 orders of magnitude more data.
- **Rope/cables**: routing, untangling, knot-tying (surgical suturing lineage); industrial cable harness assembly is a live commercial target.
- **Food/cooking**: RoboCook's dough **[S]**; cutting, scooping, and granular media are active; food is where deformable-DOM meets safety/hygiene constraints.
- **Surgical robotics**: tissue is the ultimate deformable. Supervised autonomy exists (the STAR system's autonomous laparoscopic bowel anastomosis, 2022 [K]); regulatory and safety constraints keep learned policies far from clinical autonomy.
- **Long-horizon composition** is the survey-flagged frontier: "compositional generalization for subskills is a significant challenge for long-horizon tasks" **[S]**.

## 6. Soft robots as actuators [K]

The inverse problem: compliant hardware (pneumatic grippers, granular jamming, Festo-style continuum arms) makes *grasping* easier by conforming to objects — morphological computation replacing control precision — at the cost of hard modeling/proprioception problems (infinite-DoF *self*-state). Learning-based control of soft actuators is its own field; reduced-order models like the Cosserat rods of §4.2 bridge both communities. For policy purposes: soft grippers *simplify* the action space; soft objects *explode* the state space.

---

## 7. Maturity scorecard vs. rigid manipulation

| Axis | Rigid (L4) | Deformable |
|---|---|---|
| State representation | Solved-ish (6-DoF pose + shape priors) | Open; infinite-dim, self-occluding **[S]** |
| Simulator fidelity | Weak for contact **[S]** | Weaker (FleX/MPM particle approximations) [K] |
| Dominant paradigm | End-to-end BC/VLA **[V]** | Model-based: learned dynamics + planning **[S]**, now diffusion-hybrid **[S]** |
| Data cost exemplar | π0: ~10⁴ hours **[S]** | RoboCook: 20 min/tool **[S]** (structure ≫ scale) |
| Benchmarks | Fragmented but numerous **[S]** | SoftGym/DaXBench, sim-only, aging **[S]** |
| Goal specification | Mostly definable | Open research problem **[S]** |
| Generalist policies | Emerging (VLAs) | None; task/object-specific **[S]** |

Bottom line: deformable manipulation is where rigid manipulation was around 2019 — pre-foundation-model, benchmark-fragmented, structure-hungry — but with a harder theoretical core (the state space itself). If manipulation is the frontier, deformables are the frontier's frontier; anyone claiming a "solved" story here is selling something.

---

## References (this lecture)

- Deformables survey — arXiv:2312.10419 (Neurocomputing) **[S]**
- 2026 DOM survey — arXiv:2602.22998 **[S]** (goal-definition problem; DaXBench numbers)
- SoftGym — Lin et al., CoRL 2020, PMLR v155 **[S]**
- RoboCook — arXiv:2306.14447 **[S]**
- UniClothDiff — arXiv:2503.11999 **[S]**
- Dynamics-Informed Diffusion Policy — arXiv:2505.17434 **[S]**
- π0 laundry demos — pi.website **[S]**
- [K]: FlingBot; VCD; Transporter Nets; STAR surgical system (2022); Festo/jamming grippers; MPM/FleX simulation lineage
