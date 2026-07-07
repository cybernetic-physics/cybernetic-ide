# Lecture 04 — Rigid-Body Manipulation: The Frontier

*Series: Robot Policies (see [syllabus](robot-policies-00-syllabus.md) for the [V]/[S]/[K] evidence legend)*

---

## 1. Why manipulation resisted the locomotion recipe

Locomotion's recipe (Lecture 3) requires a fast, faithful simulator and a specifiable reward. Manipulation breaks both legs of that stool:

1. **Contact-rich dynamics defeat simulators.** Verified, from the ManipulationNet benchmark paper: simulation-based manipulation benchmarks "offer reproducible tasks and accessibility at scale, but their imperfect approximations of contact dynamics render results incapable of fully reflecting the true manipulation capabilities" — i.e., sim systematically overestimates real capability **[S]**. Friction, compliance, and intermittent contact are exactly what current physics engines approximate worst [K].
2. **Rewards don't write themselves.** "Pour the coffee without splashing" has no clean scalar. Locomotion's velocity-tracking reward has no manipulation analogue at task generality [K].
3. **Semantic diversity.** A walking policy meets one distribution (terrain); a kitchen policy meets thousands of objects, layouts, and instructions. This is a *representation* problem — hence pretrained vision-language backbones **[V]**.
4. **Partial observability**: occlusion by the robot's own gripper, unobservable object properties (mass, friction, contents) [K].

The verified status line: as of the 2025 IL-manipulation survey (82 methods reviewed), imitation-learned manipulation policies "suffer from evident overfitting, and the generalization of the policies remains a widespread and unresolved issue" **[V]**. Even the flagship generalization result, π0.5, self-describes as "far from perfect," calling generalization "the fundamental challenge" **[V]**. Manipulation is the frontier not because nothing works, but because *nothing generalizes* the way locomotion's recipe does.

So the field pivoted: **if you can't simulate or specify, imitate.** The modern history of manipulation policies is the history of scaling behavior cloning.

---

## 2. The 2023 duo: making BC actually work

### 2.1 ACT / ALOHA — chunking (Zhao et al., RSS 2023) **[V]**

Covered mechanically in Lecture 2 §3.1: a CVAE transformer that emits ~100-step chunks of absolute joint targets at 50 Hz control on the low-cost ALOHA bimanual rig; ~50 demos → 80–90% success on fine bimanual tasks (ziploc bags, battery insertion). Chunking's k-fold horizon reduction is *the* transferable idea — every VLA below inherits it **[V]**.

### 2.2 Diffusion Policy — multimodality (Chi et al., RSS 2023) **[S]**

Also in Lecture 2 §3.2: visuomotor policy as conditional denoising over action sequences; receding-horizon execution; +46.9% average over prior SOTA across 12 tasks/4 benchmarks **[S]**. Its core justification — demonstration data is *multimodal*, and mean-seeking regression fails where mode-seeking generation succeeds — is the reason generative heads (diffusion, then flow) became the default action decoder in everything below.

---

## 3. The VLA era: policies become foundation models

**Definition, verified**: VLAs *are* policies — language-conditioned observation-to-action mappings **[V]**. Two action-decoding paradigms dominate **[V]**:

| Paradigm | Mechanism | Exemplars |
|---|---|---|
| **Discrete tokens** | Bin each action dim (typically 256 bins), emit as next-token classification on the LLM stack | RT-2, OpenVLA (bins mapped to the 256 least-used Llama-2 tokens, cross-entropy) **[V]** |
| **Continuous generative** | Flow-matching/diffusion module generates action chunks conditioned on VLM features | π0 (flow), GR00T N1 (diffusion transformer w/ flow-matching objective) **[V]/[S]** |

Known limits of the discrete route: quantization error and slow autoregressive decoding at high control rates **[V]** — the stated motivation for Physical Intelligence's FAST tokenizer (DCT-compressed action tokens; universal FAST+ tokenizer trained on ~1M action sequences; ~5× faster training than diffusion VLAs) **[S]**.

### 3.1 Lineage in five steps [K for narrative; tagged facts]

1. **RT-1 → RT-2 (Google, 2022–23)** [K]: RT-2's move — co-fine-tune a VLM on web data + robot trajectories with actions as tokens — established "VLA" as a category.
2. **Open X-Embodiment / RT-X (2023)** [K]: the cross-lab dataset pooling (~1M+ episodes, 20+ embodiments) that made open generalist training possible; π0 explicitly complements its own corpus with OXE **[S]**.
3. **OpenVLA (2024)** **[S]**: first fully open-source 7B VLA **[V]**; Llama-2 backbone + fused DINOv2/SigLIP visual encoder; trained on 970K real robot demonstrations; outperforms the closed 55B RT-2-X by 16.5% absolute across 29 tasks with 7× fewer parameters; LoRA-fine-tunable on consumer GPUs **[S]**.
4. **π0 (Physical Intelligence, Oct 2024)** **[V]**: 3B PaliGemma + 300M from-scratch **action expert** (a separate-weights transformer expert with block-wise attention — not merely a "head") = 3.3B total; conditional flow matching over H=50 action chunks; up to 50 Hz; pretrained on ~10,000 hours of dexterous demonstrations across 7 robot configurations and 68 tasks + OXE **[V]/[S]**. Out-of-box evaluations: best across all tasks vs. OpenVLA and Octo, near-perfect on shirt folding and easy bussing; the paper attributes OpenVLA's failures to autoregressive discretization without chunking **[S]**. Demonstrated on laundry folding, table bussing, grocery bagging, box assembly **[S]**. Follows an LLM-style pretrain → post-train recipe, including a hierarchical mode where a high-level VLM issues language subcommands to low-level π0 **[S]**. π0.5 (2025) extends to open-world homes; generalization still self-assessed "far from perfect" **[V]**.
5. **GR00T N1 (NVIDIA, Mar 2025)** **[S]**: the humanoid foundation policy, treated in depth in Lecture 6 (dual-system architecture; 2.2B params; System 2 at 10 Hz, System 1 diffusion-transformer at 120 Hz; "data pyramid" with 88 h of teleop expanded ~10× by neural trajectories + 780K sim trajectories; ~50K H100-hours of pretraining; beats Diffusion Policy baselines by ~30 points on real GR-1 bimanual tasks, ~76.6% success).

### 3.2 The plot twist: maybe you don't need the generative head **[S]**

OpenVLA-OFT (Stanford, Feb 2025): fine-tuning OpenVLA with **parallel decoding + action chunking + continuous actions + plain L1 regression** hits **97.1%** average on LIBERO — beating π0, Octo, Diffusion Policy et al. on that benchmark — with 26× faster action generation than base OpenVLA; with FiLM language conditioning it runs high-frequency control on a real bimanual ALOHA, topping RDT-1B, π0, ACT, and Diffusion Policy on four dexterous tasks **[S]**. Fine-tuning cost: 50–150K gradient steps, 8×A100/H100, 1–2 days; bf16 inference in ~16–18 GB **[S]**.

Lesson: the *chunking + continuous action* insight matters more than which generative machinery decodes it — and benchmark rankings among VLA variants remain unstable (Lecture 6's benchmark crisis).

---

## 4. Dexterous hands [K — background; verification pass returned no surviving claims here]

- **OpenAI Dactyl / Rubik's Cube (2018–19)**: the proof that domain-randomized RL could produce in-hand reorientation on a 24-DOF Shadow Hand — automatic domain randomization (ADR) at massive sim scale. A legacy result: the program disbanded, but ADR became standard vocabulary.
- **Hardware democratization**: LEAP Hand (~$2K, 16-DOF) and similar open designs moved dexterity research out of the Shadow-Hand price class.
- **Sim-scale RL dexterity**: in-hand rotation and reorientation via parallel-sim RL + distillation (e.g., touch/proprioception-only rotation on real hands from the Berkeley/Meta line of work); bimanual+hands humanoid teleoperation datasets feeding VLA training.
- **Tactile sensing**: GelSight-style optical tactile sensors, Meta Digit — rich sensing exists, but integration into flagship generalist policies remains sparse; most VLAs are vision+proprioception only. Treat "tactile-integrated foundation policy" as an open slot on the 2026 bingo card.

---

## 5. The data problem — manipulation's real bottleneck

Every paradigm above is starved by the same constraint: **there is no internet of robot actions.** The field's countermeasures [K, with tagged instances]:

1. **Teleoperation at scale**: ALOHA rigs; Physical Intelligence's ~10K-hour corpus **[S]**; humanoid teleop via VR. Cost: human-hours ≈ robot-hours.
2. **Portable data collection**: UMI (Chi et al., 2024) — hand-held gripper + GoPro turns any human into a demonstration source without a robot [K].
3. **Simulation data generation**: MimicGen / DexMimicGen (NVIDIA) — synthesize thousands of demos from a handful by re-composing object-centric segments [K]; GR00T N1's 780K sim trajectories (~6,500 h equivalent) **[S]**.
4. **Neural/world-model data**: GR00T N1's "neural trajectories" — model-generated video expansions of real teleop (88 h → 827 h, ~10×) **[S]**. This is where your Cosmos/world-model background plugs in: world models as *data engines* for policies.
5. **Cross-embodiment pooling**: OXE/RT-X [K]; π0's 7-platform corpus **[S]**.

The data pyramid (GR00T's term **[S]**) is the field's consensus picture: a broad base of web/human video, a middle of simulation and synthetic trajectories, a small expensive peak of real teleoperation.

---

## 6. Status assessment

| Dimension | State (2025–26) | Evidence |
|---|---|---|
| Single-task, in-distribution BC | Strong (80–97% on benchmarks) | ACT **[V]**, OpenVLA-OFT **[S]** |
| Language-conditioned multi-task | Real but brittle; benchmark-dependent rankings | π0 vs OpenVLA vs OFT **[S]** |
| Open-world generalization | *The* unresolved problem | 2025 survey; π0.5 self-assessment **[V]** |
| Dexterous in-hand manipulation | Demonstrated in narrow regimes; not integrated into generalists | [K] |
| Tactile integration | Sparse in flagships | [K] |
| Sim-to-real for manipulation | Core unsolved obstacle (dynamics + perception gaps) | **[V]** |
| Standard evaluation | Missing — "no ImageNet of robotics" | **[S]** (Lecture 6) |

Compare with Lecture 3's table and the asymmetry your acquaintances gestured at becomes precise: locomotion has a settled *method* with an open long tail; manipulation has impressive *artifacts* and no settled method — the recipe itself (which data, which action decoder, RL or not) is still in play. That is what "manipulation is the frontier" means, verified **[V]**.

---

## References (this lecture)

- IL-manipulation survey — arXiv:2508.17449 **[V]** (overfitting/generalization status)
- VLA anatomy survey — arXiv:2512.11362 **[V]** (tokenization paradigms; chunking)
- ACT — arXiv:2304.13705 **[V]**; Diffusion Policy — arXiv:2303.04137 **[S]**
- π0 — arXiv:2410.24164 + pi.website/blog **[V]/[S]**; π0.5 — arXiv:2504.16054 **[V]**; FAST — arXiv:2501.09747 **[S]**
- OpenVLA — arXiv:2406.09246 **[S]**; OpenVLA-OFT — arXiv:2502.19645 / openvla-oft.github.io **[S]**
- GR00T N1 — arXiv:2503.14734 **[S]**
- ManipulationNet — arXiv (Mar 2026) **[S]** (sim overestimation; benchmark gap)
- [K]: RT-1/RT-2; Open X-Embodiment; UMI (Chi et al. 2024); MimicGen/DexMimicGen; OpenAI Dactyl/Rubik's; LEAP Hand; GelSight/Digit
