# Robot Policies: A Graduate Course in Lecture Notes

**Series:** Robot Policies — from π(a|o) to Foundation Models
**Prepared:** July 2026, from a multi-agent deep-research pass (105 agents, 23 sources fetched, 115 claims extracted, 25 adversarially verified by 3-vote panels) plus curated background knowledge.
**Audience:** A reader fluent in modern generative AI — VLAs (π0/π0.5, OpenVLA, RT-2), world models (GR00T, Cosmos), LLMs, diffusion models (Stable Diffusion), and flow-matching models (E2-TTS) — who has *not* studied robot learning. Every lecture builds bridges from that background.

---

## How to read these notes: the evidence legend

Robotics moves fast and is full of confidently repeated folklore. Every substantive claim in this series is tagged:

| Tag | Meaning |
|---|---|
| **[V]** | *Verified.* Survived a 3-vote adversarial verification panel (independent skeptic agents each tried to refute it against primary sources). Highest confidence. |
| **[S]** | *Sourced.* Extracted from a primary paper or lab publication with a verbatim quote, but not run through the adversarial panel (verification budget). Cite-checked once. |
| **[K]** | *Background knowledge.* Standard, widely-taught material (textbook RL, famous papers) reproduced from the author's training knowledge. Reliable for fundamentals, but re-verify any specific number before quoting in your own work. |

Two claims were **refuted** during verification and are deliberately *absent* or corrected in these notes:

1. ~~"A direct-torque policy must itself be evaluated at ~1 kHz at runtime"~~ — refuted 1-2. The locomotion survey argues PD-target policies *permit* low policy rates while torque is produced at 1 kHz, and that torque policies push toward higher evaluation rates, but "must run at 1 kHz" overstates what deployed torque-policy papers actually do.
2. ~~"Diffusion Policy *pioneered* conditional denoising for control"~~ — refuted 0-3 on the priority claim (e.g., Janner et al.'s *Diffuser* and other diffusion-for-decision-making work predates it). The correct, verbatim-supported statement is that Diffusion Policy **introduced "a new way of generating robot behavior by representing a robot's visuomotor policy as a conditional denoising diffusion process"** and made it *work on real robots* — the analogy to Stable Diffusion is pedagogically sound; the priority claim is not.

---

## Course map

| # | Lecture | Core question |
|---|---|---|
| 01 | [What Is a Robot Policy?](robot-policies-01-fundamentals.md) | The object itself: π(a\|o), how it differs from controllers, planners, and world models; observation & action spaces; the control-frequency hierarchy. |
| 02 | [The Taxonomy: How Policies Are Trained](robot-policies-02-training-paradigms.md) | Model-free RL, model-based RL, behavior cloning, diffusion/flow policies, adversarial imitation (GAIL/AMP), motion tracking (DeepMimic), teacher–student distillation, sim-to-real. |
| 03 | [Locomotion: The "Solved" Problem That Isn't](robot-policies-03-locomotion.md) | Massively parallel sim RL, quadrupeds, humanoid whole-body control (HOVER, ExBody2), and a critical audit of "locomotion is solved." |
| 04 | [Rigid-Body Manipulation: The Frontier](robot-policies-04-manipulation-rigid.md) | Why manipulation is hard, Diffusion Policy, ACT/ALOHA, the VLA era (RT-2, OpenVLA, π0/π0.5, GR00T), dexterous hands, data engines. |
| 05 | [Deformable & Soft-Body Manipulation](robot-policies-05-deformable-soft-body.md) | Infinite-dimensional state, GNN dynamics models, SoftGym/DaXBench, diffusion approaches to cloth and dough, surgical robotics, soft grippers. |
| 06 | [Frontiers & Cross-Cutting Themes](robot-policies-06-frontiers.md) | System 1/System 2 hierarchies (GR00T, Helix), the sim-to-real gap, the benchmark crisis, scaling-law debates, 2025–2026 outlook. |
| 07 | [Case Study: LocoMuJoCo & the Unitree G1](robot-policies-07-case-study-loco-mujoco.md) | Grounding every concept from Lectures 1–6 in the code that lives in this repository's sibling projects. |

---

## The one-paragraph thesis of the whole course

A **policy** is a learned closed-loop mapping from observations to actions — the robotics analogue of "the model" in your generative-AI world. The field currently lives in two regimes that barely resemble each other. **Locomotion** policies are *tiny* (2–3 layer MLPs, ~10⁵–10⁶ parameters) trained by **reinforcement learning** in massively parallel GPU simulation (minutes of wall-clock for a walking policy **[V]**) and transferred to hardware via teacher–student distillation and domain randomization; they run at ~50 Hz above 500 Hz–1 kHz PD loops **[V]**. **Manipulation** policies are increasingly *huge* (billion-parameter VLAs — literally language-conditioned policies built on pretrained VLMs, e.g., π0 = 3B PaliGemma backbone + 300M flow-matching action expert **[V]**) trained by **imitation learning** on teleoperated demonstrations, because manipulation's contact-rich, semantically diverse tasks resist both simulation and reward specification. Locomotion is field-robust but explicitly *not solved* by its own leading researchers **[V]**; manipulation generalization "remains a widespread and unresolved issue" per the 2025 survey literature **[V]**. The deformable-object world (Lecture 5) is roughly a decade behind rigid manipulation, and the frontier everywhere (Lecture 6) is hierarchical systems that put a slow VLA "System 2" on top of a fast reactive "System 1."

---

## Primary reading list (anchor sources)

**Surveys (read first):**
- Ha, Lee, van de Panne, Xie, Yu, Khadiv — *Learning-Based Legged Locomotion: State of the Art and Future Perspectives* (arXiv:2406.01152, IJRR 2025) — the locomotion anchor.
- *A Survey on Imitation-Learning-Based Robotic Manipulation Policies* (arXiv:2508.17449, 2025) — the manipulation anchor.
- *The Anatomy of Vision-Language-Action Models* (arXiv:2512.11362, Dec 2025) — the VLA anchor.
- *A Survey on Robotic Manipulation of Deformable Objects* (arXiv:2312.10419, Neurocomputing) — the deformables anchor.

**Landmark primary papers, in course order:**
- Rudin et al., *Learning to Walk in Minutes Using Massively Parallel Deep RL* (arXiv:2109.11978, CoRL 2021)
- Lee et al., *Learning Quadrupedal Locomotion over Challenging Terrain* (Science Robotics, 2020)
- Miki et al., *Learning Robust Perceptive Locomotion for Quadrupedal Robots in the Wild* (Science Robotics, 2022)
- He et al., *HOVER: Versatile Neural Whole-Body Controller for Humanoid Robots* (arXiv:2410.21229, ICRA 2025)
- Ji et al., *ExBody2: Advanced Expressive Whole-Body Control* (arXiv:2412.13196)
- Chi et al., *Diffusion Policy* (arXiv:2303.04137, RSS 2023 / IJRR)
- Zhao et al., *Learning Fine-Grained Bimanual Manipulation with Low-Cost Hardware* (ACT/ALOHA, arXiv:2304.13705, RSS 2023)
- Black et al., *π0: A Vision-Language-Action Flow Model for General Robot Control* (arXiv:2410.24164)
- Kim et al., *OpenVLA* (arXiv:2406.09246) and *OpenVLA-OFT* (arXiv:2502.19645)
- NVIDIA, *GR00T N1: An Open Foundation Model for Generalist Humanoid Robots* (arXiv:2503.14734)
- Lin et al., *SoftGym* (CoRL 2020, PMLR v155)
- Shi et al., *RoboCook* (arXiv:2306.14447, CoRL 2023)

Each lecture ends with its own expanded reference list.
