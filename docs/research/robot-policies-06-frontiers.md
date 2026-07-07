# Lecture 06 — Frontiers & Cross-Cutting Themes

*Series: Robot Policies (see [syllabus](robot-policies-00-syllabus.md) for the [V]/[S]/[K] evidence legend)*

Four themes cut across Lectures 3–5 and define where the field is heading: hierarchy, the reality gap, the evaluation crisis, and the scaling-laws question.

---

## 1. Hierarchy: System 2 over System 1

The single clearest architectural trend of 2024–2026: split the policy into a **slow deliberative layer** (VLM/VLA reasoning about *what* to do) and a **fast reactive layer** (generating *motor* actions), echoing Kahneman's System 2/System 1.

**Why it emerged** — sourced motivation: monolithic end-to-end VLAs struggle with (a) efficient real-time inference, (b) pretraining cost, and (c) end-to-end fine-tuning complexity on embodied data (domain shift, catastrophic forgetting) **[S]** (dual-system VLA taxonomy). You already saw the physics reason in Lecture 1: control needs 50–500 Hz, billion-parameter models deliver 5–10 Hz. Hierarchy is the rate adapter.

**Flagship: GR00T N1 (NVIDIA, arXiv:2503.14734, Mar 2025)** **[S]**
- Dual-system VLA, *jointly trained end-to-end*: System 2 = Eagle-2 VLM (SmolLM2 LLM + SigLIP-2 encoder; 1.34B of 2.2B total params) interprets vision + language at **10 Hz**; System 1 = diffusion transformer trained with action flow matching generating closed-loop actions at **120 Hz** (chunks of H=16, K=4 inference steps, 63.9 ms per chunk on an L40, bf16).
- Trained on the **data pyramid**: 88 h in-house GR-1 teleop, expanded ~10× to 827 h via *neural trajectories* (world-model-generated data — your Cosmos intuition operationalized), + 780K simulation trajectories (~6,500 h), + human egocentric video; ~50K H100-hours of pretraining.
- Results (self-reported): beats Diffusion Policy on RoboCasa (32.1% vs 25.6%), DexMimicGen (66.5% vs 56.1%), and real Fourier GR-1 bimanual tasks (~76.6% success; ~+30 pts over DP full-data) **[S]**.

**Figure's Helix** **[S/K]**: a dual-system VLA for humanoid control — published only as a company project page, no paper **[S]**; public materials describe a ~7B VLM System 2 at 7–9 Hz commanding an ~80M visuomotor System 1 at 200 Hz [K — company-reported, unverified]. Treat all Helix numbers as marketing-grade.

**The research lineage is dated and real** **[S]**: LCB (IROS 2024), HiRT (CoRL 2024), RoboDual (Oct 2024), OpenHelix survey (May 2025) — hierarchical VLA is a recognized paradigm, not just two companies' branding. π0 itself runs a hierarchical mode: a high-level VLM issues language subcommands to low-level π0 **[S]**.

**The full-stack humanoid picture** (synthesis): VLA System 2 (10 Hz) → action-chunk System 1 (50–120 Hz) → *whole-body tracking policy* à la HOVER/ExBody2 (50 Hz) → PD loops (0.5–1 kHz). Locomotion (L3) and manipulation (L4) research are converging into layers of one stack — HOVER explicitly positions kinematic motion tracking as the common abstraction that upstream systems (teleop, VLAs) command **[V]**.

---

## 2. The sim-to-real gap — one gap, two very different statuses

**Verified asymmetry [V]:** For locomotion, train-in-sim → zero-shot transfer is *standard working practice* (Lecture 3's entire evidence base). For VLA manipulation, "the sim-to-real gap remains a core obstacle for deploying VLA policies, as discrepancies between simulated and real-world dynamics (friction, latency, actuation response) and perception (illumination, textures, sensor noise) severely degrade policy transfer" — with domain randomization, fidelity enhancement, and robust representations offered as *mitigations, not solutions* **[V]** (Dec 2025 VLA survey; corroborated by the 2026 Annual Reviews "Reality Gap" survey).

Why the asymmetry [K, synthesis]: locomotion's contact set is small and rehearsable (feet × ground), its observations are proprioceptive (easy to simulate), and randomization covers the residual. Manipulation's contact set is combinatorial (any gripper–object–scene triple), its observations are photometric (hard to simulate), and — per ManipulationNet — sim benchmarks *systematically overestimate* real capability because contact dynamics are imperfectly approximated **[S]**.

Live directions [K/S]: real-to-sim-to-real (ASAP's delta-action dynamics correction [K]); identification-over-randomization (ETH's no-DR transfer via actuator physics, Lecture 3 **[S]**); and world-model evaluation (assessing policies inside learned simulators).

---

## 3. The evaluation crisis: no ImageNet of robotics

Sourced from the ManipulationNet paper (Mar 2026) **[S]**:

- "Progress toward general manipulation systems remains fragmented due to the absence of widely adopted standard benchmarks" — as of early 2026, robot manipulation has **no standard benchmark**, even for ostensibly identical tasks.
- The **impossible trinity**: existing benchmarks achieve at most two of *realism* (real-world evaluation), *authenticity* (verifiable standardized execution), and *accessibility* (broad participation) — the claimed root cause of the fragmentation **[S]**.
- Even at launch, its own baselines were task-specific methods — real-world standardized evaluation of *generalist* policies (π0, OpenVLA, GR00T) was still nascent **[S]**.

Practical implication for reading papers [K]: success rates are not comparable across papers (different tasks, objects, initial-state distributions, human graders); simulation numbers inflate; and independent re-evaluations can crater self-reported results (HOVER vs BumbleBee, Lecture 3 **[V-caveat]**). Robotics circa 2026 is pre-GLUE: trust *relative* ablations within a paper far more than *absolute* cross-paper claims.

---

## 4. Scaling laws: hypothesis, not law

What your LLM intuition wants — clean loss-vs-compute power laws — does not yet exist for robot policies. The sourced state of play:

- **Embodiment scaling (locomotion)** **[S]**: CoRL 2025 study — ~1,000 procedurally generated embodiments (humanoids, quadrupeds, hexapods), per-embodiment RL experts distilled into one generalist; generalization to *unseen embodiments* improves with the number of training embodiments, and the policy zero-shot transfers to real robots. The authors *explicitly* frame this as "preliminary empirical evidence for embodiment scaling laws" — hypothesis, not established law **[S]**.
- **Data scaling (manipulation)** [K]: UMI-era studies report power-law-ish generalization gains with demonstration diversity (objects/environments, not just count); π0's 10K hours **[S]** and GR00T's pyramid **[S]** are bets that scale wins, but no one has published a robotics Chinchilla. The confound: quality/diversity/embodiment matter more than token count, and evaluation noise (§3) obscures the exponent.
- The honest 2026 statement: *scaling helps; laws unproven.* Anyone quoting "robotics scaling laws" as settled is extrapolating from n≈3 points on a noisy benchmark.

---

## 5. Where the field is heading, 2026 edition

A defensible short list, each anchored to material from this course:

1. **Foundation policies consolidate the stack** — one pretrained VLA post-trained per robot (π0's LLM-style pretrain→post-train recipe **[S]**; GR00T's open release **[S]**), with the locomotion layer (HOVER-style tracking controllers) as the "device driver" underneath **[V]**.
2. **RL returns on top of imitation** [K]: RL fine-tuning of VLAs (improving beyond demonstration quality, optimizing real task reward) is the visible next wave — early 2025 work exists (e.g., RL-tuned VLA papers, preference-based post-training), but nothing with verified flagship status yet; watch this space.
3. **World models as data engines and evaluators** — neural trajectories **[S]** today; policy evaluation inside learned simulators next [K].
4. **Cross-embodiment as the generalization axis** — OXE pooling [K], π0's 7 platforms **[S]**, embodiment-scaling distillation **[S]**; the bet is that embodiment diversity is to robotics what language diversity was to LLMs.
5. **Loco-manipulation closes the L3/L4 seam** — flagged as open by the locomotion survey **[V]**; humanoid whole-body VLAs (GR00T, Helix) are the current attempt.
6. **The unsexy blockers decide the timeline**: evaluation (§3), sim-to-real for contact (§2), safety certification **[V]** (survey open-problem list), and tactile integration [K]. Progress here won't demo well but will gate deployment.

---

## References (this lecture)

- GR00T N1 — arXiv:2503.14734 **[S]**
- Dual-system VLA taxonomy — github.com/OpenHelix-robot/awesome-dual-system-vla **[S]** (LCB, HiRT, RoboDual, OpenHelix dates; Helix status)
- π0 — arXiv:2410.24164 **[S]** (hierarchical mode)
- VLA anatomy survey — arXiv:2512.11362 **[V]** (sim-to-real status)
- The Reality Gap in Robotics — arXiv:2510.20808 (Annual Reviews 2026) **[V-corroboration]**
- ManipulationNet — arXiv (Mar 2026) **[S]** (benchmark crisis; impossible trinity)
- Embodiment scaling laws — CoRL 2025, PMLR v305 **[S]**
- Locomotion survey — arXiv:2406.01152 **[V]** (safety, loco-manipulation, foundation models as open)
- HOVER — arXiv:2410.21229 **[V]**; BumbleBee — arXiv:2506.12779 (re-evaluation caveat)
- [K]: Figure Helix public materials; ASAP (2025); UMI data-scaling study (2024); RL-fine-tuning-of-VLA early literature (2025)
