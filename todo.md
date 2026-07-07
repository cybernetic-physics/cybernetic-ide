# Cybernetic IDE Robotics TODO

## What We Are Trying To Do

Cybernetic IDE should feel like a robotics-native development environment, not
just a code editor with a simulator window bolted onto it.

The north star is simple: a developer should be able to open Cybernetic IDE,
write Python that looks almost identical to official Unitree SDK2 code, run it
against a Unitree G1 in MuJoCo, inspect the robot visually, ask an AI agent to
debug or modify the behavior, and later move the same conceptual workflow
toward real hardware behind explicit safety gates.

The broad architecture we are pursuing is:

- A Dockerized MuJoCo runtime that owns physics, rendering, and the G1 model.
- A Robot Viewer inside the editor that can orbit, zoom, pan, capture frames,
  and stay responsive while normal IDE UI remains usable.
- A Python package, `cybernetic-robotics`, that gives beginners a small
  `G1Robot` API and gives power users Unitree SDK2-shaped imports.
- A default robotics MCP server so native Cybernetic/Zed agents and ACP agents
  like Codex or Claude can control the simulator, inspect telemetry, edit
  scenes, run Python scripts, and capture screenshots.
- A future provider boundary where simulator mode can use the local MuJoCo
  runtime and real-robot mode can use official Unitree SDK2/CycloneDDS with
  safety gates.

Meta reasoning: we are deliberately building the developer experience first,
but we cannot let that become fake robotics. Every convenience layer should
preserve the shape of the official SDK where possible, document where the
current simulator approximation differs, and leave a clean path toward the real
transport/controller stack.

## Current Completion Read

We are close to a useful prototype, but not close to full robotics parity.

Roughly complete:

- Embedded Robot Viewer connected to the local MuJoCo G1 runtime.
- Dockerized G1 MuJoCo protocol harness.
- Booster-inspired HTTP/WebSocket viewer protocol.
- Beginner Python package API.
- Unitree-shaped high-level arm action facade.
- Unitree-shaped `LocoClient` facade.
- Simulator-backed `rt/lowstate` and `rt/lowcmd` compatibility path.
- MCP tools for simulator lifecycle, camera control, snapshots, scene edits,
  Python control jobs, high-level G1 commands, low-state reads, and low-command
  writes.
- Example scripts for beginner control, arm action, locomotion, behavior
  gallery, yoga flow, and low-level channel control.

Not complete:

- Official CycloneDDS-compatible Unitree SDK2 transport.
- True official `unitree_mujoco` runtime integration.
- Whole-body balance control for low-level commands.
- Real hardware connection mode.
- Safety gates suitable for physical robots.
- Rich Robot Viewer telemetry panels.
- Agent workflows that can inspect, repair, and iterate on robot behavior with
  the same quality expected from software coding agents.

Practical estimate: the current goal is about 65 percent complete for a local
Cybernetic IDE simulator demo, and about 25 percent complete for the larger
ambition of official SDK2-compatible sim-to-real robotics development.

## Why This Goal Matters

The immediate value is lowering the activation energy for robotics development:
someone new can run a script and make a G1 move without learning DDS, MuJoCo
asset paths, Docker wiring, or the reversed viewer protocol.

The deeper value is product direction. If Cybernetic IDE can make robotics code
feel inspectable, editable, and agent-assisted, then robotics workflows become
much more like ordinary software workflows: write code, run it, see the effect,
ask an agent to diagnose the state, patch the code, repeat.

The risk is building a pretty demo that teaches the wrong mental model. That is
why each layer needs explicit caveats:

- High-level arm and locomotion calls are simulator facades today.
- Low-level `rt/lowcmd` currently applies held MuJoCo joint targets, not a real
  Unitree low-level controller.
- The local HTTP bridge is not CycloneDDS.
- Real robots require different safety and transport assumptions.

## Immediate Next Tasks

### 1. Keep Robotics Commits Synced To GitHub

Status: ongoing.

Why: the project is moving quickly, and the user explicitly asked us to
commit/push often. Keeping GitHub current lets future agents, local IDE
sessions, and external review all start from the same truth.

Acceptance:

- `git status --short --branch` shows a clean worktree.
- `main` is pushed to `cybernetic-origin/main`.
- Each meaningful robotics slice includes code, docs, examples, and validation
  notes together instead of scattering the truth across multiple later commits.

### 2. Re-run Live Validation After The Held-Lowcmd Change

Status: complete as of the latest lowcmd telemetry slice.

Why: validation revealed that free-running low-level torque commands can topple
the robot because we do not yet have Unitree's balance controller. We changed
the simulator approximation to apply low-level joint targets as held MuJoCo
frames. That must be validated live before treating the slice as done.

Acceptance:

- Done: rebuilt `cyber/unitree-g1-mujoco-protocol:0.1.0`.
- Done: recreated the `unitree-g1-mujoco` container.
- Done: `curl http://127.0.0.1:38383/lowstate` returns motor and IMU state.
- Done: `PYTHONPATH=packages/cybernetic-robotics/src python3 examples/g1_lowcmd_sdk.py`
  runs without falling the robot.
- Done: `/status` shows `fallen: false` after the low-level demo.
- Done: captured `.runtime/g1-control-demo/lowcmd-telemetry-smoke.jpg` after
  the demo.

### 3. Tighten Low-Level Simulator Semantics

Status: partially complete.

Why: the low-level channel path is useful, but it is still an approximation.
The simulator should behave predictably for developers and agents while making
it obvious where official controller behavior is missing.

Tasks:

- Done: expose `mode_pr`, `mode_machine`, and command CRC in `/lowstate`.
- Done: clamp low-level commands by G1 variant and joint limits.
- Done: record which motor indices were accepted, ignored, or clamped.
- Done: add explicit errors for invalid command shapes instead of silently
  skipping malformed entries.
- Done: add a command watchdog timeout, not only the current `received_at`
  timestamp.
- Done: add tests for malformed command lists.
- Remaining: add direct harness-level clamping tests once the local test
  environment can import MuJoCo/numpy or the validation logic is extracted into
  a dependency-light helper.

Reasoning: clear low-level semantics make examples safer and make AI agents
better debugging partners. Agents need structured feedback, not vibes.

### 4. Replace Local HTTP Channel Approximation With Official SDK2/DDS Bridge

Status: pending.

Why: this is the major gap between the current demo and the real product. The
official Unitree stack talks through SDK2/CycloneDDS topics. Our current
`unitree_sdk2py` compatibility layer preserves import shapes but routes through
local HTTP.

Tasks:

- Partial: build an opt-in sidecar with pinned official
  `unitree_sdk2_python`, `unitree_sdk2`, and `unitree_mujoco` sources mounted
  read-only, Debian CycloneDDS installed, the official Python CycloneDDS
  binding built, and a JSON diagnostics entrypoint that initializes a DDS
  domain and creates `rt/lowcmd`/`rt/lowstate` SDK2 channel objects.
- Remaining: launch official `unitree_mujoco`, run official G1 SDK2 examples,
  and prove DDS pub/sub against that simulator peer.
- Done: add `unitree_official_mujoco_plan` so agents can see the missing
  upstream native binary/MuJoCo symlink/build gate and the exact G1 launch
  command before attempting the peer pub/sub proof.
- Done: add and verify `unitree_build_official_mujoco_peer`; it builds the
  upstream native C++ `simulate/build/unitree_mujoco` binary from the pinned
  runtime cache.
- Done: add `unitree_probe_official_mujoco_launch`; raw launch uncovered the
  missing DDS library path and GLFW display requirement, and the sidecar now
  uses Xvfb plus explicit Unitree/MuJoCo library paths for a headless startup
  probe.
- Done: add and verify `unitree_probe_official_mujoco_dds`; it launches the
  official G1 peer under Xvfb, subscribes to `rt/lowstate` with official
  Unitree HG IDL types, and received a 35-motor `LowState_` sample on DDS
  domain `1` with `mode_machine=5`. The report still surfaces CycloneDDS
  loopback multicast warnings so the next sustained-control work does not hide
  transport risk.
- Done: add and verify `unitree_probe_official_mujoco_lowcmd`; it reads
  official `rt/lowstate`, builds a CRC-valid 35-motor HG `LowCmd_` using the
  current `mode_machine`, and successfully published 8 of 8 safe hold frames to
  official `rt/lowcmd`.
- Done: add and verify `unitree_probe_official_mujoco_arm_motion`; it sends a
  bounded official `rt/lowcmd` target for `right_shoulder_roll`, publishes 220
  of 220 frames, and verifies via official `rt/lowstate` that the joint moved
  from `0.0` to about `-0.289 rad`.
- Done: parameterize `unitree_probe_official_mujoco_arm_motion` for official
  G1 arm joint, target delta, frame count, and PD gains; verified a non-default
  `left_elbow` run with `delta=0.18`, 120 of 120 official `rt/lowcmd` writes,
  and `rt/lowstate` motion from `0.0` to about `0.207 rad`.
- Done: add and verify `unitree_probe_official_mujoco_arm_pose`; the
  `raise_right_hand` preset publishes a bounded multi-joint HG `LowCmd_`,
  writes 180 of 180 official `rt/lowcmd` frames, and verifies five moved
  right-arm joints through official `rt/lowstate`.
- Done: add `OfficialG1Sim` and `cyber-g1 official raise-hand` so Python users
  can trigger that official Unitree MuJoCo + SDK2/CycloneDDS hand-raise proof
  without writing Docker Compose or MCP boilerplate.
- Done: add `OfficialG1Sim.status()`, `cyber-g1 official status`, and
  `CYBER_UNITREE_TRANSPORT=dds` Python session diagnostics that report official
  sidecar SDK2/CycloneDDS channel creation and MuJoCo peer readiness.
- Done: add bundle-gated LocoMuJoCo yoga policy runtime support to the local
  MuJoCo protocol server, including `yoga_policy` status/start/stop commands,
  cycle/fall telemetry, and `.runtime/unitree-g1-mujoco/policy/` compose
  mounting for deploy bundles.
- Remaining: promote the proven lowstate/lowcmd/motion probes into a long-lived
  DDS session transport, then map the Cybernetic Python facade onto that
  transport for normal developer scripts.
- Implement a provider that can choose `transport=local_http|dds`.
- Keep the Python user code stable while swapping the backend.
- Done: add first transport/session diagnostics that show selected transport,
  sim/real mode, DDS domain, interface, topic freshness, lowcmd timestamps, and
  official sidecar readiness when `transport=dds` in simulator mode.
- Remaining: connect `unitree_session_status` and the normal Python SDK facade
  to a long-lived real SDK2/CycloneDDS sidecar session instead of only
  short-lived official probes.

Reasoning: preserving the user API while replacing the transport is the trick.
The user should feel like the bridge is invisible, but the developer docs must
make the internals inspectable.

### 5. Add A Real Safety Model Before Real Hardware

Status: pending.

Why: it is fine for the local simulator to be playful; it is not fine for real
hardware to be casual. Real G1 control needs explicit unlocks, mode awareness,
fresh telemetry checks, and immediate stop paths.

Tasks:

- Add `mode=sim|real` session selection.
- Require explicit real-robot unlock before any physical network interface is
  used.
- Gate low-level commands behind an expert setting.
- Refuse motion if `rt/lowstate` is stale.
- Add visible emergency stop and API-level `safety_stop`.
- Document what each stop path sends: `StopMove`, damping, zero torque, or
  pause-only simulator behavior.
- Add structured warnings to examples that could be unsafe on hardware.

Reasoning: the same pleasant API that helps beginners can become dangerous if
it hides real-world consequences. Safety needs to be part of the architecture,
not a paragraph at the end.

### 6. Make The Robot Viewer A Telemetry Workbench

Status: pending.

Why: the viewer is currently good enough to see the robot, but robotics
development also needs state: joint positions, FSM mode, command freshness,
fallen state, current scene, active controller, and logs.

Tasks:

- Add a low-state panel with joint table and IMU summary.
- Add command state showing last high-level action, last lowcmd, and watchdog.
- Add scene/object inventory from `/visual_scene`.
- Add camera bookmark controls.
- Add a screenshot/history strip for before/after comparison.
- Ensure viewer interactions never steal mouse input from the normal editor UI.

Reasoning: the viewer should be a debugging instrument, not just a picture.
The more state we surface, the more useful both the human and the agent become.

### 7. Expand Robotics MCP Tools For Agent Workflows

Status: pending.

Why: the Agent Panel should open with robotics superpowers already loaded.
Agents should not need to discover ad hoc scripts before they can be helpful.

Tasks:

- Done: add `g1_joint_state` with named joint mapping.
- Done: add `g1_apply_joint_targets` with joint names rather than only motor
  indices.
- Done: add `viewer_snapshot_series` for multi-angle visual evidence after an
  agent runs a behavior or scene edit.
- Done: expose the existing `viewer_snapshot_file`, `g1_loco_command`,
  `g1_lowstate`, `g1_joint_state`, `g1_apply_joint_targets`, and `g1_lowcmd`
  tools in the default Robotics Agent profile.
- Done: expose `unitree_prepare_sdk2_sidecar` and
  `unitree_sdk2_sidecar_status` in the default Robotics Agent profile so
  agents can inspect the official SDK2 bridge scaffold.
- Done: expose `unitree_official_mujoco_plan` in the default Robotics Agent
  profile so agents can inspect the native official simulator launch gate.
- Done: expose `unitree_build_official_mujoco_peer` in the default Robotics
  Agent profile so agents can build the native official simulator peer.
- Done: expose `unitree_probe_official_mujoco_dds` in the default Robotics
  Agent profile so agents can prove official SDK2 lowstate sample exchange
  before attempting lowcmd control.
- Done: expose `unitree_probe_official_mujoco_lowcmd` in the default Robotics
  Agent profile so agents can prove safe official lowcmd writes before
  attempting deliberate motion.
- Done: expose `unitree_probe_official_mujoco_arm_motion` in the default
  Robotics Agent profile so agents can prove bounded official lowcmd motion
  with configurable arm joint, target delta, frame count, and gains.
- Done: expose `unitree_probe_official_mujoco_arm_pose` in the default Robotics
  Agent profile so agents can prove coordinated bounded arm poses such as
  `raise_right_hand` through official SDK2/CycloneDDS lowcmd.
- Done: add `scene_add_object`, `scene_remove_object`, and
  `scene_list_objects` for simple generated MJCF scene objects.
- Done: add `sim_validate_behavior` that checks fallen state, command
  freshness, and screenshot availability after a script runs.
- Done: expose `sim_policy_status`, `sim_policy_start`, and `sim_policy_stop`
  in the default Robotics Agent profile so agents can inspect or run the
  optional learned-policy runtime when a deploy bundle is mounted.
- Add `unitree_example_scaffold` variants for arm action, locomotion,
  low-level joint target, scene edit, and telemetry monitor.
- Add tool docs that list safety level, side effects, and expected simulator
  state for each tool.

Reasoning: MCP tools are how AI agents become robotics development partners.
Good tools should be task-shaped, inspectable, and safe by default.

### 8. Grow The Example Script Gallery

Status: pending.

Why: examples are part of the product, not garnish. A new developer learns the
system by editing working scripts. A power user judges whether the abstractions
are honest by reading examples that do real things.

Tasks:

- `examples/g1_wave_hand_sdk.py`: high-level wave behavior.
- Done: `examples/g1_official_raise_hand.py`: official sidecar-backed SDK2/DDS
  hand-raise proof.
- `examples/g1_walk_square_loco.py`: `LocoClient` movement pattern.
- `examples/g1_lowstate_monitor.py`: telemetry monitor with joint names.
- Done: `examples/g1_joint_targets.py`: named-joint low-level target demo.
- `examples/g1_scene_obstacle.py`: add an object, move camera, snapshot.
- `examples/g1_agent_debug_loop.py`: run behavior, capture state, print a
  debugging bundle an AI agent can use.
- `examples/g1_safety_stop.py`: show stop/pause/damp behavior clearly.

Reasoning: examples should cover the behaviors a developer naturally wants:
move, wave, inspect telemetry, change the world, stop safely, and debug.

### 9. Keep Documentation Current As Code Changes

Status: ongoing.

Why: this repo is becoming both a product and a reverse-engineering record. If
docs lag behind implementation, agents and humans will make bad assumptions.

Tasks:

- Update `README.md` when user-facing commands or examples change.
- Update `docs/src/cybernetic-robotics-python.md` when package APIs change.
- Update `docs/src/unitree-g1-sdk-integration.md` when architecture changes.
- Update `docs/src/upstream-robotics-audit.md` when upstream research changes.
- Add validation commands to docs whenever a new script or tool is added.
- Keep clear language around simulator approximation vs official SDK parity.

Reasoning: docs are the shared memory of the project. They should say what is
real, what is approximated, and what remains aspirational.

## Larger Research Tracks

### Locomotion And Balance

We cloned and reviewed Locomujoco, Unitree MuJoCo, Unitree SDK2, Unitree SDK2
Python, and Unitree RL Gym. Those repos point toward a future where Cybernetic
IDE can support learned or official locomotion controllers instead of only
pose/kinematic demos.

Open questions:

- Partially answered: `packages/g1-yoga-rl/` now scaffolds the LocoMuJoCo path
  by projecting Cybernetic's yoga pose registry into LocoMuJoCo's Unitree G1
  joint set and providing trajectory, stability, tuning, speed benchmark, and
  experimental PPOJax mimic-training/export scripts.
- Partially answered: `g1-yoga-analyze-stability` now shows the easy curriculum
  now includes every yoga pose except `tree` after the latest foot and torso
  pose tuning. `tree` remains the single-support stretch case that still
  requires a balance controller.
- Should the next balance controller use official Unitree examples first, or a
  learned policy path from Unitree RL Gym / LocoMuJoCo?
- What is the smallest controller that can hold a yoga pose without toppling?
- How do we expose policy execution safely inside the IDE?

### Sim-To-Real Provider Boundary

The provider abstraction should let the same UI and examples target:

- local HTTP simulator approximation
- official Unitree MuJoCo DDS simulator
- physical Unitree G1

Open questions:

- Where should provider config live in Cybernetic settings?
- Which state should be global, per workspace, or per robot session?
- How should the Agent Panel discover which provider is active?

### Agent-Native Robotics Development

The distinctive product bet is that robotics work can be agent-assisted in the
same way software work is agent-assisted, but with visual and physical state in
the loop.

Open questions:

- What is the best artifact for an agent to inspect after a robot script runs:
  logs, screenshots, low-state JSON, or a bundled report?
- How should agents propose scene edits without destroying the current MJCF?
- Which tools should be read-only by default, and which should require explicit
  approval or expert mode?

## Done Criteria For The Current Goal

This goal should not be considered done until:

- The current local simulator demo is stable, documented, and pushed.
- Unitree-shaped Python examples cover high-level arm action, locomotion, and
  low-level channel control.
- The Agent Panel has default MCP tools for simulator lifecycle, viewer
  screenshots, low-state inspection, scene editing, and Python control jobs.
- The docs clearly explain the current approximation and the path to official
  SDK2/DDS parity.
- There is a concrete next engineering plan for replacing local HTTP transport
  with official Unitree SDK2/CycloneDDS in sim mode.
