#!/usr/bin/env node

import { spawn, spawnSync } from "node:child_process";
import fs from "node:fs";
import fsp from "node:fs/promises";
import os from "node:os";
import path from "node:path";

const PROTOCOL_VERSION = "2025-11-25";
const SERVER_VERSION = "0.1.0";
const DEFAULT_GAME_CONTROL_URL = "http://127.0.0.1:38383";
const DEFAULT_WS_URL = "ws://127.0.0.1:8788";
const DEFAULT_CONTAINER = "unitree-g1-mujoco";
const OFFICIAL_MUJOCO_SESSION_CONTAINER = "unitree-g1-sdk2-session";
const UNITREE_RPC_BRIDGE_CONTAINER = "unitree-g1-rpc-bridge";
const DEFAULT_POSE = "raise_right_hand";
const MAX_LOG_BYTES = 256_000;
const CAMERA_BOOKMARKS_PATH = ".runtime/robot-viewer-camera-bookmarks.json";
const G1_ACTION_POSES = {
  release_arm: { sdk_action: "release arm", action_id: 99, pose: "neutral" },
  neutral: { sdk_action: "release arm", action_id: 99, pose: "neutral" },
  two_hand_kiss: { sdk_action: "two-hand kiss", action_id: 11, pose: "two_hand_kiss" },
  left_kiss: { sdk_action: "left kiss", action_id: 12, pose: "left_kiss" },
  right_kiss: { sdk_action: "right kiss", action_id: 13, pose: "right_kiss" },
  hands_up: { sdk_action: "hands up", action_id: 15, pose: "hands_up" },
  clap: { sdk_action: "clap", action_id: 17, pose: "clap" },
  high_five: { sdk_action: "high five", action_id: 18, pose: "high_five" },
  hug: { sdk_action: "hug", action_id: 19, pose: "hug" },
  heart: { sdk_action: "heart", action_id: 20, pose: "heart" },
  right_heart: { sdk_action: "right heart", action_id: 21, pose: "right_heart" },
  reject: { sdk_action: "reject", action_id: 22, pose: "reject" },
  raise_right_hand: { sdk_action: "right hand up", action_id: 23, pose: "raise_right_hand" },
  x_ray: { sdk_action: "x-ray", action_id: 24, pose: "x_ray" },
  face_wave: { sdk_action: "face wave", action_id: 25, pose: "face_wave" },
  high_wave: { sdk_action: "high wave", action_id: 26, pose: "high_wave" },
  shake_hand: { sdk_action: "shake hand", action_id: 27, pose: "shake_hand" },
};
const VIEW_PRESETS = {
  current: {
    description: "Current viewer camera without moving it.",
    commands: [],
  },
  front: {
    description: "Default reset camera framing.",
    commands: [{ action: "reset" }],
  },
  left: {
    description: "Reset camera, then orbit to the robot's left side.",
    commands: [{ action: "reset" }, { action: "orbit", dx: -90, dy: 0 }],
  },
  right: {
    description: "Reset camera, then orbit to the robot's right side.",
    commands: [{ action: "reset" }, { action: "orbit", dx: 90, dy: 0 }],
  },
  rear: {
    description: "Reset camera, then orbit behind the robot.",
    commands: [{ action: "reset" }, { action: "orbit", dx: 180, dy: 0 }],
  },
  top: {
    description: "Reset camera, then tilt toward a top-down debugging angle.",
    commands: [{ action: "reset" }, { action: "orbit", dx: 0, dy: -65 }, { action: "zoom", delta: -0.5 }],
  },
  three_quarter: {
    description: "Reset camera, then capture a three-quarter view.",
    commands: [{ action: "reset" }, { action: "orbit", dx: 45, dy: -12 }],
  },
};
const OFFICIAL_G1_ARM_JOINTS = [
  "left_shoulder_pitch",
  "left_shoulder_roll",
  "left_shoulder_yaw",
  "left_elbow",
  "left_wrist_roll",
  "left_wrist_pitch",
  "left_wrist_yaw",
  "right_shoulder_pitch",
  "right_shoulder_roll",
  "right_shoulder_yaw",
  "right_elbow",
  "right_wrist_roll",
  "right_wrist_pitch",
  "right_wrist_yaw",
];
const OFFICIAL_G1_ARM_POSE_PRESETS = ["raise_right_hand", "raise_left_hand"];

const root = findRepoRoot(process.env.CYBER_ROBOTICS_ROOT || process.cwd());
const jobs = new Map();
let nextJobId = 1;

if (process.argv.includes("--help")) {
  console.log("Usage: cyber-robotics-mcp serve");
  process.exit(0);
}

if (process.argv[2] && process.argv[2] !== "serve") {
  console.error(`Unsupported command: ${process.argv[2]}`);
  process.exit(64);
}

const tools = [
  tool("sim_prepare_runtime", "Prepare the Unitree G1 MuJoCo Docker runtime assets and compose env.", {}, [], {
    readOnlyHint: false,
    idempotentHint: true,
  }),
  tool("sim_start", "Start the Unitree G1 MuJoCo simulator Docker service.", {}, [], {
    readOnlyHint: false,
    idempotentHint: true,
    openWorldHint: true,
  }),
  tool("sim_stop", "Stop the Unitree G1 MuJoCo simulator Docker service.", {}, [], {
    readOnlyHint: false,
    destructiveHint: true,
    idempotentHint: true,
    openWorldHint: true,
  }),
  tool("sim_restart", "Restart the Unitree G1 MuJoCo simulator Docker service.", {}, [], {
    readOnlyHint: false,
    destructiveHint: true,
    openWorldHint: true,
  }),
  tool("sim_status", "Read simulator, Docker, render-cache, and robot status.", {}, [], {
    readOnlyHint: true,
  }),
  tool("unitree_session_status", "Read Unitree G1 session transport, DDS, simulator, and topic diagnostics.", {}, [], {
    readOnlyHint: true,
  }),
  tool("unitree_provider_status", "Summarize the active Unitree provider, command path, telemetry path, and limitations.", {}, [], {
    readOnlyHint: true,
  }),
  tool(
    "unitree_sdk_compatibility_audit",
    "Compare cloned official Unitree G1 SDK2 Python examples with Cybernetic's current unitree_sdk2py shim support.",
    {
      upstream_root: {
        type: "string",
        default: "/Users/cuboniks/wagmi/unitree_sdk2_python",
        description: "Path to the cloned official unitree_sdk2_python repository.",
      },
    },
    [],
    {
      readOnlyHint: true,
      idempotentHint: true,
    },
  ),
  tool(
    "unitree_sdk_behavior_smoke",
    "Run safe behavior-level smoke checks through Cybernetic's Unitree SDK-shaped G1 shim.",
    {
      kind: {
        type: "string",
        enum: ["all", "arm", "loco", "lowcmd", "hand"],
        default: "all",
        description: "Subset of official-style SDK behavior surfaces to smoke test.",
      },
      output_path: {
        type: "string",
        default: ".runtime/sdk-smoke/latest.json",
        description: "Workspace-relative JSON file for the smoke evidence report.",
      },
      transport: {
        type: "string",
        enum: ["local_http", "rpc_bridge", "dds"],
        description: "Optional Unitree transport override for the smoke run.",
      },
    },
    [],
    {
      readOnlyHint: false,
      idempotentHint: true,
      openWorldHint: true,
    },
  ),
  tool("unitree_prepare_sdk2_sidecar", "Prepare pinned official Unitree SDK2 Python, SDK2 C++, and Unitree MuJoCo sources for the opt-in SDK2 sidecar.", {}, [], {
    readOnlyHint: false,
    idempotentHint: true,
    openWorldHint: true,
  }),
  tool("unitree_sdk2_sidecar_status", "Run the opt-in Unitree SDK2 sidecar diagnostic container and return its source, topic, and transport report.", {}, [], {
    readOnlyHint: false,
    idempotentHint: true,
    openWorldHint: true,
  }),
  tool("unitree_official_mujoco_plan", "Report the official Unitree MuJoCo G1 peer build, launch, and DDS probe plan from the SDK2 sidecar.", {}, [], {
    readOnlyHint: false,
    idempotentHint: true,
    openWorldHint: true,
  }),
  tool("unitree_build_official_mujoco_peer", "Build the official Unitree MuJoCo C++ G1 peer inside the SDK2 sidecar runtime cache.", {}, [], {
    readOnlyHint: false,
    idempotentHint: true,
    openWorldHint: true,
  }),
  tool("unitree_probe_official_mujoco_launch", "Launch the official Unitree MuJoCo G1 peer briefly under Xvfb to verify runtime library/display readiness.", {}, [], {
    readOnlyHint: false,
    idempotentHint: true,
    openWorldHint: true,
  }),
  tool("unitree_start_official_mujoco_session", "Start a managed long-running official Unitree MuJoCo G1 DDS peer session container.", {}, [], {
    readOnlyHint: false,
    idempotentHint: true,
    openWorldHint: true,
  }),
  tool("unitree_official_mujoco_session_status", "Inspect the managed official Unitree MuJoCo G1 DDS peer session container.", {}, [], {
    readOnlyHint: true,
    idempotentHint: true,
    openWorldHint: true,
  }),
  tool("unitree_read_official_mujoco_lowstate", "Read one official rt/lowstate sample from the managed Unitree MuJoCo G1 DDS peer session.", {}, [], {
    readOnlyHint: true,
    idempotentHint: true,
    openWorldHint: true,
  }),
  tool(
    "unitree_probe_official_mujoco_loco_rpc",
    "Probe whether the managed official Unitree MuJoCo G1 DDS peer serves the G1 LocoClient sport RPC topics.",
    {
      include_stop: {
        type: "boolean",
        default: false,
        description: "Also issue a safe StopMove RPC after the read probe.",
      },
      timeout_seconds: {
        type: "number",
        minimum: 0.2,
        maximum: 10,
        default: 2,
        description: "Per-RPC timeout used by the official Unitree LocoClient.",
      },
    },
    [],
    { readOnlyHint: false, idempotentHint: true, openWorldHint: true },
  ),
  tool(
    "unitree_probe_official_mujoco_rpc_discovery",
    "Inspect which official Unitree G1 RPC service request topics have matched DDS readers on the managed MuJoCo session.",
    {
      wait_seconds: {
        type: "number",
        minimum: 0.1,
        maximum: 10,
        default: 1,
        description: "Seconds to wait for DDS publication-match discovery.",
      },
    },
    [],
    { readOnlyHint: true, idempotentHint: true, openWorldHint: true },
  ),
  tool(
    "unitree_probe_rpc_bridge_smoke",
    "Start temporary Unitree-shaped sport/agv/arm RPC services in the SDK2 sidecar and call them with official SDK clients.",
    {
      timeout_seconds: {
        type: "number",
        minimum: 0.2,
        maximum: 10,
        default: 1,
        description: "Per-RPC timeout used by the smoke clients.",
      },
    },
    [],
    { readOnlyHint: false, idempotentHint: true, openWorldHint: true },
  ),
  tool("unitree_start_rpc_bridge", "Start a managed Unitree sport/agv/arm RPC bridge container on the SDK2 DDS domain.", {}, [], {
    readOnlyHint: false,
    idempotentHint: true,
    openWorldHint: true,
  }),
  tool("unitree_rpc_bridge_status", "Inspect the managed Unitree sport/agv/arm RPC bridge container.", {}, [], {
    readOnlyHint: true,
    idempotentHint: true,
    openWorldHint: true,
  }),
  tool(
    "unitree_probe_rpc_bridge_client",
    "Call an already-running managed Unitree sport/agv/arm RPC bridge with official SDK clients.",
    {
      timeout_seconds: {
        type: "number",
        minimum: 0.2,
        maximum: 10,
        default: 1,
        description: "Per-RPC timeout used by the bridge clients.",
      },
    },
    [],
    { readOnlyHint: false, idempotentHint: true, openWorldHint: true },
  ),
  tool(
    "unitree_verify_rpc_bridge",
    "Verify the managed Unitree sport/agv/arm RPC bridge and summarize simulator readback/forwarding evidence.",
    {
      timeout_seconds: {
        type: "number",
        minimum: 0.2,
        maximum: 10,
        default: 1,
        description: "Per-RPC timeout used by the bridge clients.",
      },
      start_if_needed: {
        type: "boolean",
        default: true,
        description: "Start the managed bridge if it is not already running and ready.",
      },
      stop_after: {
        type: "boolean",
        default: false,
        description: "Stop the managed bridge after verification.",
      },
    },
    [],
    { readOnlyHint: false, idempotentHint: true, openWorldHint: true },
  ),
  tool(
    "unitree_command_rpc_bridge",
    "Send one SDK-shaped Unitree sport/agv/arm RPC through the managed bridge and summarize simulator forwarding/readback evidence.",
    {
      service: {
        type: "string",
        enum: ["sport", "agv", "arm"],
        default: "sport",
        description: "Unitree RPC service to call.",
      },
      method: {
        type: "string",
        default: "get_fsm_id",
        description:
          "Method alias such as get_fsm_id, get_phase, move, stop_move, damp, wave_hand, shake_hand, set_arm_task, switch_move_mode, set_speed_mode, switch_to_user_ctrl, switch_to_internal_ctrl, height_adjust, execute_action, or get_action_list.",
      },
      params: {
        type: "object",
        default: {},
        description: "JSON parameters for the selected method, for example {\"vx\":0.05,\"omega\":0,\"duration\":0.5}.",
      },
      timeout_seconds: {
        type: "number",
        minimum: 0.2,
        maximum: 10,
        default: 1,
        description: "Per-RPC timeout used by the bridge client.",
      },
      start_if_needed: {
        type: "boolean",
        default: true,
        description: "Start the managed bridge if it is not already running and ready.",
      },
      stop_after: {
        type: "boolean",
        default: false,
        description: "Stop the managed bridge after the command.",
      },
    },
    [],
    { readOnlyHint: false, idempotentHint: false, openWorldHint: true },
  ),
  tool("unitree_stop_rpc_bridge", "Stop and remove the managed Unitree sport/agv/arm RPC bridge container.", {}, [], {
    readOnlyHint: false,
    idempotentHint: true,
    openWorldHint: true,
  }),
  tool("unitree_stop_official_mujoco_session", "Stop and remove the managed official Unitree MuJoCo G1 DDS peer session container.", {}, [], {
    readOnlyHint: false,
    idempotentHint: true,
    openWorldHint: true,
  }),
  tool("unitree_probe_official_mujoco_dds", "Run the official Unitree MuJoCo G1 peer and verify SDK2/CycloneDDS rt/lowstate sample exchange.", {}, [], {
    readOnlyHint: false,
    idempotentHint: true,
    openWorldHint: true,
  }),
  tool("unitree_probe_official_mujoco_lowcmd", "Run the official Unitree MuJoCo G1 peer and verify a safe SDK2/CycloneDDS rt/lowcmd hold-command publish.", {}, [], {
    readOnlyHint: false,
    idempotentHint: true,
    openWorldHint: true,
  }),
  tool(
    "unitree_probe_official_mujoco_arm_motion",
    "Run the official Unitree MuJoCo G1 peer and verify a bounded SDK2/CycloneDDS single-arm-joint motion through rt/lowcmd.",
    {
      joint: {
        type: "string",
        enum: OFFICIAL_G1_ARM_JOINTS,
        default: "right_shoulder_roll",
        description: "Official Unitree HG arm joint to move.",
      },
      delta: {
        type: "number",
        minimum: -0.5,
        maximum: 0.5,
        default: -0.25,
        description: "Target offset in radians from the first observed lowstate position.",
      },
      frames: {
        type: "integer",
        minimum: 20,
        maximum: 600,
        default: 220,
        description: "Number of lowcmd frames to publish.",
      },
      kp: {
        type: "number",
        minimum: 0,
        maximum: 80,
        default: 35.0,
        description: "PD proportional gain for the moving joint.",
      },
      kd: {
        type: "number",
        minimum: 0,
        maximum: 5,
        default: 1.2,
        description: "PD derivative gain for the moving joint.",
      },
      hold_kp: {
        type: "number",
        minimum: 0,
        maximum: 80,
        default: 18.0,
        description: "PD proportional gain used to hold non-target joints near their sampled positions.",
      },
      hold_kd: {
        type: "number",
        minimum: 0,
        maximum: 5,
        default: 0.8,
        description: "PD derivative gain used to hold non-target joints near their sampled positions.",
      },
    },
    [],
    {
      readOnlyHint: false,
      idempotentHint: true,
      openWorldHint: true,
    },
  ),
  tool(
    "unitree_probe_official_mujoco_arm_pose",
    "Run the official Unitree MuJoCo G1 peer and verify a bounded SDK2/CycloneDDS multi-joint arm pose through rt/lowcmd.",
    {
      preset: {
        type: "string",
        enum: OFFICIAL_G1_ARM_POSE_PRESETS,
        default: "raise_right_hand",
        description: "Built-in bounded arm pose to publish when joint_deltas is omitted.",
      },
      joint_deltas: {
        type: "object",
        additionalProperties: { type: "number", minimum: -0.5, maximum: 0.5 },
        description: "Optional map of official G1 arm joint name to target offset in radians. Values are clamped to +/-0.5.",
      },
      frames: {
        type: "integer",
        minimum: 20,
        maximum: 600,
        default: 180,
        description: "Number of lowcmd frames to publish.",
      },
      kp: {
        type: "number",
        minimum: 0,
        maximum: 80,
        default: 30.0,
        description: "PD proportional gain for moving pose joints.",
      },
      kd: {
        type: "number",
        minimum: 0,
        maximum: 5,
        default: 1.0,
        description: "PD derivative gain for moving pose joints.",
      },
      hold_kp: {
        type: "number",
        minimum: 0,
        maximum: 80,
        default: 18.0,
        description: "PD proportional gain used to hold non-target joints near their sampled positions.",
      },
      hold_kd: {
        type: "number",
        minimum: 0,
        maximum: 5,
        default: 0.8,
        description: "PD derivative gain used to hold non-target joints near their sampled positions.",
      },
      min_moved_joints: {
        type: "integer",
        minimum: 1,
        maximum: 8,
        default: 2,
        description: "Minimum target joints that must move beyond the threshold for the probe to pass.",
      },
    },
    [],
    {
      readOnlyHint: false,
      idempotentHint: true,
      openWorldHint: true,
    },
  ),
  tool(
    "unitree_command_official_mujoco_arm_pose",
    "Send a bounded SDK2/CycloneDDS multi-joint arm pose to an already-running managed official Unitree MuJoCo G1 session.",
    {
      preset: {
        type: "string",
        enum: OFFICIAL_G1_ARM_POSE_PRESETS,
        default: "raise_right_hand",
        description: "Built-in bounded arm pose to publish when joint_deltas is omitted.",
      },
      joint_deltas: {
        type: "object",
        additionalProperties: { type: "number", minimum: -0.5, maximum: 0.5 },
        description: "Optional map of official G1 arm joint name to target offset in radians. Values are clamped to +/-0.5.",
      },
      frames: {
        type: "integer",
        minimum: 20,
        maximum: 600,
        default: 180,
        description: "Number of lowcmd frames to publish.",
      },
      kp: {
        type: "number",
        minimum: 0,
        maximum: 80,
        default: 30.0,
        description: "PD proportional gain for moving pose joints.",
      },
      kd: {
        type: "number",
        minimum: 0,
        maximum: 5,
        default: 1.0,
        description: "PD derivative gain for moving pose joints.",
      },
      hold_kp: {
        type: "number",
        minimum: 0,
        maximum: 80,
        default: 18.0,
        description: "PD proportional gain used to hold non-target joints near their sampled positions.",
      },
      hold_kd: {
        type: "number",
        minimum: 0,
        maximum: 5,
        default: 0.8,
        description: "PD derivative gain used to hold non-target joints near their sampled positions.",
      },
      min_moved_joints: {
        type: "integer",
        minimum: 1,
        maximum: 8,
        default: 2,
        description: "Minimum target joints that must move beyond the threshold for the command to pass.",
      },
    },
    [],
    {
      readOnlyHint: false,
      idempotentHint: true,
      openWorldHint: true,
    },
  ),
  tool(
    "unitree_command_official_mujoco_lowcmd",
    "Publish one bounded generic SDK2/CycloneDDS Unitree HG LowCmd frame to an already-running managed official Unitree MuJoCo G1 session.",
    {
      topic: {
        type: "string",
        enum: ["rt/lowcmd", "rt/arm_sdk"],
        default: "rt/lowcmd",
        description: "Official Unitree HG LowCmd topic to publish. Use rt/arm_sdk for official G1 arm SDK examples.",
      },
      motor_cmd: {
        type: "array",
        items: {
          type: "object",
          properties: {
            mode: { type: "integer", minimum: 0, maximum: 15 },
            q: { type: "number", minimum: -3.5, maximum: 3.5 },
            dq: { type: "number", minimum: -20, maximum: 20 },
            tau: { type: "number", minimum: -80, maximum: 80 },
            kp: { type: "number", minimum: 0, maximum: 80 },
            kd: { type: "number", minimum: 0, maximum: 8 },
          },
          additionalProperties: false,
        },
        default: [],
        description: "LowCmd motor_cmd prefix to apply. Unspecified motor slots hold their sampled lowstate positions.",
      },
      mode_pr: { type: "integer", default: 0 },
      mode_machine: { type: "integer", default: 0 },
      crc: { type: "integer", default: 0 },
      frames: {
        type: "integer",
        minimum: 1,
        maximum: 60,
        default: 1,
        description: "Number of identical lowcmd frames to publish.",
      },
      timeout_seconds: {
        type: "number",
        minimum: 0.5,
        maximum: 30,
        default: 6,
        description: "Deadline for reading the safety lowstate sample before publishing.",
      },
    },
    [],
    {
      readOnlyHint: false,
      idempotentHint: false,
      openWorldHint: true,
    },
  ),
  tool(
    "unitree_official_mujoco_evidence_bundle",
    "Run a bounded arm pose against the managed official Unitree MuJoCo G1 DDS session and write before/after rt/lowstate evidence.",
    {
      output_path: {
        type: "string",
        default: ".runtime/official-mujoco-evidence/latest.json",
        description: "Workspace-relative JSON file for the evidence bundle.",
      },
      preset: {
        type: "string",
        enum: OFFICIAL_G1_ARM_POSE_PRESETS,
        default: "raise_right_hand",
        description: "Built-in bounded arm pose to publish when joint_deltas is omitted.",
      },
      joint_deltas: {
        type: "object",
        additionalProperties: { type: "number", minimum: -0.5, maximum: 0.5 },
        description: "Optional map of official G1 arm joint name to target offset in radians. Values are clamped to +/-0.5.",
      },
      frames: {
        type: "integer",
        minimum: 20,
        maximum: 600,
        default: 180,
        description: "Number of lowcmd frames to publish.",
      },
      kp: { type: "number", minimum: 0, maximum: 80, default: 30.0 },
      kd: { type: "number", minimum: 0, maximum: 5, default: 1.0 },
      hold_kp: { type: "number", minimum: 0, maximum: 80, default: 18.0 },
      hold_kd: { type: "number", minimum: 0, maximum: 5, default: 0.8 },
      min_moved_joints: { type: "integer", minimum: 1, maximum: 8, default: 2 },
      start_if_needed: {
        type: "boolean",
        default: true,
        description: "Start the managed official session if it is not already running and ready.",
      },
      stop_after: {
        type: "boolean",
        default: false,
        description: "Stop the managed session after the bundle only when this tool started it.",
      },
    },
    [],
    {
      readOnlyHint: false,
      idempotentHint: true,
      openWorldHint: true,
    },
  ),
  tool("sim_pause", "Pause MuJoCo simulation time.", {}, [], { readOnlyHint: false, idempotentHint: true }),
  tool("sim_resume", "Resume MuJoCo simulation time.", {}, [], { readOnlyHint: false }),
  tool("sim_reset", "Reset the MuJoCo simulation state.", {}, [], {
    readOnlyHint: false,
    destructiveHint: true,
  }),
  tool(
    "sim_step",
    "Advance the MuJoCo simulation by one or more single-step commands.",
    {
      count: { type: "integer", minimum: 1, maximum: 100, default: 1 },
    },
    [],
    { readOnlyHint: false },
  ),
  tool(
    "sim_validate_behavior",
    "Validate simulator health after a robot behavior and optionally capture a viewer snapshot.",
    {
      max_lowcmd_age_seconds: {
        type: "number",
        default: 5.0,
        description: "Maximum acceptable age for the most recent lowcmd when control mode is lowcmd.",
      },
      require_snapshot: {
        type: "boolean",
        default: true,
        description: "When true, capture a viewer frame and fail if it is unavailable.",
      },
      snapshot_path: {
        type: "string",
        default: ".runtime/behavior-validation/latest.jpg",
        description: "Workspace-relative snapshot path when require_snapshot is true.",
      },
    },
    [],
    { readOnlyHint: false },
  ),
  tool(
    "robot_evidence_bundle",
    "Write a reviewable simulator evidence bundle with status, telemetry, provider diagnostics, and optional viewer screenshots.",
    {
      output_path: {
        type: "string",
        default: ".runtime/robot-evidence/latest.json",
        description: "Workspace-relative JSON manifest path.",
      },
      label: {
        type: "string",
        default: "latest",
        description: "Filesystem-safe label used for snapshot filenames.",
      },
      include_snapshot: {
        type: "boolean",
        default: true,
        description: "Capture the current viewer frame beside the manifest.",
      },
      include_snapshot_series: {
        type: "boolean",
        default: false,
        description: "Capture current/front/right/three-quarter views beside the manifest.",
      },
      snapshot_format: {
        type: "string",
        enum: ["jpeg", "png"],
        default: "jpeg",
        description: "Image format for captured viewer evidence.",
      },
    },
    [],
    { readOnlyHint: false, idempotentHint: true, openWorldHint: true },
  ),
  tool(
    "sim_apply_pose",
    "Apply a named simulator pose, such as raise_right_hand or neutral.",
    {
      pose: { type: "string", default: DEFAULT_POSE },
    },
    [],
    { readOnlyHint: false },
  ),
  tool("sim_policy_status", "Read the optional G1 yoga policy runtime status from the simulator.", {}, [], {
    readOnlyHint: true,
  }),
  tool(
    "sim_policy_start",
    "Start the optional LocoMuJoCo-trained G1 yoga policy runtime.",
    {
      loop: { type: "boolean", default: true },
      frame: { type: "integer", minimum: 0, default: 0 },
    },
    [],
    { readOnlyHint: false },
  ),
  tool("sim_policy_stop", "Stop the optional G1 yoga policy runtime.", {}, [], {
    readOnlyHint: false,
    idempotentHint: true,
  }),
  tool(
    "viewer_camera_control",
    "Control the MuJoCo free camera through the simulator protocol.",
    {
      action: { type: "string", enum: ["state", "reset", "orbit", "pan", "zoom", "set"] },
      dx: { type: "number", default: 0 },
      dy: { type: "number", default: 0 },
      delta: { type: "number", default: 0 },
      lookat: {
        type: "array",
        items: { type: "number" },
        minItems: 3,
        maxItems: 3,
        description: "Free-camera look-at point for action=set.",
      },
      distance: { type: "number", minimum: 0.45, maximum: 12.0, description: "Free-camera distance for action=set." },
      azimuth: { type: "number", description: "Free-camera azimuth in degrees for action=set." },
      elevation: { type: "number", minimum: -89.0, maximum: 20.0, description: "Free-camera elevation in degrees for action=set." },
    },
    ["action"],
    { readOnlyHint: false },
  ),
  tool(
    "viewer_camera_bookmark_save",
    "Save the current Robot Viewer free-camera state as a named workspace bookmark.",
    {
      name: { type: "string", default: "default", description: "Bookmark name, e.g. front_debug or hand_closeup." },
      description: { type: "string", default: "", description: "Optional note explaining what this view is useful for." },
    },
    ["name"],
    { readOnlyHint: false },
  ),
  tool("viewer_camera_bookmark_list", "List saved Robot Viewer camera bookmarks.", {}, [], {
    readOnlyHint: true,
  }),
  tool(
    "viewer_camera_bookmark_apply",
    "Restore a saved Robot Viewer free-camera bookmark.",
    {
      name: { type: "string", default: "default" },
    },
    ["name"],
    { readOnlyHint: false },
  ),
  tool(
    "viewer_camera_bookmark_delete",
    "Delete a saved Robot Viewer free-camera bookmark.",
    {
      name: { type: "string", default: "default" },
    },
    ["name"],
    { readOnlyHint: false, destructiveHint: true },
  ),
  tool(
    "viewer_snapshot",
    "Capture the current Robot Viewer camera frame as an image result.",
    {
      format: { type: "string", enum: ["jpeg", "png"], default: "jpeg" },
    },
    [],
    { readOnlyHint: true },
  ),
  tool(
    "viewer_snapshot_file",
    "Capture the current Robot Viewer camera frame to a workspace file and return the path.",
    {
      path: { type: "string", description: "Workspace-relative output path." },
      format: { type: "string", enum: ["jpeg", "png"], default: "jpeg" },
    },
    ["path"],
    { readOnlyHint: false },
  ),
  tool(
    "viewer_snapshot_series",
    "Capture a named set of Robot Viewer camera angles to workspace files for visual debugging.",
    {
      output_dir: { type: "string", default: ".runtime/robot-viewer-snapshots" },
      prefix: { type: "string", default: "g1" },
      format: { type: "string", enum: ["jpeg", "png"], default: "jpeg" },
      views: {
        type: "array",
        items: { type: "string", enum: ["current", "front", "left", "right", "rear", "top", "three_quarter"] },
        default: ["current", "front", "right", "three_quarter"],
      },
    },
    [],
    { readOnlyHint: false },
  ),
  tool("scene_get", "Read the current visual scene summary from the simulator.", {}, [], {
    readOnlyHint: true,
  }),
  tool("scene_read_mjcf", "Read the active Unitree G1 MJCF scene XML from the mounted asset tree.", {}, [], {
    readOnlyHint: true,
  }),
  tool(
    "scene_validate_mjcf",
    "Validate a container-side MJCF path with MuJoCo inside the simulator container.",
    {
      model_path: { type: "string", description: "Container path, defaults to UNITREE_G1_MODEL_PATH." },
    },
    [],
    { readOnlyHint: true },
  ),
  tool(
    "scene_add_box",
    "Create a Unitree G1 MJCF scene copy with an added box object and optionally activate it.",
    {
      name: { type: "string", pattern: "^[A-Za-z0-9_-]+$" },
      position: { type: "array", items: { type: "number" }, minItems: 3, maxItems: 3 },
      size: { type: "array", items: { type: "number" }, minItems: 3, maxItems: 3 },
      rgba: { type: "array", items: { type: "number" }, minItems: 4, maxItems: 4 },
      activate: {
        type: "boolean",
        default: false,
        description: "When true, updates compose.env and recreates the simulator container.",
      },
    },
    ["name", "position", "size"],
    { readOnlyHint: false, destructiveHint: true },
  ),
  tool(
    "scene_add_object",
    "Create a Unitree G1 MJCF scene copy with an added object. Currently supports box objects.",
    {
      type: { type: "string", enum: ["box"], default: "box" },
      name: { type: "string", pattern: "^[A-Za-z0-9_-]+$" },
      position: { type: "array", items: { type: "number" }, minItems: 3, maxItems: 3 },
      size: { type: "array", items: { type: "number" }, minItems: 3, maxItems: 3 },
      rgba: { type: "array", items: { type: "number" }, minItems: 4, maxItems: 4 },
      activate: {
        type: "boolean",
        default: false,
        description: "When true, updates compose.env and recreates the simulator container.",
      },
    },
    ["name", "position", "size"],
    { readOnlyHint: false, destructiveHint: true },
  ),
  tool("scene_list_objects", "List Cybernetic-generated MJCF scene objects and generated scene files.", {}, [], {
    readOnlyHint: true,
  }),
  tool(
    "scene_remove_object",
    "Create a Unitree G1 MJCF scene copy with a Cybernetic-generated object removed and optionally activate it.",
    {
      name: { type: "string", pattern: "^[A-Za-z0-9_-]+$" },
      scene_path: {
        type: "string",
        description: "Optional workspace, host, or /opt/unitree_mujoco/... scene path. Defaults to the active scene.",
      },
      activate: {
        type: "boolean",
        default: false,
        description: "When true, updates compose.env and recreates the simulator container.",
      },
    },
    ["name"],
    { readOnlyHint: false, destructiveHint: true },
  ),
  tool(
    "unitree_sdk_scaffold_python",
    "Generate or write a Unitree SDK-shaped Python control script for the local G1 simulator.",
    {
      path: { type: "string", description: "Optional workspace-relative file path to write." },
      action: {
        type: "string",
        enum: [
          "raise_hand",
          "release_arm",
          "arm_action",
          "locomotion",
          "lowcmd_joint_target",
          "scene_edit",
          "telemetry_monitor",
        ],
        default: "raise_hand",
      },
      sdk_action: {
        type: "string",
        default: "right hand up",
        description: "Arm action name for action=arm_action, for example 'right hand up' or 'release arm'.",
      },
    },
    [],
    { readOnlyHint: false },
  ),
  tool("robotics_tool_reference", "List robotics MCP tools with safety level, side effects, and expected simulator state.", {}, [], {
    readOnlyHint: true,
  }),
  tool("g1_list_actions", "List supported high-level G1 SDK facade actions.", {}, [], {
    readOnlyHint: true,
  }),
  tool(
    "g1_execute_action",
    "Execute a high-level G1 action through the Unitree SDK facade protocol.",
    {
      action: { type: "string", enum: Object.keys(G1_ACTION_POSES) },
    },
    ["action"],
    { readOnlyHint: false },
  ),
  tool(
    "g1_loco_command",
    "Execute a Unitree G1 LocoClient-shaped simulator command such as Move, StopMove, Damp, or Start.",
    {
      command: {
        type: "string",
        enum: [
          "state",
          "get_fsm_id",
          "get_fsm_mode",
          "get_balance_mode",
          "get_swing_height",
          "get_stand_height",
          "get_phase",
          "set_balance_mode",
          "set_swing_height",
          "set_stand_height",
          "set_speed_mode",
          "switch_move_mode",
          "switch_to_user_ctrl",
          "switch_to_internal_ctrl",
          "damp",
          "start",
          "squat",
          "sit",
          "stand_up",
          "zero_torque",
          "stop_move",
          "move",
          "low_stand",
          "high_stand",
          "wave_hand",
          "shake_hand",
        ],
        default: "state",
      },
      balance_mode: { type: "integer", default: 0 },
      swing_height: { type: "number", default: 0 },
      stand_height: { type: "number", default: 0 },
      speed_mode: { type: "integer", default: 0 },
      continuous_move: { type: "boolean", default: false },
      internal_mode: { type: "integer", default: 0 },
      vx: { type: "number", default: 0 },
      vy: { type: "number", default: 0 },
      omega: { type: "number", default: 0 },
      duration: { type: "number", default: 1.0 },
    },
    ["command"],
    { readOnlyHint: false },
  ),
  tool(
    "g1_agv_command",
    "Execute a Unitree G1 AgvClient-shaped simulator command such as Move or HeightAdjust.",
    {
      command: {
        type: "string",
        enum: ["move", "height_adjust"],
        default: "move",
      },
      vx: {
        type: "number",
        default: 0,
        description: "Forward velocity in m/s, clamped to Unitree's documented AGV range [-1.5, 1.5].",
      },
      vy: {
        type: "number",
        default: 0,
        description: "Accepted for API compatibility but ignored by the G1 AGV simulator path.",
      },
      vyaw: {
        type: "number",
        default: 0,
        description: "Yaw velocity in rad/s, clamped to Unitree's documented AGV range [-0.6, 0.6].",
      },
      vz: {
        type: "number",
        default: 0,
        description: "Height column velocity intent, clamped to [-1.0, 1.0].",
      },
    },
    ["command"],
    { readOnlyHint: false },
  ),
  tool(
    "g1_motion_switcher",
    "Execute a Unitree MotionSwitcherClient-shaped simulator command such as CheckMode, SelectMode, or ReleaseMode.",
    {
      command: {
        type: "string",
        enum: ["check_mode", "select_mode", "release_mode", "set_silent", "get_silent"],
        default: "check_mode",
      },
      name: {
        type: "string",
        default: "ai",
        description: "Motion mode name or alias for select_mode.",
      },
      silent: {
        type: "boolean",
        default: false,
        description: "Silent flag for set_silent.",
      },
    },
    ["command"],
    { readOnlyHint: false },
  ),
  tool("g1_lowstate", "Read simulator-backed Unitree rt/lowstate motor and IMU telemetry.", {}, [], {
    readOnlyHint: true,
  }),
  tool("g1_safety_check", "Evaluate Unitree G1-inspired lowstate safety checks before issuing more motion.", {}, [], {
    readOnlyHint: true,
  }),
  tool("g1_joint_state", "Read named G1 joint state with motor-index mapping and limits.", {}, [], {
    readOnlyHint: true,
  }),
  tool(
    "g1_apply_joint_targets",
    "Apply simulator-backed G1 joint targets by joint name.",
    {
      targets: {
        type: "object",
        description: "Object mapping joint names to target radians, for example {\"right_shoulder_pitch_joint\": -1.2}.",
      },
      kp: { type: "number", default: 38.0 },
      kd: { type: "number", default: 1.4 },
      tau: { type: "number", default: 0 },
      dq: { type: "number", default: 0 },
    },
    ["targets"],
    { readOnlyHint: false },
  ),
  tool(
    "g1_lowcmd",
    "Publish a simulator-backed Unitree rt/lowcmd motor command list.",
    {
      motor_cmd: {
        type: "array",
        description: "Array of motor commands with q, dq, tau, kp, kd, and mode fields.",
        items: { type: "object" },
      },
      topic: { type: "string", default: "rt/lowcmd" },
      mode_pr: { type: "integer", default: 0 },
      mode_machine: { type: "integer", default: 0 },
      crc: { type: "integer", default: 0 },
    },
    ["motor_cmd"],
    { readOnlyHint: false },
  ),
  tool(
    "g1_hand_sdk",
    "Publish a simulator-backed Unitree rt/hand_sdk command intent for opening or closing the G1 hand.",
    {
      tau: {
        type: "number",
        minimum: -1.5,
        maximum: 1.5,
        default: 0.3,
        description: "Positive closes the hand, negative opens it, matching Unitree's hand SDK example.",
      },
      weight: {
        type: "number",
        minimum: 0,
        maximum: 1,
        default: 1,
        description: "Blend weight encoded as weight*100 in cmds[0].mode by Unitree's example.",
      },
      motor_count: {
        type: "integer",
        minimum: 1,
        maximum: 12,
        default: 4,
        description: "Number of hand motor commands to publish; Unitree's simple hand SDK example uses 4.",
      },
    },
    [],
    { readOnlyHint: false },
  ),
  tool(
    "g1_dex3_command",
    "Publish a simulator-backed Unitree Dex3 HandCmd_ intent to rt/dex3/{left,right}/cmd.",
    {
      hand: {
        type: "string",
        enum: ["left", "right"],
        default: "right",
      },
      q: {
        type: "number",
        default: 0.25,
        description: "Target position for each of the seven Dex3 motors; clamped by simulator-side left/right URDF limits.",
      },
      kp: {
        type: "number",
        default: 1.5,
      },
      kd: {
        type: "number",
        default: 0.1,
      },
    },
    [],
    { readOnlyHint: false },
  ),
  tool("safety_stop", "Release motion mode, damp locomotion, neutralize the G1 pose, and pause the simulator.", {}, [], {
    readOnlyHint: false,
    destructiveHint: true,
  }),
  tool(
    "docker_logs",
    "Read recent logs from the Unitree G1 MuJoCo simulator container.",
    {
      tail: { type: "integer", minimum: 1, maximum: 1000, default: 120 },
    },
    [],
    { readOnlyHint: true, openWorldHint: true },
  ),
  tool(
    "protocol_probe_http",
    "Probe a simulator HTTP endpoint on GameControl.",
    {
      path: { type: "string", default: "/status" },
    },
    [],
    { readOnlyHint: true },
  ),
  tool(
    "protocol_probe_ws",
    "Subscribe once to a Booster-style physics WebSocket topic and summarize the frame.",
    {
      topic: {
        type: "string",
        enum: ["simulation_state", "visual_frame", "visual_scene", "camera_frame_0"],
        default: "simulation_state",
      },
      timeout_ms: { type: "integer", minimum: 100, maximum: 10000, default: 5000 },
    },
    [],
    { readOnlyHint: true },
  ),
  tool(
    "python_control_start",
    "Start a Python control script as a managed robotics job.",
    {
      script_path: { type: "string" },
      args: { type: "array", items: { type: "string" }, default: [] },
    },
    ["script_path"],
    { readOnlyHint: false, openWorldHint: true },
  ),
  tool(
    "python_control_run",
    "Run a Python control script to completion and return stdout/stderr.",
    {
      script_path: { type: "string" },
      args: { type: "array", items: { type: "string" }, default: [] },
      timeout_ms: { type: "integer", minimum: 1000, maximum: 120000, default: 30000 },
    },
    ["script_path"],
    { readOnlyHint: false, openWorldHint: true },
  ),
  tool(
    "python_control_stop",
    "Stop a managed Python control job.",
    {
      job_id: { type: "string" },
    },
    ["job_id"],
    { readOnlyHint: false, destructiveHint: true },
  ),
  tool(
    "python_control_pause",
    "Pause a managed Python control job with SIGSTOP.",
    {
      job_id: { type: "string" },
    },
    ["job_id"],
    { readOnlyHint: false },
  ),
  tool(
    "python_control_resume",
    "Resume a managed Python control job with SIGCONT.",
    {
      job_id: { type: "string" },
    },
    ["job_id"],
    { readOnlyHint: false },
  ),
  tool(
    "python_control_logs",
    "Read logs and state for a managed Python control job.",
    {
      job_id: { type: "string" },
    },
    ["job_id"],
    { readOnlyHint: true },
  ),
  tool("python_control_list", "List managed Python control jobs.", {}, [], { readOnlyHint: true }),
];

const prompts = [
  {
    name: "robotics-quickstart",
    title: "Robotics Quickstart",
    description: "Start a Cybernetic IDE robotics session with the G1 simulator and viewer.",
  },
  {
    name: "g1-raise-hand-demo",
    title: "G1 Raise Hand Demo",
    description: "Write and run a Unitree SDK-shaped Python script that raises the G1 hand.",
  },
];

let stdinBuffer = "";
process.stdin.setEncoding("utf8");
process.stdin.on("data", (chunk) => {
  stdinBuffer += chunk;
  while (stdinBuffer.includes("\n")) {
    const index = stdinBuffer.indexOf("\n");
    const line = stdinBuffer.slice(0, index).trim();
    stdinBuffer = stdinBuffer.slice(index + 1);
    if (line.length > 0) {
      void handleLine(line);
    }
  }
});

process.on("SIGTERM", () => {
  for (const job of jobs.values()) {
    if (job.status === "running" || job.status === "paused") {
      job.child.kill("SIGTERM");
    }
  }
  process.exit(0);
});

async function handleLine(line) {
  let message;
  try {
    message = JSON.parse(line);
  } catch (error) {
    respondError(null, -32700, `Invalid JSON: ${error.message}`);
    return;
  }

  if (message.id === undefined || message.id === null) {
    return;
  }

  try {
    const result = await handleRequest(message.method, message.params ?? {});
    respond(message.id, result);
  } catch (error) {
    respondError(message.id, -32603, error.stack || error.message || String(error));
  }
}

async function handleRequest(method, params) {
  switch (method) {
    case "initialize":
      return {
        protocolVersion: params.protocolVersion || PROTOCOL_VERSION,
        capabilities: {
          tools: { listChanged: false },
          prompts: { listChanged: false },
        },
        serverInfo: {
          name: "cyber-robotics-mcp",
          title: "Cybernetic Robotics",
          version: SERVER_VERSION,
          description: "Default robotics tools for Cybernetic IDE Unitree G1 and MuJoCo workflows.",
        },
      };
    case "ping":
      return {};
    case "tools/list":
      return { tools };
    case "tools/call":
      return callTool(params.name, params.arguments ?? {});
    case "prompts/list":
      return { prompts };
    case "prompts/get":
      return getPrompt(params.name);
    default:
      throw new Error(`Unsupported MCP method: ${method}`);
  }
}

async function callTool(name, args) {
  switch (name) {
    case "sim_prepare_runtime":
      return textResult(runChecked("node", ["script/prepare-unitree-g1-mujoco-container.mjs"], { timeoutMs: 180000 }));
    case "sim_start":
      return textResult(runChecked("docker", [...composeArgs(), "up", "-d"], { timeoutMs: 180000 }));
    case "sim_stop":
      return textResult(runChecked("docker", [...composeArgs(), "stop", DEFAULT_CONTAINER], { timeoutMs: 60000 }));
    case "sim_restart":
      return textResult(runChecked("docker", [...composeArgs(), "restart", DEFAULT_CONTAINER], { timeoutMs: 120000 }));
    case "sim_status":
      return textResult(await simStatus());
    case "unitree_session_status":
      return textResult(await unitreeSessionStatus());
    case "unitree_provider_status":
      return textResult(providerStatusFromDiagnostics(await unitreeSessionStatus()));
    case "unitree_sdk_compatibility_audit":
      return textResult(unitreeSdkCompatibilityAudit(args));
    case "unitree_sdk_behavior_smoke":
      return textResult(unitreeSdkBehaviorSmoke(args));
    case "unitree_prepare_sdk2_sidecar":
      return textResult(runChecked("node", ["script/prepare-unitree-g1-sdk2-sidecar.mjs"], { timeoutMs: 240000 }));
    case "unitree_sdk2_sidecar_status":
      return textResult(sdk2SidecarStatus());
    case "unitree_official_mujoco_plan":
      return textResult(sdk2OfficialMujocoPlan());
    case "unitree_build_official_mujoco_peer":
      return textResult(sdk2BuildOfficialMujocoPeer());
    case "unitree_probe_official_mujoco_launch":
      return textResult(sdk2ProbeOfficialMujocoLaunch());
    case "unitree_start_official_mujoco_session":
      return textResult(sdk2StartOfficialMujocoSession());
    case "unitree_official_mujoco_session_status":
      return textResult(sdk2OfficialMujocoSessionStatus());
    case "unitree_read_official_mujoco_lowstate":
      return textResult(sdk2ReadOfficialMujocoLowstate());
    case "unitree_probe_official_mujoco_loco_rpc":
      return textResult(sdk2ProbeOfficialMujocoLocoRpc(args));
    case "unitree_probe_official_mujoco_rpc_discovery":
      return textResult(sdk2ProbeOfficialMujocoRpcDiscovery(args));
    case "unitree_probe_rpc_bridge_smoke":
      return textResult(sdk2ProbeRpcBridgeSmoke(args));
    case "unitree_start_rpc_bridge":
      return textResult(sdk2StartRpcBridge());
    case "unitree_rpc_bridge_status":
      return textResult(sdk2RpcBridgeStatus());
    case "unitree_probe_rpc_bridge_client":
      return textResult(sdk2ProbeRpcBridgeClient(args));
    case "unitree_verify_rpc_bridge":
      return textResult(sdk2VerifyRpcBridge(args));
    case "unitree_command_rpc_bridge":
      return textResult(sdk2CommandRpcBridge(args));
    case "unitree_stop_rpc_bridge":
      return textResult(sdk2StopRpcBridge());
    case "unitree_stop_official_mujoco_session":
      return textResult(sdk2StopOfficialMujocoSession());
    case "unitree_probe_official_mujoco_dds":
      return textResult(sdk2ProbeOfficialMujocoDds());
    case "unitree_probe_official_mujoco_lowcmd":
      return textResult(sdk2ProbeOfficialMujocoLowcmd());
    case "unitree_probe_official_mujoco_arm_motion":
      return textResult(sdk2ProbeOfficialMujocoArmMotion(args));
    case "unitree_probe_official_mujoco_arm_pose":
      return textResult(sdk2ProbeOfficialMujocoArmPose(args));
    case "unitree_command_official_mujoco_arm_pose":
      return textResult(sdk2CommandOfficialMujocoArmPose(args));
    case "unitree_command_official_mujoco_lowcmd":
      return textResult(sdk2CommandOfficialMujocoLowcmd(args));
    case "unitree_official_mujoco_evidence_bundle":
      return textResult(await sdk2OfficialMujocoEvidenceBundle(args));
    case "sim_pause":
      return textResult(await command({ command: "pause" }));
    case "sim_resume":
      return textResult(await command({ command: "resume" }));
    case "sim_reset":
      return textResult(await command({ command: "reset" }));
    case "sim_step":
      return textResult(await repeatStep(toInt(args.count, 1)));
    case "sim_validate_behavior":
      return textResult(await validateBehavior(args));
    case "robot_evidence_bundle":
      return textResult(await robotEvidenceBundle(args));
    case "sim_apply_pose":
      return textResult(await command({ command: "pose", pose: args.pose || DEFAULT_POSE }));
    case "sim_policy_status":
      return textResult(await command({ command: "yoga_policy", action: "status" }));
    case "sim_policy_start":
      return textResult(await command({
        command: "yoga_policy",
        action: "start",
        loop: args.loop !== false,
        frame: toInt(args.frame, 0),
      }));
    case "sim_policy_stop":
      return textResult(await command({ command: "yoga_policy", action: "stop" }));
    case "viewer_camera_control":
      return textResult(await camera(args));
    case "viewer_camera_bookmark_save":
      return textResult(await saveCameraBookmark(args));
    case "viewer_camera_bookmark_list":
      return textResult(await listCameraBookmarks());
    case "viewer_camera_bookmark_apply":
      return textResult(await applyCameraBookmark(args));
    case "viewer_camera_bookmark_delete":
      return textResult(await deleteCameraBookmark(args));
    case "viewer_snapshot":
      return imageResult(await snapshot(args.format || "jpeg"));
    case "viewer_snapshot_file":
      return textResult(await snapshotFile(args.path, args.format || "jpeg"));
    case "viewer_snapshot_series":
      return textResult(await snapshotSeries(args));
    case "scene_get":
      return textResult(await getJson("/visual_scene"));
    case "scene_read_mjcf":
      return textResult(await readActiveMjcf());
    case "scene_validate_mjcf":
      return textResult(validateMjcf(args.model_path));
    case "scene_add_box":
      return textResult(await addBoxToScene(args));
    case "scene_add_object":
      return textResult(await addObjectToScene(args));
    case "scene_list_objects":
      return textResult(await listSceneObjects());
    case "scene_remove_object":
      return textResult(await removeObjectFromScene(args));
    case "unitree_sdk_scaffold_python":
      return textResult(await scaffoldPython(args));
    case "robotics_tool_reference":
      return textResult(roboticsToolReference());
    case "g1_list_actions":
      return textResult({
        actions: Object.entries(G1_ACTION_POSES).map(([action, value]) => ({ action, ...value })),
      });
    case "g1_execute_action":
      return textResult(await executeG1Action(args.action));
    case "g1_loco_command":
      return textResult(await executeG1LocoCommand(args));
    case "g1_agv_command":
      return textResult(await executeG1AgvCommand(args));
    case "g1_motion_switcher":
      return textResult(await executeG1MotionSwitcher(args));
    case "g1_lowstate":
      return textResult(await getJson("/lowstate"));
    case "g1_safety_check":
      return textResult(await safetyCheck());
    case "g1_joint_state":
      return textResult(await getJson("/joint_state"));
    case "g1_apply_joint_targets":
      return textResult(await executeG1JointTargets(args));
    case "g1_lowcmd":
      return textResult(await executeG1Lowcmd(args));
    case "g1_hand_sdk":
      return textResult(await executeG1HandSdk(args));
    case "g1_dex3_command":
      return textResult(await executeG1Dex3Command(args));
    case "safety_stop":
      return textResult(await safetyStop());
    case "docker_logs":
      return textResult(runChecked("docker", ["logs", "--tail", String(toInt(args.tail, 120)), DEFAULT_CONTAINER], { timeoutMs: 30000 }));
    case "protocol_probe_http":
      return textResult(await protocolProbeHttp(args.path || "/status"));
    case "protocol_probe_ws":
      return textResult(await protocolProbeWs(args.topic || "simulation_state", toInt(args.timeout_ms, 5000)));
    case "python_control_start":
      return textResult(startPythonJob(args.script_path, args.args || []));
    case "python_control_run":
      return textResult(runPythonControl(args.script_path, args.args || [], toInt(args.timeout_ms, 30000)));
    case "python_control_stop":
      return textResult(signalJob(args.job_id, "SIGTERM", "stopping"));
    case "python_control_pause":
      return textResult(signalJob(args.job_id, "SIGSTOP", "paused"));
    case "python_control_resume":
      return textResult(signalJob(args.job_id, "SIGCONT", "running"));
    case "python_control_logs":
      return textResult(jobSnapshot(args.job_id));
    case "python_control_list":
      return textResult({ jobs: Array.from(jobs.values()).map(publicJob) });
    default:
      return toolError(`Unknown robotics tool: ${name}`);
  }
}

function getPrompt(name) {
  if (name === "robotics-quickstart") {
    return {
      description: "Start a Cybernetic IDE robotics session.",
      messages: [
        {
          role: "user",
          content: {
            type: "text",
            text: "Check sim_status, prepare/start the simulator if needed, open or use the Robot Viewer, then explain what robotics tools are available.",
          },
        },
      ],
    };
  }

  if (name === "g1-raise-hand-demo") {
    return {
      description: "Create and run a G1 raise-hand SDK demo.",
      messages: [
        {
          role: "user",
          content: {
            type: "text",
            text: "Use the Unitree SDK facade to create a Python script that raises the Unitree G1 right hand in the local MuJoCo sim, run it, and capture a viewer_snapshot.",
          },
        },
      ],
    };
  }

  throw new Error(`Unknown prompt: ${name}`);
}

async function simStatus() {
  const status = await getJson("/status").catch((error) => ({ status: "unreachable", error: error.message }));
  const inspect = run("docker", ["inspect", DEFAULT_CONTAINER, "--format", "{{.State.Status}} {{.State.Running}}"], {
    timeoutMs: 10000,
  });
  const env = readComposeEnv();
  return {
    root,
    game_control_url: gameControlUrl(),
    physics_url: physicsUrl(),
    compose_env_exists: fs.existsSync(composeEnvPath()),
    docker: inspect.status === 0 ? inspect.stdout.trim() : inspect.stderr.trim(),
    env,
    status,
  };
}

function unitreeSdkCompatibilityAudit(args = {}) {
  const upstreamRoot = typeof args.upstream_root === "string" && args.upstream_root
    ? args.upstream_root
    : "/Users/cuboniks/wagmi/unitree_sdk2_python";
  const packageSrc = path.join(root, "packages/cybernetic-robotics/src");
  const env = {
    ...process.env,
    PYTHONPATH: process.env.PYTHONPATH ? `${packageSrc}${path.delimiter}${process.env.PYTHONPATH}` : packageSrc,
  };
  const result = runChecked(
    "python3",
    ["-m", "cybernetic_robotics.cli", "sdk-audit", "--upstream-root", upstreamRoot],
    { timeoutMs: 60_000, env },
  );
  let report = null;
  try {
    report = JSON.parse(result.stdout);
  } catch {
    report = null;
  }
  return {
    command: `python3 -m cybernetic_robotics.cli sdk-audit --upstream-root ${upstreamRoot}`,
    stdout: result.stdout,
    stderr: result.stderr,
    report,
  };
}

function unitreeSdkBehaviorSmoke(args = {}) {
  const kind = ["all", "arm", "loco", "lowcmd", "hand"].includes(args.kind) ? args.kind : "all";
  const transport = ["local_http", "rpc_bridge", "dds"].includes(args.transport) ? args.transport : null;
  const outputPath = typeof args.output_path === "string" && args.output_path
    ? args.output_path
    : ".runtime/sdk-smoke/latest.json";
  safeWorkspacePath(outputPath);
  const packageSrc = path.join(root, "packages/cybernetic-robotics/src");
  const env = {
    ...process.env,
    PYTHONPATH: process.env.PYTHONPATH ? `${packageSrc}${path.delimiter}${process.env.PYTHONPATH}` : packageSrc,
    CYBER_G1_GAME_CONTROL_URL: gameControlUrl(),
  };
  const commandArgs = ["-m", "cybernetic_robotics.cli", "sdk-smoke", "--kind", kind, "--output", outputPath];
  if (transport) {
    commandArgs.push("--transport", transport);
  }
  const result = runChecked("python3", commandArgs, { timeoutMs: 120_000, env });
  let report = null;
  try {
    report = JSON.parse(result.stdout);
  } catch {
    report = null;
  }
  return {
    command: `python3 ${commandArgs.join(" ")}`,
    stdout: result.stdout,
    stderr: result.stderr,
    report,
    path: safeWorkspacePath(outputPath),
    workspace_relative_path: outputPath,
  };
}

async function unitreeSessionStatus() {
  const mode = normalizeChoice(process.env.CYBER_UNITREE_MODE, ["sim", "real"], "sim");
  const transport = normalizeChoice(process.env.CYBER_UNITREE_TRANSPORT, ["local_http", "dds", "rpc_bridge"], "local_http");
  const ddsDomainId = toInt(process.env.CYBER_UNITREE_DDS_DOMAIN, mode === "sim" ? 1 : 0);
  const networkInterface = process.env.CYBER_UNITREE_NETWORK_INTERFACE || (mode === "sim" ? "lo" : null);
  const warnings = [];
  const result = {
    ok: true,
    implemented: transport === "local_http",
    config: {
      mode,
      transport,
      dds_domain_id: ddsDomainId,
      network_interface: networkInterface,
      safety_profile: process.env.CYBER_UNITREE_SAFETY_PROFILE || (mode === "sim" ? "simulator" : "real"),
      real_unlocked: process.env.CYBER_UNITREE_REAL_UNLOCK === "I_UNDERSTAND_THIS_CONTROLS_REAL_HARDWARE",
      endpoints: {
        game_control_url: gameControlUrl(),
        physics_url: physicsUrl(),
      },
    },
    warnings,
    simulator: null,
    official_sidecar: null,
    topics: {},
  };

  let officialStatus = null;
  if ((transport === "dds" || transport === "rpc_bridge") && mode === "sim") {
    try {
      officialStatus = sdk2SidecarStatus();
      result.official_sidecar = summarizeOfficialSidecarStatus(officialStatus.report);
      if (result.official_sidecar.ok) {
        result.implemented = true;
      } else {
        warnings.push("official SDK2 sidecar status did not pass");
      }
    } catch (error) {
      warnings.push(`official SDK2 sidecar unavailable: ${error.message}`);
      result.official_sidecar = { ok: false, error: error.message };
    }
  } else if (transport === "dds" || transport === "rpc_bridge") {
    warnings.push(`${transport} transport is selected outside simulator mode; real hardware control still requires a long-lived official provider`);
  }
  if (mode === "real") {
    if (!networkInterface) warnings.push("real mode requires CYBER_UNITREE_NETWORK_INTERFACE");
    if (!result.config.real_unlocked) warnings.push("real mode is locked; do not send hardware commands until explicitly unlocked");
  }

  try {
    const statusValue = await getJson("/status");
    const simulation = statusValue.simulation && typeof statusValue.simulation === "object" ? statusValue.simulation : {};
    const lowcmd = simulation.lowcmd && typeof simulation.lowcmd === "object" ? simulation.lowcmd : {};
    result.simulator = {
      reachable: true,
      ready: statusValue.ready === true,
      pose: simulation.pose || null,
      paused: simulation.paused === true,
      fallen: simulation.fallen === true,
      model_path: simulation.model_path || null,
    };
    result.topics["rt/lowcmd"] = {
      source: transport,
      active: lowcmd.active === true,
      stale: lowcmd.stale === true,
      age_seconds: lowcmd.age_seconds ?? null,
      watchdog_seconds: lowcmd.watchdog_seconds ?? null,
      last_received_at: lowcmd.received_at ?? null,
      message_count: lowcmd.motor_cmd_count ?? null,
    };
  } catch (error) {
    if (transport !== "dds") {
      result.ok = false;
    }
    result.simulator = { reachable: false, error: error.message };
  }

  try {
    const lowstate = await getJson("/lowstate");
    const lowcmd = lowstate.lowcmd && typeof lowstate.lowcmd === "object" ? lowstate.lowcmd : {};
    result.topics["rt/lowstate"] = {
      source: transport,
      available: true,
      motor_count: Array.isArray(lowstate.motor_state) ? lowstate.motor_state.length : 0,
      mode_machine: lowstate.mode_machine ?? null,
      mode_pr: lowstate.mode_pr ?? null,
      crc: lowstate.crc ?? null,
      lowcmd_active: lowcmd.active === true,
      lowcmd_stale: lowcmd.stale === true,
    };
  } catch (error) {
    result.topics["rt/lowstate"] = { available: false, error: error.message };
  }

  if (officialStatus?.report?.sdk2_probe && typeof officialStatus.report.sdk2_probe === "object") {
    const channels = officialStatus.report.sdk2_probe.channels && typeof officialStatus.report.sdk2_probe.channels === "object"
      ? officialStatus.report.sdk2_probe.channels
      : {};
    const lowcmd = channels["rt/lowcmd"];
    if (lowcmd && typeof lowcmd === "object") {
      result.topics["rt/lowcmd"] = {
        ...(result.topics["rt/lowcmd"] || {}),
        source: "official_sdk2_sidecar",
        created: lowcmd.created === true,
        role: lowcmd.role || null,
        sample_motor_count: lowcmd.sample_motor_count ?? null,
      };
    }
    const lowstate = channels["rt/lowstate"];
    if (lowstate && typeof lowstate === "object") {
      result.topics["rt/lowstate"] = {
        ...(result.topics["rt/lowstate"] || {}),
        source: "official_sdk2_sidecar",
        created: lowstate.created === true,
        role: lowstate.role || null,
      };
    }
  }

  result.ok = Boolean(result.ok && !warnings.some((warning) => warning.includes("requires")));
  return result;
}

function providerStatusFromDiagnostics(diagnostics) {
  const config = diagnostics.config || {};
  const transport = config.transport || "local_http";
  const mode = config.mode || "sim";
  const officialOk = diagnostics.official_sidecar?.ok === true;
  const simulatorReachable = diagnostics.simulator?.reachable === true;

  if (transport === "local_http" && mode === "sim") {
    return {
      ok: diagnostics.ok === true && simulatorReachable,
      provider: "local_http_simulator",
      implemented: simulatorReachable,
      command_path: "Cybernetic GameControl HTTP commands plus the Booster-style physics WebSocket.",
      telemetry_path: "Simulator HTTP /status, /lowstate, /joint_state, and rendered camera frames.",
      motion: {
        arm_actions: "simulator_named_poses",
        locomotion: "kinematic_base_velocity",
        lowcmd: "simulator_joint_targets",
      },
      limitations: [
        "No CycloneDDS transport is used.",
        "Locomotion is a local approximation, not Unitree's whole-body balance controller.",
      ],
      next_step: "Use CYBER_UNITREE_TRANSPORT=rpc_bridge for high-level sport/agv/arm tests, or dds for official lowcmd/lowstate sidecar probes.",
      config,
      warnings: diagnostics.warnings || [],
      diagnostics_summary: providerDiagnosticsSummary(diagnostics, simulatorReachable, officialOk),
    };
  }

  if (transport === "dds" && mode === "sim") {
    return {
      ok: diagnostics.ok === true && officialOk,
      provider: officialOk ? "official_mujoco_dds_simulator" : "official_mujoco_dds_simulator_unready",
      implemented: officialOk,
      command_path: "Official SDK2/CycloneDDS sidecar for supported arm poses; local HTTP remains the fallback for viewer and local loco tools.",
      telemetry_path: "Official sidecar rt/lowstate reads and bounded rt/lowcmd writes plus local simulator diagnostics when available.",
      motion: {
        arm_actions: officialOk ? "managed_official_mujoco_session_for_supported_poses" : "unavailable_until_sidecar_ready",
        locomotion: "local_http_compatibility_until_dds_loco_provider_lands",
        lowcmd: officialOk ? "managed_official_mujoco_session_bounded_frame" : "unavailable_until_sidecar_ready",
      },
      limitations: [
        "Only bounded arm-pose commands and one-frame generic lowcmd writes are routed through the managed official DDS session today.",
        "LocoClient locomotion and sustained lowcmd streaming still need the long-lived DDS provider.",
      ],
      next_step: "Start or inspect the managed official MuJoCo session, then promote loco and sustained lowcmd streaming to that provider.",
      config,
      warnings: diagnostics.warnings || [],
      diagnostics_summary: providerDiagnosticsSummary(diagnostics, simulatorReachable, officialOk),
    };
  }

  if (transport === "rpc_bridge" && mode === "sim") {
    return {
      ok: diagnostics.ok === true && officialOk,
      provider: officialOk ? "unitree_rpc_bridge_simulator" : "unitree_rpc_bridge_simulator_unready",
      implemented: officialOk,
      command_path: "Official SDK2-shaped sport/agv/arm RPC bridge backed by the local simulator provider.",
      telemetry_path: "Bridge command evidence plus simulator HTTP /status, /lowstate, /joint_state, and rendered camera frames.",
      motion: {
        arm_actions: "managed_unitree_rpc_bridge_arm_service",
        locomotion: "managed_unitree_rpc_bridge_sport_agv",
        lowcmd: "local_http_simulator_until_generic_dds_streaming_lands",
      },
      limitations: [
        "This is a simulator-side service bridge, not physical robot DDS control.",
        "Only the mapped sport/agv/arm RPC subset is available; generic lowcmd streaming remains separate.",
      ],
      next_step: "Use CYBER_UNITREE_TRANSPORT=rpc_bridge for high-level LocoClient/AgvClient tests, then promote lowcmd streaming separately.",
      config,
      warnings: diagnostics.warnings || [],
      diagnostics_summary: providerDiagnosticsSummary(diagnostics, simulatorReachable, officialOk),
    };
  }

  return {
    ok: false,
    provider: "real_unitree_dds",
    implemented: false,
    command_path: "Not enabled: real hardware requires an explicit provider, interface, unlock, and safety model.",
    telemetry_path: "Not enabled until real-mode DDS safety gates are implemented.",
    motion: { arm_actions: "disabled", locomotion: "disabled", lowcmd: "disabled" },
    limitations: [
      "Real hardware control is intentionally locked.",
      "Set CYBER_UNITREE_NETWORK_INTERFACE and the real unlock only after the real provider is implemented and reviewed.",
    ],
    next_step: "Finish the simulator DDS provider and safety gates before enabling physical robot control.",
    config,
    warnings: diagnostics.warnings || [],
    diagnostics_summary: providerDiagnosticsSummary(diagnostics, simulatorReachable, officialOk),
  };
}

function providerDiagnosticsSummary(diagnostics, simulatorReachable, officialOk) {
  return {
    simulator_reachable: simulatorReachable,
    official_sidecar_ok: officialOk,
    topics: diagnostics.topics || {},
  };
}

function summarizeOfficialSidecarStatus(report) {
  const sdk2Probe = report?.sdk2_probe && typeof report.sdk2_probe === "object" ? report.sdk2_probe : {};
  const peer = report?.official_mujoco_peer && typeof report.official_mujoco_peer === "object" ? report.official_mujoco_peer : {};
  const channels = sdk2Probe.channels && typeof sdk2Probe.channels === "object" ? sdk2Probe.channels : {};
  return {
    ok: sdk2Probe.domain_initialized === true,
    source: "official_unitree_mujoco_sdk2_sidecar",
    domain_initialized: sdk2Probe.domain_initialized === true,
    dds_domain_id: sdk2Probe.domain ?? null,
    network_interface: sdk2Probe.network_interface ?? null,
    lowcmd_channel_created: channels["rt/lowcmd"]?.created === true,
    lowstate_channel_created: channels["rt/lowstate"]?.created === true,
    official_mujoco_binary_exists: peer.binary_exists === true,
    official_mujoco_scene_exists: peer.scene_exists === true,
    expected_topics: Array.isArray(report?.expected_topics) ? report.expected_topics : [],
    next_step: report?.next_step ?? null,
  };
}

async function repeatStep(count) {
  const results = [];
  for (let index = 0; index < Math.max(1, count); index += 1) {
    results.push(await command({ command: "step" }));
  }
  return { count: results.length, last: results.at(-1), results };
}

async function validateBehavior(args) {
  const maxLowcmdAge = Number(args.max_lowcmd_age_seconds ?? 5.0);
  const requireSnapshot = args.require_snapshot !== false;
  const snapshotPath = args.snapshot_path || ".runtime/behavior-validation/latest.jpg";
  const checks = [];
  const status = await getJson("/status");
  const simulation = status.simulation && typeof status.simulation === "object" ? status.simulation : {};
  const lowstate = await getJson("/lowstate").catch((error) => ({ error: error.message }));
  const render = simulation.render && typeof simulation.render === "object" ? simulation.render : {};
  const lowcmd = simulation.lowcmd && typeof simulation.lowcmd === "object" ? simulation.lowcmd : null;

  addCheck(checks, "simulator_ready", status.ready === true, status.ready === true ? "simulator reports ready" : "simulator is not ready");
  addCheck(checks, "robot_not_fallen", simulation.fallen !== true, simulation.fallen === true ? "robot is fallen" : "robot is upright");
  addCheck(checks, "render_ok", !render.last_error, render.last_error ? `render error: ${render.last_error}` : "render cache has no error");
  addCheck(
    checks,
    "lowstate_available",
    !lowstate.error && Array.isArray(lowstate.motor_state),
    lowstate.error ? `lowstate unavailable: ${lowstate.error}` : `lowstate motors=${lowstate.motor_state.length}`,
  );

  const controlMode = String(simulation.control_mode || simulation.pose || "");
  if (controlMode === "lowcmd" || lowcmd) {
    const age = lowcmd?.received_at ? Date.now() / 1000 - Number(lowcmd.received_at) : Number.POSITIVE_INFINITY;
    addCheck(
      checks,
      "lowcmd_fresh",
      lowcmd.stale !== true && Number.isFinite(age) && age <= maxLowcmdAge,
      Number.isFinite(age)
        ? `last lowcmd age ${age.toFixed(3)}s, max ${maxLowcmdAge}s, stale=${lowcmd.stale === true}`
        : "no lowcmd timestamp available",
      {
        age_seconds: Number.isFinite(age) ? age : null,
        max_age_seconds: maxLowcmdAge,
        active: lowcmd.active === true,
        stale: lowcmd.stale === true,
        watchdog_seconds: lowcmd.watchdog_seconds ?? null,
      },
    );
    addCheck(
      checks,
      "lowcmd_applied_targets",
      Number(lowcmd?.applied_position_targets || 0) > 0 || Number(lowcmd?.motor_cmd_count || 0) > 0,
      `lowcmd motor_cmd_count=${lowcmd?.motor_cmd_count ?? 0}, applied_position_targets=${lowcmd?.applied_position_targets ?? 0}`,
    );
  }

  let snapshotValue = null;
  if (requireSnapshot) {
    try {
      snapshotValue = await snapshotFile(snapshotPath, "jpeg");
      addCheck(checks, "snapshot_available", snapshotValue.bytes > 0, `snapshot bytes=${snapshotValue.bytes}`, {
        path: snapshotValue.path,
        workspace_relative_path: snapshotValue.workspace_relative_path,
        bytes: snapshotValue.bytes,
      });
    } catch (error) {
      addCheck(checks, "snapshot_available", false, `snapshot failed: ${error.message}`);
    }
  }

  const ok = checks.every((check) => check.ok);
  return {
    ok,
    checks,
    summary: {
      ready: status.ready === true,
      fallen: simulation.fallen === true,
      control_mode: simulation.control_mode || null,
      pose: simulation.pose || null,
      paused: simulation.paused === true,
      pelvis_height: simulation.pelvis_height ?? null,
      mode_machine: lowstate.mode_machine ?? null,
      mode_pr: lowstate.mode_pr ?? null,
      crc: lowstate.crc ?? null,
      model_path: simulation.model_path || null,
      render_seq: render.render_seq ?? null,
      lowcmd_active: lowcmd?.active === true,
      lowcmd_stale: lowcmd?.stale === true,
      lowcmd_watchdog_seconds: lowcmd?.watchdog_seconds ?? null,
    },
    snapshot: snapshotValue,
    status,
    lowstate,
  };
}

async function robotEvidenceBundle(args = {}) {
  const manifestPath = safeWorkspacePath(args.output_path || ".runtime/robot-evidence/latest.json");
  const baseDir = path.dirname(manifestPath);
  const label = safeSegment(args.label || path.basename(manifestPath, path.extname(manifestPath)) || "latest");
  const format = args.snapshot_format === "png" ? "png" : "jpeg";
  const extension = format === "png" ? "png" : "jpg";
  const includeSnapshot = args.include_snapshot !== false;
  const includeSnapshotSeries = args.include_snapshot_series === true;
  await fsp.mkdir(baseDir, { recursive: true });

  const [statusResult, lowstateResult, jointStateResult, sceneResult, providerResult] = await Promise.allSettled([
    getJson("/status"),
    getJson("/lowstate"),
    getJson("/joint_state"),
    getJson("/visual_scene"),
    unitreeSessionStatus().then((diagnostics) => providerStatusFromDiagnostics(diagnostics)),
  ]);

  const status = settledValue(statusResult);
  const lowstate = settledValue(lowstateResult);
  const jointState = settledValue(jointStateResult);
  const scene = settledValue(sceneResult);
  const providerStatus = settledValue(providerResult);
  const checks = [];
  const simulation = status?.simulation && typeof status.simulation === "object" ? status.simulation : {};
  const render = simulation?.render && typeof simulation.render === "object" ? simulation.render : {};

  addCheck(
    checks,
    "status_available",
    !status.error && status.ready === true,
    status.error ? `status unavailable: ${status.error}` : `simulator ready=${status.ready === true}`,
  );
  addCheck(
    checks,
    "lowstate_available",
    !lowstate.error && Array.isArray(lowstate.motor_state),
    lowstate.error ? `lowstate unavailable: ${lowstate.error}` : `lowstate motors=${lowstate.motor_state.length}`,
  );
  addCheck(
    checks,
    "joint_state_available",
    !jointState.error && Array.isArray(jointState.joints),
    jointState.error ? `joint_state unavailable: ${jointState.error}` : `joint_state joints=${jointState.joints?.length ?? 0}`,
  );
  addCheck(
    checks,
    "scene_available",
    !scene.error,
    scene.error ? `visual_scene unavailable: ${scene.error}` : "visual scene metadata captured",
  );
  addCheck(
    checks,
    "robot_not_fallen",
    simulation.fallen !== true,
    simulation.fallen === true ? "robot is fallen" : "robot is not reporting fallen",
  );
  addCheck(
    checks,
    "render_ok",
    !render.last_error,
    render.last_error ? `render error: ${render.last_error}` : "render cache has no error",
  );

  let snapshotValue = null;
  if (includeSnapshot) {
    try {
      const snapshotPath = path.relative(root, path.join(baseDir, `${label}.${extension}`));
      snapshotValue = await snapshotFile(snapshotPath, format);
      addCheck(checks, "snapshot_available", snapshotValue.bytes > 0, `snapshot bytes=${snapshotValue.bytes}`, {
        path: snapshotValue.path,
        workspace_relative_path: snapshotValue.workspace_relative_path,
        bytes: snapshotValue.bytes,
      });
    } catch (error) {
      snapshotValue = { error: error.message };
      addCheck(checks, "snapshot_available", false, `snapshot failed: ${error.message}`);
    }
  }

  let snapshotSeriesValue = null;
  if (includeSnapshotSeries) {
    try {
      snapshotSeriesValue = await snapshotSeries({
        output_dir: path.relative(root, path.join(baseDir, `${label}-views`)),
        prefix: label,
        format,
      });
      const viewCount = Array.isArray(snapshotSeriesValue.views) ? snapshotSeriesValue.views.length : 0;
      addCheck(checks, "snapshot_series_available", viewCount > 0, `snapshot series views=${viewCount}`, {
        workspace_relative_output_dir: snapshotSeriesValue.workspace_relative_output_dir,
        views: viewCount,
      });
    } catch (error) {
      snapshotSeriesValue = { error: error.message };
      addCheck(checks, "snapshot_series_available", false, `snapshot series failed: ${error.message}`);
    }
  }

  const manifest = {
    ok: checks.every((check) => check.ok),
    label,
    captured_at: new Date().toISOString(),
    output_path: manifestPath,
    workspace_relative_output_path: path.relative(root, manifestPath),
    checks,
    summary: {
      ready: status.ready === true,
      fallen: simulation.fallen === true,
      paused: simulation.paused === true,
      control_mode: simulation.control_mode || null,
      pose: simulation.pose || null,
      robot_mode: simulation.robot_mode || lowstate.mode_machine || null,
      model_path: simulation.model_path || null,
      motor_count: Array.isArray(lowstate.motor_state) ? lowstate.motor_state.length : null,
      joint_count: Array.isArray(jointState.joints) ? jointState.joints.length : null,
      render_seq: render.render_seq ?? null,
      provider: providerStatus.provider || null,
      provider_ok: providerStatus.ok === true,
    },
    simulator: status,
    lowstate,
    joint_state: jointState,
    visual_scene: scene,
    provider_status: providerStatus,
    snapshot: snapshotValue,
    snapshot_series: snapshotSeriesValue,
    agent_hints: [
      "Use lowstate and joint_state as telemetry evidence; screenshots are visual evidence.",
      "Use checks to decide whether a behavior is reviewable before explaining success to the user.",
      "Use provider_status to distinguish local HTTP shim, rpc_bridge, DDS sidecar, and unsupported real-hardware paths.",
    ],
  };
  await fsp.writeFile(manifestPath, `${JSON.stringify(manifest, null, 2)}\n`);
  return manifest;
}

function settledValue(result) {
  if (result.status === "fulfilled") {
    return result.value;
  }
  return { error: result.reason?.message || String(result.reason) };
}

function addCheck(checks, name, ok, message, extra = {}) {
  checks.push({ name, ok: Boolean(ok), message, ...extra });
}

async function safetyCheck() {
  const [status, lowstate] = await Promise.all([getJson("/status"), getJson("/lowstate")]);
  const simulation = status?.simulation && typeof status.simulation === "object" ? status.simulation : {};
  const imu = lowstate?.imu_state && typeof lowstate.imu_state === "object" ? lowstate.imu_state : {};
  const motors = Array.isArray(lowstate?.motor_state) ? lowstate.motor_state : [];
  const lowcmd = lowstate?.lowcmd && typeof lowstate.lowcmd === "object" ? lowstate.lowcmd : {};
  const limits = {
    orientation_limit_rad: 1.0,
    joint_velocity_limit: 10.0,
    angular_velocity_limit: 6.0,
    motor_casing_temp_limit: 85.0,
    motor_winding_temp_limit: 120.0,
  };
  const checks = [];
  const orientationAngle = orientationAngleFromQuaternion(imu.quaternion);
  addCheck(
    checks,
    "bad_orientation",
    orientationAngle === null || orientationAngle <= limits.orientation_limit_rad,
    orientationAngle === null
      ? "orientation unavailable"
      : `orientation angle ${orientationAngle.toFixed(3)} rad within ${limits.orientation_limit_rad.toFixed(3)} rad`,
    { value: orientationAngle, limit: limits.orientation_limit_rad, source: "unitree_g1_terminations.bad_orientation" },
  );
  const maxJointVelocity = maxAbsMotorField(motors, "dq");
  addCheck(
    checks,
    "joint_vel_out_of_limit",
    maxJointVelocity === null || maxJointVelocity <= limits.joint_velocity_limit,
    maxJointVelocity === null
      ? "joint velocities unavailable"
      : `max joint velocity ${maxJointVelocity.toFixed(3)} within ${limits.joint_velocity_limit.toFixed(3)}`,
    { value: maxJointVelocity, limit: limits.joint_velocity_limit, source: "unitree_g1_terminations.joint_vel_out_of_limit" },
  );
  const maxAngularVelocity = maxAbsArray(imu.gyroscope);
  addCheck(
    checks,
    "ang_vel_out_of_limit",
    maxAngularVelocity === null || maxAngularVelocity <= limits.angular_velocity_limit,
    maxAngularVelocity === null
      ? "angular velocity unavailable"
      : `max angular velocity ${maxAngularVelocity.toFixed(3)} within ${limits.angular_velocity_limit.toFixed(3)}`,
    { value: maxAngularVelocity, limit: limits.angular_velocity_limit, source: "unitree_g1_terminations.ang_vel_out_of_limit" },
  );
  const maxCasingTemp = maxTemperature(motors, 0);
  addCheck(
    checks,
    "motor_casing_overheat",
    maxCasingTemp === null || maxCasingTemp <= limits.motor_casing_temp_limit,
    maxCasingTemp === null
      ? "motor casing temperatures unavailable"
      : `max casing temperature ${maxCasingTemp.toFixed(1)}C within ${limits.motor_casing_temp_limit.toFixed(1)}C`,
    { value: maxCasingTemp, limit: limits.motor_casing_temp_limit, source: "unitree_g1_terminations.motor_casing_overheat" },
  );
  const maxWindingTemp = maxTemperature(motors, 1);
  addCheck(
    checks,
    "motor_winding_overheat",
    maxWindingTemp === null || maxWindingTemp <= limits.motor_winding_temp_limit,
    maxWindingTemp === null
      ? "motor winding temperatures unavailable"
      : `max winding temperature ${maxWindingTemp.toFixed(1)}C within ${limits.motor_winding_temp_limit.toFixed(1)}C`,
    { value: maxWindingTemp, limit: limits.motor_winding_temp_limit, source: "unitree_g1_terminations.motor_winding_overheat" },
  );
  addCheck(
    checks,
    "lowcmd_stale",
    lowcmd.stale !== true,
    lowcmd.stale === true ? "lowcmd watchdog reports stale command" : "lowcmd watchdog is not stale",
    { value: lowcmd.stale === true, source: "cybernetic_simulator.lowcmd_watchdog" },
  );
  addCheck(
    checks,
    "fallen",
    simulation.fallen !== true,
    simulation.fallen === true ? "simulator reports robot fallen" : "simulator does not report a fall",
    { value: simulation.fallen === true, source: "cybernetic_simulator.status" },
  );
  const failedChecks = checks.filter((check) => !check.ok);
  return {
    ok: failedChecks.length === 0,
    safe_to_command: failedChecks.length === 0,
    source: "unitree_g1_terminations_json_lowstate",
    checked_at: new Date().toISOString(),
    limits,
    checks,
    failed_checks: failedChecks,
    recommendation: failedChecks.length === 0 ? "continue" : "call safety_stop before issuing more motion",
  };
}

function orientationAngleFromQuaternion(value) {
  if (!Array.isArray(value) || value.length < 4) {
    return null;
  }
  let [w, x, y, z] = value.slice(0, 4).map(Number);
  if (![w, x, y, z].every(Number.isFinite)) {
    return null;
  }
  const norm = Math.sqrt(w * w + x * x + y * y + z * z);
  if (norm <= 0) {
    return null;
  }
  w /= norm;
  x /= norm;
  y /= norm;
  z /= norm;
  const projectedZ = Math.max(-1, Math.min(1, -1 + 2 * (x * x + y * y)));
  return Math.acos(Math.max(-1, Math.min(1, -projectedZ)));
}

function maxAbsMotorField(motors, field) {
  const values = motors
    .map((motor) => Number(motor?.[field]))
    .filter(Number.isFinite)
    .map(Math.abs);
  return values.length > 0 ? Math.max(...values) : null;
}

function maxAbsArray(value) {
  if (!Array.isArray(value)) {
    return null;
  }
  const values = value.map(Number).filter(Number.isFinite).map(Math.abs);
  return values.length > 0 ? Math.max(...values) : null;
}

function maxTemperature(motors, index) {
  const values = [];
  for (const motor of motors) {
    const temperature = motor?.temperature;
    if (Array.isArray(temperature) && temperature.length > index) {
      const value = Number(temperature[index]);
      if (Number.isFinite(value)) {
        values.push(value);
      }
    }
  }
  return values.length > 0 ? Math.max(...values) : null;
}

async function command(body) {
  return postJson("/command", body);
}

async function camera(args) {
  const payload = {
    action: args.action || "state",
    dx: Number(args.dx || 0),
    dy: Number(args.dy || 0),
    delta: Number(args.delta || 0),
  };
  if (payload.action === "set") {
    const cameraState = normalizeCameraState(args);
    Object.assign(payload, cameraState);
  }
  return postJson("/camera", payload);
}

async function saveCameraBookmark(args) {
  const name = safeBookmarkName(args.name || "default");
  const response = await camera({ action: "state" });
  const state = normalizeCameraState(response.camera || response);
  const bookmarks = await readCameraBookmarks();
  bookmarks[name] = {
    name,
    description: String(args.description || ""),
    camera: state,
    saved_at: new Date().toISOString(),
  };
  await writeCameraBookmarks(bookmarks);
  return {
    path: cameraBookmarksPath(),
    workspace_relative_path: CAMERA_BOOKMARKS_PATH,
    bookmark: bookmarks[name],
  };
}

async function listCameraBookmarks() {
  const bookmarks = await readCameraBookmarks();
  return {
    path: cameraBookmarksPath(),
    workspace_relative_path: CAMERA_BOOKMARKS_PATH,
    count: Object.keys(bookmarks).length,
    bookmarks,
  };
}

async function applyCameraBookmark(args) {
  const name = safeBookmarkName(args.name || "default");
  const bookmarks = await readCameraBookmarks();
  const bookmark = bookmarks[name];
  if (!bookmark) {
    throw new Error(`Unknown camera bookmark: ${name}`);
  }
  const result = await camera({ action: "set", ...bookmark.camera });
  return {
    bookmark,
    applied: result,
  };
}

async function deleteCameraBookmark(args) {
  const name = safeBookmarkName(args.name || "default");
  const bookmarks = await readCameraBookmarks();
  const existed = Object.hasOwn(bookmarks, name);
  if (existed) {
    delete bookmarks[name];
    await writeCameraBookmarks(bookmarks);
  }
  return {
    name,
    deleted: existed,
    remaining: Object.keys(bookmarks).length,
    path: cameraBookmarksPath(),
    workspace_relative_path: CAMERA_BOOKMARKS_PATH,
  };
}

function cameraBookmarksPath() {
  return safeWorkspacePath(CAMERA_BOOKMARKS_PATH);
}

async function readCameraBookmarks() {
  const filePath = cameraBookmarksPath();
  try {
    const parsed = JSON.parse(await fsp.readFile(filePath, "utf8"));
    if (parsed && typeof parsed === "object" && !Array.isArray(parsed.bookmarks)) {
      return parsed.bookmarks && typeof parsed.bookmarks === "object" ? parsed.bookmarks : {};
    }
  } catch (error) {
    if (error.code !== "ENOENT") {
      throw error;
    }
  }
  return {};
}

async function writeCameraBookmarks(bookmarks) {
  const filePath = cameraBookmarksPath();
  await fsp.mkdir(path.dirname(filePath), { recursive: true });
  await fsp.writeFile(filePath, `${JSON.stringify({ version: 1, bookmarks }, null, 2)}\n`);
}

function normalizeCameraState(value) {
  if (!value || typeof value !== "object") {
    throw new Error("Camera state must be an object");
  }
  const lookat = Array.isArray(value.lookat) ? value.lookat.map(Number) : [];
  if (lookat.length !== 3 || lookat.some((number) => !Number.isFinite(number))) {
    throw new Error("Camera lookat must contain three finite numbers");
  }
  const distance = clampNumber(value.distance, 0.45, 12.0, 2.7);
  const azimuth = Number(value.azimuth ?? -90.0);
  const elevation = clampNumber(value.elevation, -89.0, 20.0, -8.0);
  if (!Number.isFinite(azimuth)) {
    throw new Error("Camera azimuth must be finite");
  }
  return {
    lookat,
    distance,
    azimuth,
    elevation,
  };
}

function safeBookmarkName(value) {
  const name = safeSegment(String(value || "default"));
  if (!name) {
    throw new Error("Bookmark name must not be empty");
  }
  return name;
}

async function snapshot(format) {
  const normalized = format === "png" ? "png" : "jpeg";
  const path = normalized === "png" ? "/camera_frame_0.png" : "/camera_frame_0.jpg";
  const response = await fetch(`${gameControlUrl()}${path}`);
  if (!response.ok) {
    throw new Error(`Snapshot failed: HTTP ${response.status} ${await response.text()}`);
  }
  const bytes = Buffer.from(await response.arrayBuffer());
  const status = await getJson("/status").catch(() => null);
  return {
    data: bytes.toString("base64"),
    mimeType: normalized === "png" ? "image/png" : "image/jpeg",
    metadata: {
      bytes: bytes.length,
      format: normalized,
      render: status?.simulation?.render,
      pose: status?.simulation?.pose,
      paused: status?.simulation?.paused,
    },
  };
}

async function snapshotFile(userPath, format) {
  const snapshotValue = await snapshot(format);
  const outputPath = safeWorkspacePath(userPath);
  await fsp.mkdir(path.dirname(outputPath), { recursive: true });
  await fsp.writeFile(outputPath, Buffer.from(snapshotValue.data, "base64"));
  return {
    ...snapshotValue.metadata,
    path: outputPath,
    workspace_relative_path: path.relative(root, outputPath),
  };
}

async function snapshotSeries(args) {
  const format = args.format === "png" ? "png" : "jpeg";
  const extension = format === "png" ? "png" : "jpg";
  const outputDir = safeWorkspacePath(args.output_dir || ".runtime/robot-viewer-snapshots");
  const prefix = safeSegment(args.prefix || "g1");
  const requestedViews = Array.isArray(args.views) && args.views.length > 0 ? args.views : ["current", "front", "right", "three_quarter"];
  const views = requestedViews.map((view) => {
    const key = String(view);
    if (!VIEW_PRESETS[key]) {
      throw new Error(`Unsupported snapshot view: ${key}`);
    }
    return key;
  });
  const stamp = new Date().toISOString().replaceAll(":", "-").replaceAll(".", "-");
  const seriesDir = path.join(outputDir, `${prefix}-${stamp}`);
  await fsp.mkdir(seriesDir, { recursive: true });

  const captures = [];
  for (const view of views) {
    const preset = VIEW_PRESETS[view];
    if (preset.commands.length > 0) {
      for (const cameraCommand of preset.commands) {
        await camera(cameraCommand);
      }
    }
    const filePath = path.join(seriesDir, `${captures.length + 1}-${view}.${extension}`);
    const capture = await snapshotFile(path.relative(root, filePath), format);
    captures.push({
      view,
      description: preset.description,
      path: capture.path,
      workspace_relative_path: capture.workspace_relative_path,
      bytes: capture.bytes,
      render: capture.render,
      pose: capture.pose,
      paused: capture.paused,
    });
  }

  const manifest = {
    captured_at: new Date().toISOString(),
    output_dir: seriesDir,
    workspace_relative_output_dir: path.relative(root, seriesDir),
    format,
    views: captures,
    status: await getJson("/status").catch((error) => ({ error: error.message })),
  };
  await fsp.writeFile(path.join(seriesDir, "manifest.json"), `${JSON.stringify(manifest, null, 2)}\n`);
  return manifest;
}

async function executeG1LocoCommand(args) {
  const commandName = args.command || "state";
  if (commandName === "state") {
    return command({ command: "loco", action: "state" });
  }
  if (commandName === "move") {
    return command({
      command: "loco",
      action: "set_velocity",
      velocity: [Number(args.vx || 0), Number(args.vy || 0), Number(args.omega || 0)],
      duration: Number(args.duration || 1.0),
    });
  }
  if (commandName === "set_balance_mode") {
    return command({ command: "loco", action: "set_balance_mode", balance_mode: toInt(args.balance_mode, 0) });
  }
  if (commandName === "set_swing_height") {
    return command({ command: "loco", action: "set_swing_height", swing_height: Number(args.swing_height || 0) });
  }
  if (commandName === "set_stand_height") {
    return command({ command: "loco", action: "set_stand_height", stand_height: Number(args.stand_height || 0) });
  }
  if (commandName === "set_speed_mode") {
    return command({ command: "loco", action: "set_speed_mode", speed_mode: toInt(args.speed_mode, 0) });
  }
  if (commandName === "switch_move_mode") {
    return command({ command: "loco", action: "switch_move_mode", continuous_move: args.continuous_move === true });
  }
  if (commandName === "switch_to_internal_ctrl") {
    return command({ command: "loco", action: "switch_to_internal_ctrl", internal_mode: toInt(args.internal_mode, 0) });
  }
  const actionByCommand = {
    get_fsm_id: { action: "get_fsm_id" },
    get_fsm_mode: { action: "get_fsm_mode" },
    get_balance_mode: { action: "get_balance_mode" },
    get_swing_height: { action: "get_swing_height" },
    get_stand_height: { action: "get_stand_height" },
    get_phase: { action: "get_phase" },
    damp: { action: "set_fsm_id", fsm_id: 1, mode: "damp" },
    start: { action: "set_fsm_id", fsm_id: 500, mode: "start" },
    squat: { action: "set_fsm_id", fsm_id: 2, mode: "squat" },
    sit: { action: "set_fsm_id", fsm_id: 3, mode: "sit" },
    stand_up: { action: "set_fsm_id", fsm_id: 4, mode: "stand_up" },
    zero_torque: { action: "set_fsm_id", fsm_id: 0, mode: "zero_torque" },
    switch_to_user_ctrl: { action: "switch_to_user_ctrl" },
    stop_move: { action: "set_velocity", velocity: [0, 0, 0], duration: 0 },
    low_stand: { action: "low_stand" },
    high_stand: { action: "high_stand" },
    wave_hand: { action: "wave_hand" },
    shake_hand: { action: "shake_hand" },
  };
  const payload = actionByCommand[commandName];
  if (!payload) {
    throw new Error(`Unsupported G1 loco command: ${commandName}`);
  }
  return command({ command: "loco", ...payload });
}

async function executeG1AgvCommand(args) {
  const commandName = args.command || "move";
  if (commandName === "height_adjust") {
    const heightVelocity = clampNumber(args.vz, -1.0, 1.0, 0);
    return command({
      command: "agv",
      action: "height_adjust",
      service: "agv",
      simulated: true,
      height_velocity: heightVelocity,
    }).catch((error) => ({
      ok: true,
      simulated: true,
      service: "agv",
      action: "height_adjust",
      height_velocity: heightVelocity,
      transport_warning: error.message,
    }));
  }
  if (commandName !== "move") {
    throw new Error(`Unsupported G1 AGV command: ${commandName}`);
  }
  const vx = clampNumber(args.vx, -1.5, 1.5, 0);
  const vyaw = clampNumber(args.vyaw, -0.6, 0.6, 0);
  return command({
    command: "loco",
    action: "set_velocity",
    velocity: [vx, 0, vyaw],
    duration: 1.0,
    agv: {
      service: "agv",
      simulated: true,
      requested_velocity: [Number(args.vx || 0), Number(args.vy || 0), Number(args.vyaw || 0)],
      ignored_lateral_velocity: Number(args.vy || 0),
      limits: { vx: [-1.5, 1.5], vyaw: [-0.6, 0.6] },
    },
  });
}

async function executeG1MotionSwitcher(args) {
  const commandName = args.command || "check_mode";
  if (commandName === "check_mode") {
    return command({ command: "motion_switcher", action: "check_mode" });
  }
  if (commandName === "select_mode") {
    return command({ command: "motion_switcher", action: "select_mode", name: String(args.name || "ai") });
  }
  if (commandName === "release_mode") {
    return command({ command: "motion_switcher", action: "release_mode" });
  }
  if (commandName === "set_silent") {
    return command({ command: "motion_switcher", action: "set_silent", silent: args.silent === true });
  }
  if (commandName === "get_silent") {
    return command({ command: "motion_switcher", action: "get_silent" });
  }
  throw new Error(`Unsupported G1 motion switcher command: ${commandName}`);
}

async function safetyStop() {
  const steps = [];
  let ok = true;
  for (const [step, body] of [
    ["release_motion_mode", { command: "motion_switcher", action: "release_mode" }],
    ["damp_locomotion", { command: "loco", action: "set_fsm_id", fsm_id: 1, mode: "damp" }],
    ["neutral_pose", { command: "pose", pose: "neutral" }],
    ["pause", { command: "pause" }],
  ]) {
    try {
      const result = await command(body);
      const stepOk = result?.ok !== false;
      steps.push({ step, ok: stepOk, result });
      ok = ok && stepOk;
    } catch (error) {
      steps.push({ step, ok: false, error: error.message });
      ok = false;
    }
  }
  return { ok, mode: "simulator", steps };
}

async function executeG1Lowcmd(args) {
  if (!Array.isArray(args.motor_cmd)) {
    throw new Error("g1_lowcmd requires motor_cmd as an array");
  }
  return command({
    command: "lowcmd",
    topic: args.topic || "rt/lowcmd",
    mode_pr: Number(args.mode_pr || 0),
    mode_machine: Number(args.mode_machine || 0),
    crc: Number(args.crc || 0),
    motor_cmd: args.motor_cmd.map((cmd) => ({
      mode: Number(cmd?.mode || 0),
      q: Number(cmd?.q || 0),
      dq: Number(cmd?.dq || 0),
      tau: Number(cmd?.tau || 0),
      kp: Number(cmd?.kp || 0),
      kd: Number(cmd?.kd || 0),
    })),
  });
}

async function executeG1HandSdk(args) {
  const tau = clampNumber(args.tau, -1.5, 1.5, 0.3);
  const weight = clampNumber(args.weight, 0, 1, 1);
  const motorCount = clampInt(args.motor_count, 1, 12, 4);
  const mode = clampInt(Math.round(weight * 100), 0, 100, 100);
  return command({
    command: "hand_sdk",
    topic: "rt/hand_sdk",
    cmds: Array.from({ length: motorCount }, (_, index) => ({
      mode: index === 0 ? mode : 0,
      q: 0,
      dq: 0,
      tau,
      kp: 0,
      kd: 0,
    })),
  });
}

async function executeG1Dex3Command(args) {
  const hand = args.hand === "left" ? "left" : "right";
  const q = clampNumber(args.q, -1.75, 1.75, 0.25);
  const kp = clampNumber(args.kp, 0, 20, 1.5);
  const kd = clampNumber(args.kd, 0, 5, 0.1);
  return command({
    command: "dex3",
    hand,
    topic: `rt/dex3/${hand}/cmd`,
    motor_cmd: Array.from({ length: 7 }, (_, index) => ({
      mode: 0x10 | index,
      q,
      dq: 0,
      tau: 0,
      kp,
      kd,
    })),
  });
}

async function executeG1JointTargets(args) {
  if (!args.targets || typeof args.targets !== "object" || Array.isArray(args.targets)) {
    throw new Error("g1_apply_joint_targets requires targets as an object");
  }
  const targets = {};
  for (const [name, value] of Object.entries(args.targets)) {
    targets[name] = Number(value);
  }
  return command({
    command: "joint_targets",
    targets,
    kp: Number(args.kp ?? 38.0),
    kd: Number(args.kd ?? 1.4),
    tau: Number(args.tau || 0),
    dq: Number(args.dq || 0),
  });
}

async function readActiveMjcf() {
  const paths = runtimePaths();
  const xml = await fsp.readFile(paths.hostModelPath, "utf8");
  return {
    host_model_path: paths.hostModelPath,
    container_model_path: paths.containerModelPath,
    bytes: Buffer.byteLength(xml),
    xml,
  };
}

function validateMjcf(modelPath) {
  const target = modelPath || readComposeEnv().UNITREE_G1_MODEL_PATH || "/opt/unitree_mujoco/unitree_robots/g1/scene_29dof.xml";
  const code = [
    "import mujoco, sys",
    "path = sys.argv[1]",
    "model = mujoco.MjModel.from_xml_path(path)",
    "print({'nq': model.nq, 'nv': model.nv, 'nbody': model.nbody, 'ngeom': model.ngeom})",
  ].join("; ");
  return runChecked("docker", ["exec", DEFAULT_CONTAINER, "python3", "-c", code, target], { timeoutMs: 30000 });
}

async function addBoxToScene(args) {
  const name = String(args.name || "").trim();
  if (!/^[A-Za-z0-9_-]+$/.test(name)) {
    throw new Error("scene_add_box requires a safe alphanumeric name");
  }

  const position = numericArray(args.position, 3, "position");
  const size = numericArray(args.size, 3, "size");
  const rgba = args.rgba ? numericArray(args.rgba, 4, "rgba") : [0.2, 0.7, 1.0, 1.0];
  const paths = runtimePaths();
  const sourceXml = await fsp.readFile(paths.hostModelPath, "utf8");
  const body = [
    `  <body name="${escapeXml(name)}" pos="${position.join(" ")}">`,
    `    <geom name="${escapeXml(name)}_geom" type="box" size="${size.join(" ")}" rgba="${rgba.join(" ")}"/>`,
    "  </body>",
  ].join("\n");
  const marker = "</worldbody>";
  if (!sourceXml.includes(marker)) {
    throw new Error(`Could not find ${marker} in ${paths.hostModelPath}`);
  }

  const sceneDir = path.join(paths.assetRoot, "cybernetic_scenes");
  await fsp.mkdir(sceneDir, { recursive: true });
  const outName = `g1_${name}.xml`;
  const hostOutputPath = path.join(sceneDir, outName);
  const containerOutputPath = `/opt/unitree_mujoco/cybernetic_scenes/${outName}`;
  const nextXml = sourceXml.replace(marker, `${body}\n${marker}`);
  await fsp.writeFile(hostOutputPath, nextXml);

  const result = {
    host_output_path: hostOutputPath,
    container_output_path: containerOutputPath,
    activated: false,
    object: { name, position, size, rgba },
  };

  if (args.activate === true) {
    updateComposeEnv({ UNITREE_G1_MODEL_PATH: containerOutputPath });
    result.activate = runChecked("docker", [...composeArgs(), "up", "-d", "--force-recreate"], { timeoutMs: 180000 });
    result.activated = true;
  }

  return result;
}

async function addObjectToScene(args) {
  const type = args.type || "box";
  if (type !== "box") {
    throw new Error(`Unsupported scene object type: ${type}`);
  }
  return addBoxToScene(args);
}

async function listSceneObjects() {
  const paths = runtimePaths();
  const sceneDir = path.join(paths.assetRoot, "cybernetic_scenes");
  const files = fs.existsSync(sceneDir)
    ? (await fsp.readdir(sceneDir)).filter((entry) => entry.endsWith(".xml")).sort()
    : [];
  const scenes = [];
  for (const file of files) {
    const hostPath = path.join(sceneDir, file);
    const xml = await fsp.readFile(hostPath, "utf8");
    scenes.push({
      name: file,
      host_path: hostPath,
      container_path: `/opt/unitree_mujoco/cybernetic_scenes/${file}`,
      active: hostPath === paths.hostModelPath,
      objects: parseCyberneticObjects(xml),
    });
  }
  const activeXml = fs.existsSync(paths.hostModelPath) ? await fsp.readFile(paths.hostModelPath, "utf8") : "";
  return {
    active_scene: {
      host_model_path: paths.hostModelPath,
      container_model_path: paths.containerModelPath,
      generated: paths.hostModelPath.startsWith(sceneDir),
      objects: activeXml ? parseCyberneticObjects(activeXml) : [],
    },
    generated_scene_dir: sceneDir,
    scenes,
  };
}

async function removeObjectFromScene(args) {
  const name = String(args.name || "").trim();
  if (!/^[A-Za-z0-9_-]+$/.test(name)) {
    throw new Error("scene_remove_object requires a safe alphanumeric name");
  }

  const paths = runtimePaths();
  const sourcePath = resolveSceneHostPath(args.scene_path, paths);
  const sourceXml = await fsp.readFile(sourcePath, "utf8");
  const { xml, removed } = removeCyberneticObjectBlock(sourceXml, name);
  if (!removed) {
    throw new Error(`Object '${name}' was not found as a Cybernetic-generated scene object in ${sourcePath}`);
  }

  const sceneDir = path.join(paths.assetRoot, "cybernetic_scenes");
  await fsp.mkdir(sceneDir, { recursive: true });
  const base = path.basename(sourcePath, ".xml").replace(/^g1_/, "");
  const outName = `g1_${safeSegment(base)}_without_${safeSegment(name)}.xml`;
  const hostOutputPath = path.join(sceneDir, outName);
  const containerOutputPath = `/opt/unitree_mujoco/cybernetic_scenes/${outName}`;
  await fsp.writeFile(hostOutputPath, xml);

  const result = {
    host_output_path: hostOutputPath,
    container_output_path: containerOutputPath,
    removed_object: name,
    activated: false,
  };
  if (args.activate === true) {
    updateComposeEnv({ UNITREE_G1_MODEL_PATH: containerOutputPath });
    result.activate = runChecked("docker", [...composeArgs(), "up", "-d", "--force-recreate"], { timeoutMs: 180000 });
    result.activated = true;
  }
  return result;
}

async function scaffoldPython(args) {
  const action = typeof args.action === "string" ? args.action : "raise_hand";
  const code = sdkPythonTemplate(action, args);
  if (!args.path) {
    return { action, code };
  }

  const target = safeWorkspacePath(args.path);
  await fsp.mkdir(path.dirname(target), { recursive: true });
  await fsp.writeFile(target, code);
  return { action, path: target, code };
}

function roboticsToolReference() {
  return {
    version: 1,
    notes: [
      "Read tools first when unsure; prefer safety_stop after deliberate motion.",
      "Scene edit tools write copied MJCF scenes under the runtime asset tree instead of mutating pinned upstream Unitree assets.",
      "Official SDK2/CycloneDDS tools require the opt-in sidecar assets prepared by unitree_prepare_sdk2_sidecar.",
    ],
    tools: [
      toolReference("sim_status", "read", "none", "Simulator may be stopped or running."),
      toolReference("sim_start", "lifecycle", "Starts Docker MuJoCo harness.", "Docker available and assets prepared."),
      toolReference("sim_stop", "lifecycle", "Stops Docker MuJoCo harness.", "Simulator container exists."),
      toolReference("sim_reset", "state-changing", "Resets MuJoCo state.", "Simulator HTTP endpoint reachable."),
      toolReference(
        "sim_validate_behavior",
        "read-with-evidence",
        "May write a snapshot file.",
        "Simulator running after a behavior.",
      ),
      toolReference(
        "robot_evidence_bundle",
        "evidence-write",
        "Writes simulator status, lowstate, joint state, provider diagnostics, and optional screenshots into one JSON bundle.",
        "Simulator HTTP endpoint reachable; renderer needed for image evidence.",
      ),
      toolReference(
        "viewer_camera_control",
        "viewer-state",
        "Moves only the viewer camera.",
        "Simulator HTTP camera endpoint reachable.",
      ),
      toolReference("viewer_snapshot", "read", "Returns an MCP image result.", "Renderer has a cached camera frame."),
      toolReference("viewer_snapshot_file", "evidence-write", "Writes a workspace image file.", "Renderer has a cached camera frame."),
      toolReference("viewer_snapshot_series", "evidence-write", "Moves camera and writes multiple images.", "Simulator running with render frames."),
      toolReference("scene_add_object", "scene-write", "Writes copied MJCF and may restart container when activated.", "Mounted Unitree G1 MJCF assets."),
      toolReference("scene_remove_object", "scene-write", "Writes copied MJCF and may restart container when activated.", "Generated Cybernetic scene object exists."),
      toolReference(
        "unitree_sdk_scaffold_python",
        "file-write",
        "Optionally writes a Python script.",
        "Workspace writable; simulator not required until script execution.",
      ),
      toolReference("g1_execute_action", "robot-motion", "Applies a high-level G1 pose.", "Simulator running; use safety_stop afterward."),
      toolReference("g1_loco_command", "robot-motion", "Changes locomotion FSM or velocity.", "Simulator running; prefer small velocities and safety_stop afterward."),
      toolReference("g1_agv_command", "robot-motion", "Runs Unitree G1 AgvClient-shaped move or height intent.", "Simulator running; lateral AGV velocity is ignored."),
      toolReference("g1_safety_check", "read", "Evaluates Unitree-inspired termination checks.", "Run before and after deliberate motion."),
      toolReference("g1_apply_joint_targets", "robot-motion", "Publishes simulator-backed lowcmd targets.", "Simulator running with joint_state endpoint."),
      toolReference("g1_lowcmd", "robot-motion", "Publishes low-level motor commands.", "Advanced use only; validate joint indices and use safety_stop."),
      toolReference("g1_hand_sdk", "robot-motion-intent", "Publishes rt/hand_sdk open/close intent.", "Simulator running; records hand intent rather than full finger physics."),
      toolReference("g1_dex3_command", "robot-motion-intent", "Publishes Dex3 HandCmd_ intent and records synthesized hand state.", "Simulator running; records hand telemetry rather than full finger physics."),
      toolReference("g1_lowstate", "read", "Reads rt/lowstate-shaped telemetry.", "Simulator running."),
      toolReference("g1_joint_state", "read", "Reads named joint mapping and limits.", "Simulator running."),
      toolReference("safety_stop", "safety", "Damps locomotion, neutralizes pose, pauses sim.", "Use after motion or when state feels uncertain."),
      toolReference(
        "unitree_sdk_compatibility_audit",
        "read",
        "Scans cloned official Unitree G1 Python examples and reports shim import/method coverage.",
        "Official unitree_sdk2_python checkout available under ~/wagmi or supplied path.",
      ),
      toolReference(
        "unitree_sdk_behavior_smoke",
        "robot-motion-validation",
        "Runs safe arm, loco, and lowcmd Unitree SDK-shaped calls and returns simulator evidence.",
        "Simulator HTTP endpoint reachable; use safety_stop after exploratory motion.",
      ),
      toolReference("python_control_run", "script-execution", "Runs a workspace Python script to completion.", "Script reviewed; simulator state depends on script."),
      toolReference("python_control_start", "script-execution", "Starts managed long-running Python job.", "Script reviewed; monitor with python_control_logs."),
      toolReference(
        "unitree_probe_official_mujoco_rpc_discovery",
        "read",
        "Creates Unitree-typed RPC request writers and reports DDS matched-reader counts.",
        "Managed official MuJoCo DDS session running and ready.",
      ),
      toolReference(
        "unitree_probe_rpc_bridge_smoke",
        "diagnostic",
        "Starts temporary sport/agv/arm RPC services and verifies official SDK clients can call them.",
        "Official SDK2 sidecar prepared; does not command the robot.",
      ),
      toolReference(
        "unitree_start_rpc_bridge",
        "service-start",
        "Starts a named sport/agv/arm RPC bridge container.",
        "Official SDK2 sidecar prepared.",
      ),
      toolReference("unitree_rpc_bridge_status", "read", "Inspects the named sport/agv/arm RPC bridge container.", "Bridge may or may not be running."),
      toolReference(
        "unitree_probe_rpc_bridge_client",
        "diagnostic",
        "Calls the managed RPC bridge with official SDK clients.",
        "Managed Unitree RPC bridge running and ready.",
      ),
      toolReference(
        "unitree_verify_rpc_bridge",
        "diagnostic",
        "Starts if needed, calls the managed bridge, and summarizes simulator readback/forwarding evidence.",
        "Official SDK2 sidecar prepared; simulator HTTP endpoint recommended for strong evidence.",
      ),
      toolReference(
        "unitree_command_rpc_bridge",
        "robot-motion",
        "Sends one SDK-shaped sport/agv/arm RPC through the managed bridge.",
        "Managed Unitree RPC bridge running or start_if_needed enabled.",
      ),
      toolReference("unitree_stop_rpc_bridge", "service-stop", "Stops and removes the named sport/agv/arm RPC bridge container.", "Bridge container exists."),
      toolReference(
        "unitree_probe_official_mujoco_loco_rpc",
        "read-with-optional-stop",
        "May send a safe StopMove RPC when include_stop is true.",
        "Managed official MuJoCo DDS session running and ready.",
      ),
      toolReference(
        "unitree_read_official_mujoco_lowstate",
        "read",
        "Reads one official rt/lowstate sample from the managed Unitree MuJoCo session.",
        "Managed official MuJoCo DDS session running and ready.",
      ),
      toolReference(
        "unitree_command_official_mujoco_arm_pose",
        "robot-motion",
        "Publishes bounded official rt/lowcmd arm-pose frames to the managed Unitree MuJoCo session.",
        "Managed official MuJoCo DDS session running and ready.",
      ),
      toolReference(
        "unitree_command_official_mujoco_lowcmd",
        "expert-robot-motion",
        "Publishes one sanitized generic official rt/lowcmd or rt/arm_sdk frame to the managed Unitree MuJoCo session.",
        "Managed official MuJoCo DDS session running and ready; inspect lowstate first.",
      ),
      toolReference(
        "unitree_official_mujoco_evidence_bundle",
        "robot-motion-with-evidence",
        "May start official MuJoCo session, commands a bounded arm pose, and writes JSON lowstate evidence.",
        "Official SDK2 sidecar prepared; managed official session running or start_if_needed true.",
      ),
      toolReference("protocol_probe_http", "read", "Probes GameControl HTTP endpoint.", "Simulator HTTP endpoint reachable."),
      toolReference("protocol_probe_ws", "read", "Opens physics WebSocket and samples one topic.", "Physics WebSocket reachable."),
    ],
  };
}

function toolReference(name, safetyLevel, sideEffects, expectedSimulatorState) {
  return {
    name,
    safety_level: safetyLevel,
    side_effects: sideEffects,
    expected_simulator_state: expectedSimulatorState,
  };
}

async function executeG1Action(action) {
  const spec = G1_ACTION_POSES[action] || G1_ACTION_POSES.raise_right_hand;
  return command({ command: "pose", pose: spec.pose });
}

async function protocolProbeHttp(requestPath) {
  const normalized = requestPath.startsWith("/") ? requestPath : `/${requestPath}`;
  if (!/^\/[A-Za-z0-9_./?-]*$/.test(normalized)) {
    throw new Error("Unsafe HTTP probe path");
  }
  return getJson(normalized);
}

async function protocolProbeWs(topic, timeoutMs) {
  if (typeof WebSocket === "undefined") {
    throw new Error("This Node runtime does not expose a global WebSocket client");
  }
  return new Promise((resolve, reject) => {
    const ws = new WebSocket(physicsUrl());
    ws.binaryType = "arraybuffer";
    const timeout = setTimeout(() => {
      try {
        ws.close();
      } catch {}
      reject(new Error(`Timed out waiting for ${topic}`));
    }, timeoutMs);

    ws.addEventListener("open", () => {
      ws.send(`subscribe:${topic}`);
    });
    ws.addEventListener("error", () => {
      clearTimeout(timeout);
      reject(new Error(`Failed to open ${physicsUrl()}`));
    });
    ws.addEventListener("message", async (event) => {
      if (typeof event.data === "string") {
        return;
      }
      clearTimeout(timeout);
      const buffer = Buffer.from(await event.data.arrayBuffer());
      const messageType = buffer.length >= 4 ? buffer.readUInt32BE(0) : null;
      const payloadLength = buffer.length >= 8 ? buffer.readUInt32BE(4) : null;
      try {
        ws.send(`unsubscribe:${topic}`);
        ws.close();
      } catch {}
      resolve({
        topic,
        message_type: messageType,
        payload_length: payloadLength,
        frame_bytes: buffer.length,
        note: "Payload is MessagePack or camera bytes; use the protocol probe script for deep decode.",
      });
    });
  });
}

function startPythonJob(scriptPath, args) {
  const absoluteScript = safeWorkspacePath(scriptPath);
  const id = `py-${Date.now()}-${nextJobId++}`;
  const child = spawn("python3", [absoluteScript, ...args.map(String)], {
    cwd: root,
    env: pythonEnv(),
    stdio: ["ignore", "pipe", "pipe"],
  });

  const job = {
    id,
    child,
    script_path: absoluteScript,
    args,
    pid: child.pid,
    status: "running",
    started_at: new Date().toISOString(),
    finished_at: null,
    exit_code: null,
    signal: null,
    stdout: "",
    stderr: "",
  };
  jobs.set(id, job);
  child.stdout.on("data", (chunk) => appendJobLog(job, "stdout", chunk));
  child.stderr.on("data", (chunk) => appendJobLog(job, "stderr", chunk));
  child.on("exit", (code, signal) => {
    job.status = "exited";
    job.exit_code = code;
    job.signal = signal;
    job.finished_at = new Date().toISOString();
  });
  return publicJob(job);
}

function runPythonControl(scriptPath, args, timeoutMs) {
  const absoluteScript = safeWorkspacePath(scriptPath);
  return runChecked("python3", [absoluteScript, ...args.map(String)], {
    timeoutMs,
    env: pythonEnv(),
  });
}

function signalJob(jobId, signal, status) {
  const job = jobs.get(jobId);
  if (!job) {
    throw new Error(`Unknown Python control job: ${jobId}`);
  }
  if (job.status === "exited") {
    return publicJob(job);
  }
  job.child.kill(signal);
  job.status = status;
  return publicJob(job);
}

function jobSnapshot(jobId) {
  const job = jobs.get(jobId);
  if (!job) {
    throw new Error(`Unknown Python control job: ${jobId}`);
  }
  return publicJob(job, true);
}

function publicJob(job, includeLogs = false) {
  const value = {
    id: job.id,
    script_path: job.script_path,
    args: job.args,
    pid: job.pid,
    status: job.status,
    started_at: job.started_at,
    finished_at: job.finished_at,
    exit_code: job.exit_code,
    signal: job.signal,
  };
  if (includeLogs) {
    value.stdout = job.stdout;
    value.stderr = job.stderr;
  } else {
    value.stdout_tail = tail(job.stdout, 8000);
    value.stderr_tail = tail(job.stderr, 8000);
  }
  return value;
}

function appendJobLog(job, stream, chunk) {
  job[stream] += chunk.toString();
  if (job[stream].length > MAX_LOG_BYTES) {
    job[stream] = job[stream].slice(-MAX_LOG_BYTES);
  }
}

async function getJson(pathname) {
  const response = await fetch(`${gameControlUrl()}${pathname}`);
  const text = await response.text();
  if (!response.ok) {
    throw new Error(`GET ${pathname} failed: HTTP ${response.status} ${text}`);
  }
  return JSON.parse(text);
}

async function postJson(pathname, body) {
  const response = await fetch(`${gameControlUrl()}${pathname}`, {
    method: "POST",
    headers: { "content-type": "application/json" },
    body: JSON.stringify(body),
  });
  const text = await response.text();
  if (!response.ok) {
    throw new Error(`POST ${pathname} failed: HTTP ${response.status} ${text}`);
  }
  return JSON.parse(text);
}

function composeArgs() {
  return ["compose", "--env-file", composeEnvPath(), "-f", path.join(root, "overlays/unitree-g1-mujoco-container/compose.yaml")];
}

function composeEnvPath() {
  return path.join(root, ".runtime/unitree-g1-mujoco/compose.env");
}

function sdk2ComposeEnvPath() {
  return path.join(root, ".runtime/unitree-g1-sdk2/compose.env");
}

function sdk2ComposeArgs() {
  return ["compose", "--env-file", sdk2ComposeEnvPath(), "-f", path.join(root, "overlays/unitree-g1-sdk2-sidecar/compose.yaml")];
}

function sdk2SidecarStatus() {
  const envPath = sdk2ComposeEnvPath();
  if (!fs.existsSync(envPath)) {
    throw new Error("Missing SDK2 sidecar compose env. Run unitree_prepare_sdk2_sidecar first.");
  }
  const result = runChecked("docker", [...sdk2ComposeArgs(), "run", "--rm", "unitree-g1-sdk2-sidecar"], { timeoutMs: 180000 });
  let report = null;
  try {
    report = JSON.parse(result.stdout);
  } catch {
    const jsonStart = result.stdout.indexOf("{");
    const jsonEnd = result.stdout.lastIndexOf("}");
    if (jsonStart !== -1 && jsonEnd > jsonStart) {
      try {
        report = JSON.parse(result.stdout.slice(jsonStart, jsonEnd + 1));
      } catch {
        report = null;
      }
    }
  }
  return {
    command: `docker ${[...sdk2ComposeArgs(), "run", "--rm", "unitree-g1-sdk2-sidecar"].join(" ")}`,
    stdout: result.stdout,
    stderr: result.stderr,
    report,
  };
}

function sdk2OfficialMujocoPlan() {
  const status = sdk2SidecarStatus();
  return {
    command: status.command,
    stderr: status.stderr,
    report: status.report?.official_mujoco_peer ?? null,
    sdk2_probe: status.report?.sdk2_probe ?? null,
    next_step: status.report?.next_step ?? null,
  };
}

function sdk2BuildOfficialMujocoPeer() {
  const envPath = sdk2ComposeEnvPath();
  if (!fs.existsSync(envPath)) {
    throw new Error("Missing SDK2 sidecar compose env. Run unitree_prepare_sdk2_sidecar first.");
  }
  const result = runChecked(
    "docker",
    [...sdk2ComposeArgs(), "run", "--rm", "-e", "CYBER_UNITREE_ACTION=build_official_mujoco", "unitree-g1-sdk2-sidecar"],
    { timeoutMs: 1_800_000 },
  );
  let report = null;
  const jsonStart = result.stdout.indexOf("{");
  const jsonEnd = result.stdout.lastIndexOf("}");
  if (jsonStart !== -1 && jsonEnd > jsonStart) {
    try {
      report = JSON.parse(result.stdout.slice(jsonStart, jsonEnd + 1));
    } catch {
      report = null;
    }
  }
  return {
    command: `docker ${[...sdk2ComposeArgs(), "run", "--rm", "-e", "CYBER_UNITREE_ACTION=build_official_mujoco", "unitree-g1-sdk2-sidecar"].join(" ")}`,
    stdout: result.stdout,
    stderr: result.stderr,
    report,
  };
}

function sdk2ProbeOfficialMujocoLaunch() {
  const envPath = sdk2ComposeEnvPath();
  if (!fs.existsSync(envPath)) {
    throw new Error("Missing SDK2 sidecar compose env. Run unitree_prepare_sdk2_sidecar first.");
  }
  const result = runChecked(
    "docker",
    [...sdk2ComposeArgs(), "run", "--rm", "-e", "CYBER_UNITREE_ACTION=launch_probe_official_mujoco", "unitree-g1-sdk2-sidecar"],
    { timeoutMs: 240_000 },
  );
  let report = null;
  const jsonStart = result.stdout.indexOf("{");
  const jsonEnd = result.stdout.lastIndexOf("}");
  if (jsonStart !== -1 && jsonEnd > jsonStart) {
    try {
      report = JSON.parse(result.stdout.slice(jsonStart, jsonEnd + 1));
    } catch {
      report = null;
    }
  }
  return {
    command: `docker ${[...sdk2ComposeArgs(), "run", "--rm", "-e", "CYBER_UNITREE_ACTION=launch_probe_official_mujoco", "unitree-g1-sdk2-sidecar"].join(" ")}`,
    stdout: result.stdout,
    stderr: result.stderr,
    report,
  };
}

function sdk2StartOfficialMujocoSession() {
  const envPath = sdk2ComposeEnvPath();
  if (!fs.existsSync(envPath)) {
    throw new Error("Missing SDK2 sidecar compose env. Run unitree_prepare_sdk2_sidecar first.");
  }
  const removed = run("docker", ["rm", "-f", OFFICIAL_MUJOCO_SESSION_CONTAINER], { timeoutMs: 60_000 });
  const args = [
    ...sdk2ComposeArgs(),
    "run",
    "-d",
    "--name",
    OFFICIAL_MUJOCO_SESSION_CONTAINER,
    "-e",
    "CYBER_UNITREE_ACTION=serve_official_mujoco",
    "unitree-g1-sdk2-sidecar",
  ];
  const started = runChecked("docker", args, { timeoutMs: 180_000 });
  const status = waitForOfficialMujocoSessionReady();
  return {
    command: `docker ${args.join(" ")}`,
    removed_existing: {
      attempted: true,
      status: removed.status,
      stdout: removed.stdout,
      stderr: removed.stderr,
    },
    started,
    status,
  };
}

function waitForOfficialMujocoSessionReady(timeoutMs = 12_000) {
  const deadline = Date.now() + timeoutMs;
  let status = sdk2OfficialMujocoSessionStatus();
  while (status.running && !status.ready && Date.now() < deadline) {
    run("sleep", ["0.5"], { timeoutMs: 2_000 });
    status = sdk2OfficialMujocoSessionStatus();
  }
  return status;
}

function sdk2OfficialMujocoSessionStatus() {
  const inspect = run("docker", ["inspect", OFFICIAL_MUJOCO_SESSION_CONTAINER, "--format", "{{json .State}}"], { timeoutMs: 30_000 });
  const logs = run("docker", ["logs", "--tail", "2000", OFFICIAL_MUJOCO_SESSION_CONTAINER], { timeoutMs: 30_000 });
  let state = null;
  if (inspect.status === 0 && inspect.stdout.trim()) {
    try {
      state = JSON.parse(inspect.stdout.trim());
    } catch {
      state = null;
    }
  }
  let reports = logs.status === 0 ? parseJsonObjects(logs.stdout) : [];
  let lifecycleSource = "tail";
  if (state?.Running === true && reports.length === 0 && logs.status === 0) {
    const fullLogs = run("docker", ["logs", OFFICIAL_MUJOCO_SESSION_CONTAINER], { timeoutMs: 30_000 });
    if (fullLogs.status === 0) {
      reports = parseJsonObjects(fullLogs.stdout);
      lifecycleSource = "full_logs_fallback";
    }
  }
  const readyReport = reports.find((report) => report?.action === "serve_official_mujoco") ?? reports[0] ?? null;
  const exitReport =
    [...reports].reverse().find((report) => report?.action === "serve_official_mujoco_exit") ?? null;
  const lastReport = reports.length > 0 ? reports[reports.length - 1] : null;
  const running = state?.Running === true;
  return {
    container: OFFICIAL_MUJOCO_SESSION_CONTAINER,
    exists: inspect.status === 0,
    running,
    status: state?.Status ?? null,
    exit_code: state?.ExitCode ?? null,
    started_at: state?.StartedAt ?? null,
    finished_at: state?.FinishedAt ?? null,
    inspect_error: inspect.status === 0 ? null : inspect.stderr.trim(),
    ready_report: readyReport,
    last_report: lastReport,
    exit_report: exitReport,
    lifecycle_reports_seen: reports.length,
    lifecycle_report_source: lifecycleSource,
    ready: running && readyReport?.ok === true && !exitReport,
    logs_tail: logs.status === 0 ? logs.stdout : null,
    logs_error: logs.status === 0 ? null : logs.stderr.trim(),
  };
}

function sdk2StopOfficialMujocoSession() {
  const result = run("docker", ["rm", "-f", OFFICIAL_MUJOCO_SESSION_CONTAINER], { timeoutMs: 60_000 });
  return {
    container: OFFICIAL_MUJOCO_SESSION_CONTAINER,
    removed: result.status === 0,
    stdout: result.stdout,
    stderr: result.stderr,
    status: sdk2OfficialMujocoSessionStatus(),
  };
}

function sdk2ReadOfficialMujocoLowstate() {
  const envPath = sdk2ComposeEnvPath();
  if (!fs.existsSync(envPath)) {
    throw new Error("Missing SDK2 sidecar compose env. Run unitree_prepare_sdk2_sidecar first.");
  }
  const session = sdk2OfficialMujocoSessionStatus();
  if (!session.running || !session.ready) {
    return {
      ok: false,
      error: "managed official MuJoCo session is not running and ready",
      next_step: "Run unitree_start_official_mujoco_session, then inspect ready_report, exit_report, and logs_tail if readiness does not hold.",
      session,
    };
  }
  const actionEnv = "CYBER_UNITREE_ACTION=read_official_mujoco_lowstate";
  const args = [...sdk2ComposeArgs(), "run", "--rm", "-e", actionEnv, "unitree-g1-sdk2-sidecar"];
  const result = runChecked("docker", args, { timeoutMs: 120_000 });
  let report = null;
  const jsonStart = result.stdout.indexOf("{");
  const jsonEnd = result.stdout.lastIndexOf("}");
  if (jsonStart !== -1 && jsonEnd > jsonStart) {
    try {
      report = JSON.parse(result.stdout.slice(jsonStart, jsonEnd + 1));
    } catch {
      report = null;
    }
  }
  return {
    command: `docker ${args.join(" ")}`,
    session,
    stdout: result.stdout,
    stderr: result.stderr,
    report,
  };
}

function sdk2ProbeOfficialMujocoLocoRpc(options = {}) {
  const envPath = sdk2ComposeEnvPath();
  if (!fs.existsSync(envPath)) {
    throw new Error("Missing SDK2 sidecar compose env. Run unitree_prepare_sdk2_sidecar first.");
  }
  const session = sdk2OfficialMujocoSessionStatus();
  if (!session.running || !session.ready) {
    return {
      ok: false,
      error: "managed official MuJoCo session is not running and ready",
      next_step: "Run unitree_start_official_mujoco_session, then inspect ready_report, exit_report, and logs_tail if readiness does not hold.",
      session,
    };
  }
  const includeStop = options.include_stop === true;
  const timeoutSeconds = clampNumber(options.timeout_seconds, 0.2, 10, 2);
  const env = [
    "CYBER_UNITREE_ACTION=probe_official_mujoco_loco_rpc",
    `CYBER_UNITREE_LOCO_RPC_TIMEOUT=${timeoutSeconds}`,
    `CYBER_UNITREE_LOCO_RPC_STOP_MOVE=${includeStop ? 1 : 0}`,
  ];
  const args = [...sdk2ComposeArgs(), "run", "--rm"];
  for (const entry of env) {
    args.push("-e", entry);
  }
  args.push("unitree-g1-sdk2-sidecar");
  const result = runChecked("docker", args, { timeoutMs: 120_000 });
  let report = null;
  const jsonStart = result.stdout.indexOf("{");
  const jsonEnd = result.stdout.lastIndexOf("}");
  if (jsonStart !== -1 && jsonEnd > jsonStart) {
    try {
      report = JSON.parse(result.stdout.slice(jsonStart, jsonEnd + 1));
    } catch {
      report = null;
    }
  }
  return {
    command: `docker ${args.join(" ")}`,
    session,
    stdout: result.stdout,
    stderr: result.stderr,
    report,
  };
}

function sdk2ProbeOfficialMujocoRpcDiscovery(options = {}) {
  const envPath = sdk2ComposeEnvPath();
  if (!fs.existsSync(envPath)) {
    throw new Error("Missing SDK2 sidecar compose env. Run unitree_prepare_sdk2_sidecar first.");
  }
  const session = sdk2OfficialMujocoSessionStatus();
  if (!session.running || !session.ready) {
    return {
      ok: false,
      error: "managed official MuJoCo session is not running and ready",
      next_step: "Run unitree_start_official_mujoco_session, then inspect ready_report, exit_report, and logs_tail if readiness does not hold.",
      session,
    };
  }
  const waitSeconds = clampNumber(options.wait_seconds, 0.1, 10, 1);
  const env = [
    "CYBER_UNITREE_ACTION=probe_official_mujoco_rpc_discovery",
    `CYBER_UNITREE_RPC_DISCOVERY_WAIT=${waitSeconds}`,
  ];
  const args = [...sdk2ComposeArgs(), "run", "--rm"];
  for (const entry of env) {
    args.push("-e", entry);
  }
  args.push("unitree-g1-sdk2-sidecar");
  const result = runChecked("docker", args, { timeoutMs: 120_000 });
  let report = null;
  const jsonStart = result.stdout.indexOf("{");
  const jsonEnd = result.stdout.lastIndexOf("}");
  if (jsonStart !== -1 && jsonEnd > jsonStart) {
    try {
      report = JSON.parse(result.stdout.slice(jsonStart, jsonEnd + 1));
    } catch {
      report = null;
    }
  }
  return {
    command: `docker ${args.join(" ")}`,
    session,
    stdout: result.stdout,
    stderr: result.stderr,
    report,
  };
}

function sdk2ProbeRpcBridgeSmoke(options = {}) {
  const envPath = sdk2ComposeEnvPath();
  if (!fs.existsSync(envPath)) {
    throw new Error("Missing SDK2 sidecar compose env. Run unitree_prepare_sdk2_sidecar first.");
  }
  const timeoutSeconds = clampNumber(options.timeout_seconds, 0.2, 10, 1);
  const env = [
    "CYBER_UNITREE_ACTION=probe_unitree_rpc_bridge_smoke",
    `CYBER_UNITREE_RPC_BRIDGE_TIMEOUT=${timeoutSeconds}`,
  ];
  const args = [...sdk2ComposeArgs(), "run", "--rm"];
  for (const entry of env) {
    args.push("-e", entry);
  }
  args.push("unitree-g1-sdk2-sidecar");
  const result = runChecked("docker", args, { timeoutMs: 120_000 });
  let report = null;
  const jsonStart = result.stdout.indexOf("{");
  const jsonEnd = result.stdout.lastIndexOf("}");
  if (jsonStart !== -1 && jsonEnd > jsonStart) {
    try {
      report = JSON.parse(result.stdout.slice(jsonStart, jsonEnd + 1));
    } catch {
      report = null;
    }
  }
  return {
    command: `docker ${args.join(" ")}`,
    stdout: result.stdout,
    stderr: result.stderr,
    report,
  };
}

function sdk2StartRpcBridge() {
  const envPath = sdk2ComposeEnvPath();
  if (!fs.existsSync(envPath)) {
    throw new Error("Missing SDK2 sidecar compose env. Run unitree_prepare_sdk2_sidecar first.");
  }
  const removed = run("docker", ["rm", "-f", UNITREE_RPC_BRIDGE_CONTAINER], { timeoutMs: 60_000 });
  const args = [
    ...sdk2ComposeArgs(),
    "run",
    "-d",
    "--name",
    UNITREE_RPC_BRIDGE_CONTAINER,
    "-e",
    "CYBER_UNITREE_ACTION=serve_unitree_rpc_bridge",
    "unitree-g1-sdk2-sidecar",
  ];
  const started = runChecked("docker", args, { timeoutMs: 120_000 });
  const status = waitForRpcBridgeReady();
  return {
    command: `docker ${args.join(" ")}`,
    removed_existing: {
      attempted: true,
      status: removed.status,
      stdout: removed.stdout,
      stderr: removed.stderr,
    },
    started,
    status,
  };
}

function waitForRpcBridgeReady(timeoutMs = 8_000) {
  const deadline = Date.now() + timeoutMs;
  let status = sdk2RpcBridgeStatus();
  while (status.running && !status.ready && Date.now() < deadline) {
    run("sleep", ["0.25"], { timeoutMs: 2_000 });
    status = sdk2RpcBridgeStatus();
  }
  return status;
}

function sdk2RpcBridgeStatus() {
  return sdk2ManagedJsonLogContainerStatus(
    UNITREE_RPC_BRIDGE_CONTAINER,
    "serve_unitree_rpc_bridge",
    "serve_unitree_rpc_bridge_exit",
  );
}

function sdk2StopRpcBridge() {
  const result = run("docker", ["rm", "-f", UNITREE_RPC_BRIDGE_CONTAINER], { timeoutMs: 60_000 });
  return {
    container: UNITREE_RPC_BRIDGE_CONTAINER,
    removed: result.status === 0,
    stdout: result.stdout,
    stderr: result.stderr,
    status: sdk2RpcBridgeStatus(),
  };
}

function sdk2ProbeRpcBridgeClient(options = {}) {
  const envPath = sdk2ComposeEnvPath();
  if (!fs.existsSync(envPath)) {
    throw new Error("Missing SDK2 sidecar compose env. Run unitree_prepare_sdk2_sidecar first.");
  }
  const bridge = sdk2RpcBridgeStatus();
  if (!bridge.running || !bridge.ready) {
    return {
      ok: false,
      error: "managed Unitree RPC bridge is not running and ready",
      next_step: "Run unitree_start_rpc_bridge, then inspect unitree_rpc_bridge_status if readiness does not hold.",
      bridge,
    };
  }
  const timeoutSeconds = clampNumber(options.timeout_seconds, 0.2, 10, 1);
  const env = [
    "CYBER_UNITREE_ACTION=probe_unitree_rpc_bridge_client",
    `CYBER_UNITREE_RPC_BRIDGE_TIMEOUT=${timeoutSeconds}`,
  ];
  const args = [...sdk2ComposeArgs(), "run", "--rm"];
  for (const entry of env) {
    args.push("-e", entry);
  }
  args.push("unitree-g1-sdk2-sidecar");
  const result = runChecked("docker", args, { timeoutMs: 120_000 });
  let report = null;
  const jsonStart = result.stdout.indexOf("{");
  const jsonEnd = result.stdout.lastIndexOf("}");
  if (jsonStart !== -1 && jsonEnd > jsonStart) {
    try {
      report = JSON.parse(result.stdout.slice(jsonStart, jsonEnd + 1));
    } catch {
      report = null;
    }
  }
  return {
    command: `docker ${args.join(" ")}`,
    bridge,
    stdout: result.stdout,
    stderr: result.stderr,
    report,
  };
}

function sdk2VerifyRpcBridge(options = {}) {
  const startIfNeeded = options.start_if_needed !== false;
  const stopAfter = options.stop_after === true;
  let started = null;
  let stopped = null;
  const statusBefore = sdk2RpcBridgeStatus();
  if ((!statusBefore.running || !statusBefore.ready) && startIfNeeded) {
    started = sdk2StartRpcBridge();
  }
  const status = sdk2RpcBridgeStatus();
  if (!status.running || !status.ready) {
    const result = {
      ok: false,
      source: "managed_unitree_sdk2_rpc_bridge",
      started,
      status_before: statusBefore,
      status,
      error: "managed Unitree RPC bridge is not running and ready",
      next_step: "Run unitree_start_rpc_bridge or inspect unitree_rpc_bridge_status before verifying.",
    };
    if (stopAfter && started) {
      stopped = sdk2StopRpcBridge();
      result.stopped = stopped;
    }
    return result;
  }
  const client = sdk2ProbeRpcBridgeClient(options);
  const calls = client.report?.rpc_bridge_client?.calls ?? [];
  const summary = summarizeRpcBridgeCalls(calls);
  const result = {
    ok: client.report?.rpc_bridge_client?.ok === true && summary.all_calls_ok,
    source: "managed_unitree_sdk2_rpc_bridge",
    started,
    status_before: statusBefore,
    status,
    client,
    summary,
  };
  if (stopAfter) {
    stopped = sdk2StopRpcBridge();
    result.stopped = stopped;
  }
  return result;
}

function sdk2CommandRpcBridge(options = {}) {
  const startIfNeeded = options.start_if_needed !== false;
  const stopAfter = options.stop_after === true;
  let started = null;
  let stopped = null;
  const statusBefore = sdk2RpcBridgeStatus();
  if ((!statusBefore.running || !statusBefore.ready) && startIfNeeded) {
    started = sdk2StartRpcBridge();
  }
  const status = sdk2RpcBridgeStatus();
  if (!status.running || !status.ready) {
    const result = {
      ok: false,
      source: "managed_unitree_sdk2_rpc_bridge",
      started,
      status_before: statusBefore,
      status,
      error: "managed Unitree RPC bridge is not running and ready",
      next_step: "Run unitree_start_rpc_bridge or inspect unitree_rpc_bridge_status before sending a command.",
    };
    if (stopAfter && started) {
      stopped = sdk2StopRpcBridge();
      result.stopped = stopped;
    }
    return result;
  }
  const envPath = sdk2ComposeEnvPath();
  if (!fs.existsSync(envPath)) {
    throw new Error("Missing SDK2 sidecar compose env. Run unitree_prepare_sdk2_sidecar first.");
  }
  const service = typeof options.service === "string" ? options.service : "sport";
  const method = typeof options.method === "string" ? options.method : "get_fsm_id";
  const params = options.params && typeof options.params === "object" && !Array.isArray(options.params) ? options.params : {};
  const timeoutSeconds = clampNumber(options.timeout_seconds, 0.2, 10, 1);
  const env = [
    "CYBER_UNITREE_ACTION=command_unitree_rpc_bridge",
    `CYBER_UNITREE_RPC_BRIDGE_SERVICE=${service}`,
    `CYBER_UNITREE_RPC_BRIDGE_METHOD=${method}`,
    `CYBER_UNITREE_RPC_BRIDGE_PARAMS=${JSON.stringify(params)}`,
    `CYBER_UNITREE_RPC_BRIDGE_TIMEOUT=${timeoutSeconds}`,
  ];
  const args = [...sdk2ComposeArgs(), "run", "--rm"];
  for (const entry of env) {
    args.push("-e", entry);
  }
  args.push("unitree-g1-sdk2-sidecar");
  const runResult = runChecked("docker", args, { timeoutMs: 120_000 });
  let report = null;
  const jsonStart = runResult.stdout.indexOf("{");
  const jsonEnd = runResult.stdout.lastIndexOf("}");
  if (jsonStart !== -1 && jsonEnd > jsonStart) {
    try {
      report = JSON.parse(runResult.stdout.slice(jsonStart, jsonEnd + 1));
    } catch {
      report = null;
    }
  }
  const commandReport = report?.rpc_bridge_command ?? null;
  const calls = commandReport?.calls ?? [];
  const summary = summarizeRpcBridgeCalls(calls);
  const result = {
    ok: commandReport?.ok === true && summary.all_calls_ok,
    source: "managed_unitree_sdk2_rpc_bridge",
    started,
    status_before: statusBefore,
    status,
    service,
    method,
    params,
    command: `docker ${args.join(" ")}`,
    stdout: runResult.stdout,
    stderr: runResult.stderr,
    report,
    command_report: commandReport,
    summary,
  };
  if (stopAfter) {
    stopped = sdk2StopRpcBridge();
    result.stopped = stopped;
  }
  return result;
}

function summarizeRpcBridgeCalls(calls) {
  const normalized = [];
  const statusCounts = {};
  for (const call of calls || []) {
    const body = rpcCallBody(call);
    const status = call?.rpc_status?.name ?? null;
    if (status) {
      statusCounts[status] = (statusCounts[status] || 0) + 1;
    }
    const forward = body?.simulator_forward;
    const readback = body?.simulator_readback;
    normalized.push({
      name: call?.name ?? null,
      ok: call?.ok === true,
      rpc_status: status,
      data: body && Object.prototype.hasOwnProperty.call(body, "data") ? body.data : null,
      forward_provider: forward?.provider ?? null,
      forward_ok: typeof forward?.ok === "boolean" ? forward.ok : null,
      readback_provider: readback?.provider ?? null,
      readback_ok: typeof readback?.ok === "boolean" ? readback.ok : null,
    });
  }
  const simulatorForwards = normalized.filter(
    (call) => call.forward_provider === "cybernetic_game_control_http" && call.forward_ok === true,
  );
  const simulatorReadbacks = normalized.filter(
    (call) => call.readback_provider === "cybernetic_game_control_http" && call.readback_ok === true,
  );
  const bridgeStateOnly = normalized.filter(
    (call) => call.forward_provider === "bridge_state_only" || call.readback_provider === "bridge_state_only",
  );
  return {
    call_count: normalized.length,
    all_calls_ok: normalized.length > 0 && normalized.every((call) => call.ok),
    rpc_status_counts: statusCounts,
    simulator_forward_count: simulatorForwards.length,
    simulator_readback_count: simulatorReadbacks.length,
    bridge_state_only_count: bridgeStateOnly.length,
    calls: normalized,
    simulator_forwards: simulatorForwards,
    simulator_readbacks: simulatorReadbacks,
  };
}

function rpcCallBody(call) {
  const value = call?.return;
  if (!Array.isArray(value) || value.length < 2 || typeof value[1] !== "string") {
    return {};
  }
  try {
    const parsed = JSON.parse(value[1]);
    return parsed && typeof parsed === "object" && !Array.isArray(parsed) ? parsed : {};
  } catch {
    return {};
  }
}

function sdk2ManagedJsonLogContainerStatus(container, readyAction, exitAction) {
  const inspect = run("docker", ["inspect", container, "--format", "{{json .State}}"], { timeoutMs: 30_000 });
  const logs = run("docker", ["logs", "--tail", "2000", container], { timeoutMs: 30_000 });
  let state = null;
  if (inspect.status === 0 && inspect.stdout.trim()) {
    try {
      state = JSON.parse(inspect.stdout.trim());
    } catch {
      state = null;
    }
  }
  let reports = logs.status === 0 ? parseJsonObjects(logs.stdout) : [];
  let lifecycleSource = "tail";
  if (state?.Running === true && reports.length === 0 && logs.status === 0) {
    const fullLogs = run("docker", ["logs", container], { timeoutMs: 30_000 });
    if (fullLogs.status === 0) {
      reports = parseJsonObjects(fullLogs.stdout);
      lifecycleSource = "full_logs_fallback";
    }
  }
  const readyReport = reports.find((report) => report?.action === readyAction) ?? reports[0] ?? null;
  const exitReport = [...reports].reverse().find((report) => report?.action === exitAction) ?? null;
  const lastReport = reports.length > 0 ? reports[reports.length - 1] : null;
  const running = state?.Running === true;
  return {
    container,
    exists: inspect.status === 0,
    running,
    status: state?.Status ?? null,
    exit_code: state?.ExitCode ?? null,
    started_at: state?.StartedAt ?? null,
    finished_at: state?.FinishedAt ?? null,
    inspect_error: inspect.status === 0 ? null : inspect.stderr.trim(),
    ready_report: readyReport,
    last_report: lastReport,
    exit_report: exitReport,
    lifecycle_reports_seen: reports.length,
    lifecycle_report_source: lifecycleSource,
    ready: running && readyReport?.ok === true && !exitReport,
    logs_tail: logs.status === 0 ? logs.stdout : null,
    logs_error: logs.status === 0 ? null : logs.stderr.trim(),
  };
}

function parseFirstJsonObject(text) {
  const start = text.indexOf("{");
  if (start === -1) {
    return null;
  }
  let depth = 0;
  let inString = false;
  let escaped = false;
  for (let index = start; index < text.length; index += 1) {
    const char = text[index];
    if (inString) {
      if (escaped) {
        escaped = false;
      } else if (char === "\\") {
        escaped = true;
      } else if (char === "\"") {
        inString = false;
      }
      continue;
    }
    if (char === "\"") {
      inString = true;
    } else if (char === "{") {
      depth += 1;
    } else if (char === "}") {
      depth -= 1;
      if (depth === 0) {
        try {
          return JSON.parse(text.slice(start, index + 1));
        } catch {
          return null;
        }
      }
    }
  }
  return null;
}

function parseJsonObjects(text) {
  const reports = [];
  let cursor = 0;
  while (cursor < text.length) {
    const start = text.indexOf("{", cursor);
    if (start === -1) {
      break;
    }
    let depth = 0;
    let inString = false;
    let escaped = false;
    let consumed = false;
    for (let index = start; index < text.length; index += 1) {
      const char = text[index];
      if (inString) {
        if (escaped) {
          escaped = false;
        } else if (char === "\\") {
          escaped = true;
        } else if (char === "\"") {
          inString = false;
        }
        continue;
      }
      if (char === "\"") {
        inString = true;
      } else if (char === "{") {
        depth += 1;
      } else if (char === "}") {
        depth -= 1;
        if (depth === 0) {
          try {
            const report = JSON.parse(text.slice(start, index + 1));
            if (report && typeof report === "object" && !Array.isArray(report)) {
              reports.push(report);
            }
          } catch {
            // Keep scanning; CycloneDDS and MuJoCo logs can contain brace-like noise.
          }
          cursor = index + 1;
          consumed = true;
          break;
        }
      }
    }
    if (!consumed) {
      break;
    }
  }
  return reports;
}

function sdk2ProbeOfficialMujocoDds() {
  const envPath = sdk2ComposeEnvPath();
  if (!fs.existsSync(envPath)) {
    throw new Error("Missing SDK2 sidecar compose env. Run unitree_prepare_sdk2_sidecar first.");
  }
  const actionEnv = "CYBER_UNITREE_ACTION=probe_official_mujoco_dds";
  const args = [...sdk2ComposeArgs(), "run", "--rm", "-e", actionEnv, "unitree-g1-sdk2-sidecar"];
  const result = runChecked("docker", args, { timeoutMs: 300_000 });
  let report = null;
  const jsonStart = result.stdout.indexOf("{");
  const jsonEnd = result.stdout.lastIndexOf("}");
  if (jsonStart !== -1 && jsonEnd > jsonStart) {
    try {
      report = JSON.parse(result.stdout.slice(jsonStart, jsonEnd + 1));
    } catch {
      report = null;
    }
  }
  return {
    command: `docker ${args.join(" ")}`,
    stdout: result.stdout,
    stderr: result.stderr,
    report,
  };
}

function sdk2ProbeOfficialMujocoLowcmd() {
  const envPath = sdk2ComposeEnvPath();
  if (!fs.existsSync(envPath)) {
    throw new Error("Missing SDK2 sidecar compose env. Run unitree_prepare_sdk2_sidecar first.");
  }
  const actionEnv = "CYBER_UNITREE_ACTION=probe_official_mujoco_lowcmd";
  const args = [...sdk2ComposeArgs(), "run", "--rm", "-e", actionEnv, "unitree-g1-sdk2-sidecar"];
  const result = runChecked("docker", args, { timeoutMs: 300_000 });
  let report = null;
  const jsonStart = result.stdout.indexOf("{");
  const jsonEnd = result.stdout.lastIndexOf("}");
  if (jsonStart !== -1 && jsonEnd > jsonStart) {
    try {
      report = JSON.parse(result.stdout.slice(jsonStart, jsonEnd + 1));
    } catch {
      report = null;
    }
  }
  return {
    command: `docker ${args.join(" ")}`,
    stdout: result.stdout,
    stderr: result.stderr,
    report,
  };
}

function sdk2ProbeOfficialMujocoArmMotion(options = {}) {
  const envPath = sdk2ComposeEnvPath();
  if (!fs.existsSync(envPath)) {
    throw new Error("Missing SDK2 sidecar compose env. Run unitree_prepare_sdk2_sidecar first.");
  }
  const joint = OFFICIAL_G1_ARM_JOINTS.includes(options.joint) ? options.joint : "right_shoulder_roll";
  const delta = clampNumber(options.delta, -0.5, 0.5, -0.25);
  const frames = clampInt(options.frames, 20, 600, 220);
  const kp = clampNumber(options.kp, 0, 80, 35.0);
  const kd = clampNumber(options.kd, 0, 5, 1.2);
  const holdKp = clampNumber(options.hold_kp, 0, 80, 18.0);
  const holdKd = clampNumber(options.hold_kd, 0, 5, 0.8);
  const env = [
    "CYBER_UNITREE_ACTION=probe_official_mujoco_arm_motion",
    `CYBER_UNITREE_ARM_MOTION_JOINT=${joint}`,
    `CYBER_UNITREE_ARM_MOTION_DELTA=${delta}`,
    `CYBER_UNITREE_ARM_MOTION_FRAMES=${frames}`,
    `CYBER_UNITREE_ARM_MOTION_KP=${kp}`,
    `CYBER_UNITREE_ARM_MOTION_KD=${kd}`,
    `CYBER_UNITREE_ARM_MOTION_HOLD_KP=${holdKp}`,
    `CYBER_UNITREE_ARM_MOTION_HOLD_KD=${holdKd}`,
  ];
  const args = [...sdk2ComposeArgs(), "run", "--rm", ...env.flatMap((entry) => ["-e", entry]), "unitree-g1-sdk2-sidecar"];
  const result = runChecked("docker", args, { timeoutMs: 300_000 });
  let report = null;
  const jsonStart = result.stdout.indexOf("{");
  const jsonEnd = result.stdout.lastIndexOf("}");
  if (jsonStart !== -1 && jsonEnd > jsonStart) {
    try {
      report = JSON.parse(result.stdout.slice(jsonStart, jsonEnd + 1));
    } catch {
      report = null;
    }
  }
  return {
    command: `docker ${args.join(" ")}`,
    parameters: { joint, delta, frames, kp, kd, hold_kp: holdKp, hold_kd: holdKd },
    stdout: result.stdout,
    stderr: result.stderr,
    report,
  };
}

function sdk2ProbeOfficialMujocoArmPose(options = {}) {
  const envPath = sdk2ComposeEnvPath();
  if (!fs.existsSync(envPath)) {
    throw new Error("Missing SDK2 sidecar compose env. Run unitree_prepare_sdk2_sidecar first.");
  }
  const preset = OFFICIAL_G1_ARM_POSE_PRESETS.includes(options.preset) ? options.preset : "raise_right_hand";
  const jointDeltas = normalizeJointDeltas(options.joint_deltas);
  const frames = clampInt(options.frames, 20, 600, 180);
  const kp = clampNumber(options.kp, 0, 80, 30.0);
  const kd = clampNumber(options.kd, 0, 5, 1.0);
  const holdKp = clampNumber(options.hold_kp, 0, 80, 18.0);
  const holdKd = clampNumber(options.hold_kd, 0, 5, 0.8);
  const minMovedJoints = clampInt(options.min_moved_joints, 1, 8, 2);
  const env = [
    "CYBER_UNITREE_ACTION=probe_official_mujoco_arm_pose",
    `CYBER_UNITREE_ARM_POSE_PRESET=${preset}`,
    `CYBER_UNITREE_ARM_POSE_FRAMES=${frames}`,
    `CYBER_UNITREE_ARM_POSE_KP=${kp}`,
    `CYBER_UNITREE_ARM_POSE_KD=${kd}`,
    `CYBER_UNITREE_ARM_POSE_HOLD_KP=${holdKp}`,
    `CYBER_UNITREE_ARM_POSE_HOLD_KD=${holdKd}`,
    `CYBER_UNITREE_ARM_POSE_MIN_MOVED_JOINTS=${minMovedJoints}`,
  ];
  if (Object.keys(jointDeltas).length > 0) {
    env.push(`CYBER_UNITREE_ARM_POSE_DELTAS=${JSON.stringify(jointDeltas)}`);
  }
  const args = [...sdk2ComposeArgs(), "run", "--rm", ...env.flatMap((entry) => ["-e", entry]), "unitree-g1-sdk2-sidecar"];
  const result = runChecked("docker", args, { timeoutMs: 300_000 });
  let report = null;
  const jsonStart = result.stdout.indexOf("{");
  const jsonEnd = result.stdout.lastIndexOf("}");
  if (jsonStart !== -1 && jsonEnd > jsonStart) {
    try {
      report = JSON.parse(result.stdout.slice(jsonStart, jsonEnd + 1));
    } catch {
      report = null;
    }
  }
  return {
    command: `docker ${args.join(" ")}`,
    parameters: { preset, joint_deltas: jointDeltas, frames, kp, kd, hold_kp: holdKp, hold_kd: holdKd, min_moved_joints: minMovedJoints },
    stdout: result.stdout,
    stderr: result.stderr,
    report,
  };
}

function sdk2CommandOfficialMujocoArmPose(options = {}) {
  const envPath = sdk2ComposeEnvPath();
  if (!fs.existsSync(envPath)) {
    throw new Error("Missing SDK2 sidecar compose env. Run unitree_prepare_sdk2_sidecar first.");
  }
  const session = sdk2OfficialMujocoSessionStatus();
  if (!session.running || !session.ready) {
    throw new Error("Managed official MuJoCo session is not running and ready. Run unitree_start_official_mujoco_session first.");
  }
  const preset = OFFICIAL_G1_ARM_POSE_PRESETS.includes(options.preset) ? options.preset : "raise_right_hand";
  const jointDeltas = normalizeJointDeltas(options.joint_deltas);
  const frames = clampInt(options.frames, 20, 600, 180);
  const kp = clampNumber(options.kp, 0, 80, 30.0);
  const kd = clampNumber(options.kd, 0, 5, 1.0);
  const holdKp = clampNumber(options.hold_kp, 0, 80, 18.0);
  const holdKd = clampNumber(options.hold_kd, 0, 5, 0.8);
  const minMovedJoints = clampInt(options.min_moved_joints, 1, 8, 2);
  const env = [
    "CYBER_UNITREE_ACTION=command_official_mujoco_arm_pose",
    `CYBER_UNITREE_ARM_POSE_PRESET=${preset}`,
    `CYBER_UNITREE_ARM_POSE_FRAMES=${frames}`,
    `CYBER_UNITREE_ARM_POSE_KP=${kp}`,
    `CYBER_UNITREE_ARM_POSE_KD=${kd}`,
    `CYBER_UNITREE_ARM_POSE_HOLD_KP=${holdKp}`,
    `CYBER_UNITREE_ARM_POSE_HOLD_KD=${holdKd}`,
    `CYBER_UNITREE_ARM_POSE_MIN_MOVED_JOINTS=${minMovedJoints}`,
  ];
  if (Object.keys(jointDeltas).length > 0) {
    env.push(`CYBER_UNITREE_ARM_POSE_DELTAS=${JSON.stringify(jointDeltas)}`);
  }
  const args = [...sdk2ComposeArgs(), "run", "--rm", ...env.flatMap((entry) => ["-e", entry]), "unitree-g1-sdk2-sidecar"];
  const result = runChecked("docker", args, { timeoutMs: 300_000 });
  let report = null;
  const jsonStart = result.stdout.indexOf("{");
  const jsonEnd = result.stdout.lastIndexOf("}");
  if (jsonStart !== -1 && jsonEnd > jsonStart) {
    try {
      report = JSON.parse(result.stdout.slice(jsonStart, jsonEnd + 1));
    } catch {
      report = null;
    }
  }
  return {
    command: `docker ${args.join(" ")}`,
    parameters: { preset, joint_deltas: jointDeltas, frames, kp, kd, hold_kp: holdKp, hold_kd: holdKd, min_moved_joints: minMovedJoints },
    session,
    stdout: result.stdout,
    stderr: result.stderr,
    report,
  };
}

function sdk2CommandOfficialMujocoLowcmd(options = {}) {
  const envPath = sdk2ComposeEnvPath();
  if (!fs.existsSync(envPath)) {
    throw new Error("Missing SDK2 sidecar compose env. Run unitree_prepare_sdk2_sidecar first.");
  }
  const session = sdk2OfficialMujocoSessionStatus();
  if (!session.running || !session.ready) {
    throw new Error("Managed official MuJoCo session is not running and ready. Run unitree_start_official_mujoco_session first.");
  }
  const motorCmd = Array.isArray(options.motor_cmd)
    ? options.motor_cmd.slice(0, 35).map((item) => ({
        ...(Number.isFinite(Number(item?.mode)) ? { mode: clampInt(item.mode, 0, 15, 1) } : {}),
        ...(Number.isFinite(Number(item?.q)) ? { q: clampNumber(item.q, -3.5, 3.5, 0) } : {}),
        ...(Number.isFinite(Number(item?.dq)) ? { dq: clampNumber(item.dq, -20, 20, 0) } : {}),
        ...(Number.isFinite(Number(item?.tau)) ? { tau: clampNumber(item.tau, -80, 80, 0) } : {}),
        ...(Number.isFinite(Number(item?.kp)) ? { kp: clampNumber(item.kp, 0, 80, 0) } : {}),
        ...(Number.isFinite(Number(item?.kd)) ? { kd: clampNumber(item.kd, 0, 8, 0) } : {}),
      }))
    : [];
  const payload = {
    motor_cmd: motorCmd,
    mode_pr: clampInt(options.mode_pr, 0, 255, 0),
    mode_machine: clampInt(options.mode_machine, 0, 1000, 0),
    crc: clampInt(options.crc, 0, 0xffffffff, 0),
  };
  const topic = ["rt/lowcmd", "rt/arm_sdk"].includes(options.topic) ? options.topic : "rt/lowcmd";
  const frames = clampInt(options.frames, 1, 60, 1);
  const timeoutSeconds = clampNumber(options.timeout_seconds, 0.5, 30, 6);
  const env = [
    "CYBER_UNITREE_ACTION=command_official_mujoco_lowcmd",
    `CYBER_UNITREE_LOWCMD_TOPIC=${topic}`,
    `CYBER_UNITREE_LOWCMD_JSON=${JSON.stringify(payload)}`,
    `CYBER_UNITREE_LOWCMD_FRAMES=${frames}`,
    `CYBER_UNITREE_LOWCMD_TIMEOUT=${timeoutSeconds}`,
  ];
  const args = [...sdk2ComposeArgs(), "run", "--rm", ...env.flatMap((entry) => ["-e", entry]), "unitree-g1-sdk2-sidecar"];
  const result = runChecked("docker", args, { timeoutMs: 180_000 });
  let report = null;
  const jsonStart = result.stdout.indexOf("{");
  const jsonEnd = result.stdout.lastIndexOf("}");
  if (jsonStart !== -1 && jsonEnd > jsonStart) {
    try {
      report = JSON.parse(result.stdout.slice(jsonStart, jsonEnd + 1));
    } catch {
      report = null;
    }
  }
  return {
    command: `docker ${args.join(" ")}`,
    parameters: { topic, ...payload, frames, timeout_seconds: timeoutSeconds },
    session,
    stdout: result.stdout,
    stderr: result.stderr,
    report,
  };
}

async function sdk2OfficialMujocoEvidenceBundle(options = {}) {
  let session = sdk2OfficialMujocoSessionStatus();
  let startedByTool = false;
  let startResult = null;
  if ((!session.running || !session.ready) && options.start_if_needed !== false) {
    startResult = sdk2StartOfficialMujocoSession();
    startedByTool = true;
    session = startResult.status;
  }
  if (!session.running || !session.ready) {
    throw new Error("Managed official MuJoCo session is not running and ready. Run unitree_start_official_mujoco_session first.");
  }

  const before = sdk2ReadOfficialMujocoLowstate();
  const commandResult = sdk2CommandOfficialMujocoArmPose(options);
  const after = sdk2ReadOfficialMujocoLowstate();
  let stopResult = null;
  if (startedByTool && options.stop_after === true) {
    stopResult = sdk2StopOfficialMujocoSession();
  }

  const bundle = {
    ok: Boolean(
      before.report?.lowstate_read?.ok === true &&
      commandResult.report?.arm_pose_command?.ok === true &&
      after.report?.lowstate_read?.ok === true
    ),
    source: "official_unitree_mujoco_managed_session",
    started_by_tool: startedByTool,
    start_result: startResult,
    session_before_command: session,
    preset: commandResult.parameters.preset,
    parameters: commandResult.parameters,
    before: before.report?.lowstate_read?.lowstate_summary ?? null,
    command: {
      ok: commandResult.report?.arm_pose_command?.ok === true,
      moved_joints: commandResult.report?.arm_pose_command?.moved_joints ?? [],
      lowcmd_write_successes: commandResult.report?.arm_pose_command?.lowcmd_write_successes ?? null,
      report: commandResult.report?.arm_pose_command ?? null,
    },
    after: after.report?.lowstate_read?.lowstate_summary ?? null,
    stopped_after: stopResult,
    agent_hints: [
      "Use before/after official rt/lowstate summaries as the primary motion evidence.",
      "This tool commands the official Unitree MuJoCo simulator over SDK2/CycloneDDS; it does not unlock physical hardware.",
      "If ok is false, inspect command.report and the sidecar stdout/stderr returned by the lower-level tools.",
    ],
  };
  const outputPath = safeWorkspacePath(options.output_path || ".runtime/official-mujoco-evidence/latest.json");
  await fsp.mkdir(path.dirname(outputPath), { recursive: true });
  await fsp.writeFile(outputPath, `${JSON.stringify(bundle, null, 2)}\n`);
  return {
    ok: bundle.ok,
    path: outputPath,
    workspace_relative_path: path.relative(root, outputPath),
    preset: bundle.preset,
    moved_joints: bundle.command.moved_joints,
    lowcmd_write_successes: bundle.command.lowcmd_write_successes,
    before: bundle.before,
    after: bundle.after,
    started_by_tool: startedByTool,
    stopped_after: stopResult?.removed === true,
  };
}

function normalizeJointDeltas(value) {
  if (!value || typeof value !== "object" || Array.isArray(value)) {
    return {};
  }
  const result = {};
  const unknownJoints = [];
  for (const [joint, delta] of Object.entries(value)) {
    if (!OFFICIAL_G1_ARM_JOINTS.includes(joint)) {
      unknownJoints.push(joint);
      continue;
    }
    result[joint] = clampNumber(delta, -0.5, 0.5, 0);
  }
  if (unknownJoints.length > 0) {
    throw new Error(`Unknown official G1 arm joint(s): ${unknownJoints.join(", ")}`);
  }
  return result;
}

function clampNumber(value, min, max, fallback) {
  const number = Number(value ?? fallback);
  if (!Number.isFinite(number)) {
    return fallback;
  }
  return Math.min(max, Math.max(min, number));
}

function clampInt(value, min, max, fallback) {
  return Math.round(clampNumber(value, min, max, fallback));
}

function readComposeEnv() {
  const file = composeEnvPath();
  if (!fs.existsSync(file)) {
    return {};
  }
  const env = {};
  for (const line of fs.readFileSync(file, "utf8").split(/\r?\n/)) {
    if (!line.trim() || line.trim().startsWith("#")) {
      continue;
    }
    const index = line.indexOf("=");
    if (index === -1) {
      continue;
    }
    env[line.slice(0, index)] = line.slice(index + 1);
  }
  return env;
}

function updateComposeEnv(updates) {
  const current = readComposeEnv();
  const next = { ...current, ...updates };
  const keys = [
    "UNITREE_G1_MUJOCO_IMAGE",
    "UNITREE_G1_MUJOCO_PLATFORM",
    "UNITREE_G1_MUJOCO_ASSET_ROOT",
    "UNITREE_G1_MODEL_PATH",
    "UNITREE_G1_MODEL_REVISION",
    "UNITREE_G1_ROBOT_NAME",
    "UNITREE_G1_AUTORUN",
    "UNITREE_G1_FRAME_HZ",
    "UNITREE_G1_RENDER_HZ",
    "UNITREE_G1_RENDER_WIDTH",
    "UNITREE_G1_RENDER_HEIGHT",
  ];
  const lines = keys.filter((key) => next[key] !== undefined).map((key) => `${key}=${next[key]}`);
  fs.writeFileSync(composeEnvPath(), `${lines.join(os.EOL)}${os.EOL}`);
}

function runtimePaths() {
  const env = readComposeEnv();
  const assetRoot = env.UNITREE_G1_MUJOCO_ASSET_ROOT || path.join(root, ".runtime/unitree-g1-mujoco/unitree_mujoco");
  const containerModelPath = env.UNITREE_G1_MODEL_PATH || "/opt/unitree_mujoco/unitree_robots/g1/scene_29dof.xml";
  if (!containerModelPath.startsWith("/opt/unitree_mujoco/")) {
    throw new Error(`Unsupported model path outside /opt/unitree_mujoco: ${containerModelPath}`);
  }
  const relativeModelPath = containerModelPath.slice("/opt/unitree_mujoco/".length);
  return {
    assetRoot,
    containerModelPath,
    hostModelPath: path.join(assetRoot, relativeModelPath),
  };
}

function resolveSceneHostPath(userPath, paths = runtimePaths()) {
  if (!userPath) {
    return paths.hostModelPath;
  }
  const value = String(userPath);
  if (value.startsWith("/opt/unitree_mujoco/")) {
    return path.join(paths.assetRoot, value.slice("/opt/unitree_mujoco/".length));
  }
  const absolute = path.isAbsolute(value) ? path.resolve(value) : safeWorkspacePath(value);
  const assetRoot = path.resolve(paths.assetRoot);
  if (absolute !== assetRoot && !absolute.startsWith(`${assetRoot}${path.sep}`)) {
    throw new Error(`Scene path must stay inside the mounted Unitree asset tree: ${userPath}`);
  }
  return absolute;
}

function runChecked(command, args, options = {}) {
  const result = run(command, args, options);
  if (result.status !== 0) {
    throw new Error(`${command} ${args.join(" ")} failed with status ${result.status}\nstdout:\n${result.stdout}\nstderr:\n${result.stderr}`);
  }
  return result;
}

function run(command, args, options = {}) {
  const result = spawnSync(command, args, {
    cwd: root,
    env: options.env || process.env,
    encoding: "utf8",
    timeout: options.timeoutMs || 60000,
    maxBuffer: 10 * 1024 * 1024,
  });
  return {
    command,
    args,
    status: result.status,
    signal: result.signal,
    stdout: result.stdout || "",
    stderr: result.stderr || "",
    error: result.error ? result.error.message : null,
  };
}

function pythonEnv() {
  const shim = path.join(root, "overlays/unitree-g1-sdk-shim");
  return {
    ...process.env,
    PYTHONPATH: process.env.PYTHONPATH ? `${shim}${path.delimiter}${process.env.PYTHONPATH}` : shim,
    CYBER_G1_GAME_CONTROL_URL: gameControlUrl(),
    CYBER_G1_WS_HOST: new URL(physicsUrl()).hostname,
    CYBER_G1_WS_PORT: String(new URL(physicsUrl()).port || 8788),
  };
}

function safeWorkspacePath(userPath) {
  if (!userPath || typeof userPath !== "string") {
    throw new Error("Expected a workspace-relative path");
  }
  const absolute = path.resolve(root, userPath);
  if (absolute !== root && !absolute.startsWith(`${root}${path.sep}`)) {
    throw new Error(`Path escapes workspace: ${userPath}`);
  }
  return absolute;
}

function sdkPythonTemplate(action, args = {}) {
  if (action === "locomotion") {
    return locomotionPythonTemplate();
  }
  if (action === "lowcmd_joint_target") {
    return lowcmdJointTargetPythonTemplate();
  }
  if (action === "scene_edit") {
    return sceneEditPythonTemplate();
  }
  if (action === "telemetry_monitor") {
    return telemetryMonitorPythonTemplate();
  }

  const actionName =
    action === "release_arm"
      ? "release arm"
      : action === "arm_action"
        ? String(args.sdk_action || "right hand up")
        : "right hand up";
  return `#!/usr/bin/env python3
from unitree_sdk2py.core.channel import ChannelFactoryInitialize
from unitree_sdk2py.g1.arm.g1_arm_action_client import G1ArmActionClient, action_map


def main():
    ChannelFactoryInitialize(0)
    client = G1ArmActionClient()
    client.Init()
    client.SetTimeout(10.0)
    action_id = action_map["${actionName}"]
    result = client.ExecuteAction(action_id)
    if result != 0:
        raise SystemExit(f"G1 action failed with status {result}")
    print("G1 action complete: ${actionName}")


if __name__ == "__main__":
    main()
`;
}

function locomotionPythonTemplate() {
  return `#!/usr/bin/env python3
import time

from cybernetic_robotics import G1Robot
from unitree_sdk2py.core.channel import ChannelFactoryInitialize
from unitree_sdk2py.g1.loco.g1_loco_client import LocoClient


def main():
    ChannelFactoryInitialize(0, "cyber-sim")
    loco = LocoClient()
    loco.SetTimeout(10.0)
    loco.Init()

    with G1Robot.connect() as robot:
        robot.reset()
        robot.reset_camera()
        robot.snapshot(".runtime/scaffolded-locomotion-before.jpg")

        assert loco.Start() == 0
        assert loco.Move(0.2, 0.0, 0.0) == 0
        time.sleep(0.75)
        assert loco.StopMove() == 0

        robot.snapshot(".runtime/scaffolded-locomotion-after.jpg")
        robot.safety_stop()

    print("G1 locomotion scaffold complete")


if __name__ == "__main__":
    main()
`;
}

function lowcmdJointTargetPythonTemplate() {
  return `#!/usr/bin/env python3
from cybernetic_robotics import G1Robot


def main():
    targets = {
        "right_shoulder_pitch_joint": -1.05,
        "right_shoulder_roll_joint": -0.25,
        "right_elbow_joint": 0.85,
    }

    with G1Robot.connect() as robot:
        robot.reset()
        result = robot.apply_joint_targets(targets, kp=38.0, kd=1.4)
        robot.snapshot(".runtime/scaffolded-joint-targets.jpg")
        print(result)
        robot.safety_stop()


if __name__ == "__main__":
    main()
`;
}

function sceneEditPythonTemplate() {
  return `#!/usr/bin/env python3
from cybernetic_robotics import SceneWorkspace


def main():
    scene = SceneWorkspace.discover()
    host_path, container_path = scene.add_box(
        "agent_box",
        position=(0.85, 0.0, 0.08),
        size=(0.12, 0.12, 0.08),
        rgba=(0.9, 0.18, 0.1, 1.0),
        activate=False,
    )
    print({"host_path": str(host_path), "container_path": container_path})
    print("Set activate=True and restart the simulator when you are ready to boot this scene.")


if __name__ == "__main__":
    main()
`;
}

function telemetryMonitorPythonTemplate() {
  return `#!/usr/bin/env python3
import json
import time

from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelSubscriber
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowState_


def main():
    ChannelFactoryInitialize(0, "cyber-sim")
    sub = ChannelSubscriber("rt/lowstate", LowState_)
    sub.Init()

    for index in range(5):
        lowstate = sub.Read()
        sample = {
            "sample": index,
            "mode_machine": lowstate.mode_machine,
            "imu_quaternion": list(lowstate.imu_state.quaternion),
            "first_motor_q": lowstate.motor_state[0].q if lowstate.motor_state else None,
        }
        print(json.dumps(sample))
        time.sleep(0.25)


if __name__ == "__main__":
    main()
`;
}

function gameControlUrl() {
  return process.env.CYBER_G1_GAME_CONTROL_URL || DEFAULT_GAME_CONTROL_URL;
}

function physicsUrl() {
  return process.env.CYBER_G1_PHYSICS_URL || DEFAULT_WS_URL;
}

function findRepoRoot(start) {
  let current = path.resolve(start);
  while (true) {
    if (fs.existsSync(path.join(current, "overlays/unitree-g1-mujoco-protocol/Dockerfile"))) {
      return current;
    }
    const parent = path.dirname(current);
    if (parent === current) {
      return path.resolve(start);
    }
    current = parent;
  }
}

function tool(name, description, properties, required = [], annotations = {}) {
  return {
    name,
    title: name,
    description,
    inputSchema: {
      type: "object",
      additionalProperties: false,
      properties,
      required,
    },
    annotations,
  };
}

function textResult(value) {
  const text = typeof value === "string" ? value : JSON.stringify(value, null, 2);
  return {
    content: [{ type: "text", text }],
    structuredContent: typeof value === "object" ? value : undefined,
  };
}

function imageResult(snapshotValue) {
  return {
    content: [
      { type: "text", text: JSON.stringify(snapshotValue.metadata, null, 2) },
      { type: "image", data: snapshotValue.data, mimeType: snapshotValue.mimeType },
    ],
    structuredContent: snapshotValue.metadata,
  };
}

function toolError(message) {
  return { content: [{ type: "text", text: message }], isError: true };
}

function respond(id, result) {
  process.stdout.write(`${JSON.stringify({ jsonrpc: "2.0", id, result })}\n`);
}

function respondError(id, code, message) {
  process.stdout.write(`${JSON.stringify({ jsonrpc: "2.0", id, error: { code, message } })}\n`);
}

function toInt(value, fallback) {
  const parsed = Number.parseInt(value, 10);
  return Number.isFinite(parsed) ? parsed : fallback;
}

function normalizeChoice(value, allowed, fallback) {
  if (value === undefined || value === null || value === "") return fallback;
  const normalized = String(value).trim().toLowerCase().replaceAll("-", "_");
  return allowed.includes(normalized) ? normalized : fallback;
}

function numericArray(value, length, name) {
  if (!Array.isArray(value) || value.length !== length) {
    throw new Error(`${name} must be an array of ${length} numbers`);
  }
  return value.map((item) => {
    const number = Number(item);
    if (!Number.isFinite(number)) {
      throw new Error(`${name} contains a non-number`);
    }
    return number;
  });
}

function escapeXml(value) {
  return String(value).replaceAll("&", "&amp;").replaceAll('"', "&quot;").replaceAll("<", "&lt;").replaceAll(">", "&gt;");
}

function parseCyberneticObjects(xml) {
  const objects = [];
  const pattern =
    /<body name="([^"]+)" pos="([^"]+)">\s*<geom name="\1_geom" type="box" size="([^"]+)" rgba="([^"]+)"\/>\s*<\/body>/g;
  for (const match of xml.matchAll(pattern)) {
    objects.push({
      name: match[1],
      type: "box",
      position: match[2].split(/\s+/).map(Number),
      size: match[3].split(/\s+/).map(Number),
      rgba: match[4].split(/\s+/).map(Number),
    });
  }
  return objects;
}

function removeCyberneticObjectBlock(xml, name) {
  const escaped = escapeRegExp(name);
  const pattern = new RegExp(
    `\\n?\\s*<body name="${escaped}" pos="[^"]+">\\s*<geom name="${escaped}_geom" type="box" size="[^"]+" rgba="[^"]+"\\/>\\s*<\\/body>\\n?`,
    "m",
  );
  const next = xml.replace(pattern, "\n");
  return { xml: next, removed: next !== xml };
}

function escapeRegExp(value) {
  return String(value).replace(/[.*+?^${}()|[\]\\]/g, "\\$&");
}

function safeSegment(value) {
  const segment = String(value).trim().replace(/[^A-Za-z0-9_-]+/g, "-").replace(/^-+|-+$/g, "");
  return segment || "g1";
}

function tail(value, max) {
  return value.length <= max ? value : value.slice(-max);
}
