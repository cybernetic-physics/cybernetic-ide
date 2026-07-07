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
const DEFAULT_POSE = "raise_right_hand";
const MAX_LOG_BYTES = 256_000;

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
    "sim_apply_pose",
    "Apply a named simulator pose, such as raise_right_hand or neutral.",
    {
      pose: { type: "string", default: DEFAULT_POSE },
    },
    [],
    { readOnlyHint: false },
  ),
  tool(
    "viewer_camera_control",
    "Control the MuJoCo free camera through the simulator protocol.",
    {
      action: { type: "string", enum: ["state", "reset", "orbit", "pan", "zoom"] },
      dx: { type: "number", default: 0 },
      dy: { type: "number", default: 0 },
      delta: { type: "number", default: 0 },
    },
    ["action"],
    { readOnlyHint: false },
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
    "unitree_sdk_scaffold_python",
    "Generate or write a Unitree SDK-shaped Python control script for the local G1 simulator.",
    {
      path: { type: "string", description: "Optional workspace-relative file path to write." },
      action: { type: "string", enum: ["raise_hand", "release_arm"], default: "raise_hand" },
    },
    [],
    { readOnlyHint: false },
  ),
  tool("g1_list_actions", "List supported high-level G1 SDK facade actions.", {}, [], {
    readOnlyHint: true,
  }),
  tool(
    "g1_execute_action",
    "Execute a high-level G1 action through the Unitree SDK facade protocol.",
    {
      action: { type: "string", enum: ["raise_right_hand", "release_arm", "neutral"] },
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
        enum: ["state", "damp", "start", "zero_torque", "stop_move", "move", "low_stand", "high_stand", "wave_hand", "shake_hand"],
        default: "state",
      },
      vx: { type: "number", default: 0 },
      vy: { type: "number", default: 0 },
      omega: { type: "number", default: 0 },
      duration: { type: "number", default: 1.0 },
    },
    ["command"],
    { readOnlyHint: false },
  ),
  tool("safety_stop", "Pause the simulator and release the G1 arm to the neutral pose.", {}, [], {
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
    case "sim_pause":
      return textResult(await command({ command: "pause" }));
    case "sim_resume":
      return textResult(await command({ command: "resume" }));
    case "sim_reset":
      return textResult(await command({ command: "reset" }));
    case "sim_step":
      return textResult(await repeatStep(toInt(args.count, 1)));
    case "sim_apply_pose":
      return textResult(await command({ command: "pose", pose: args.pose || DEFAULT_POSE }));
    case "viewer_camera_control":
      return textResult(await camera(args));
    case "viewer_snapshot":
      return imageResult(await snapshot(args.format || "jpeg"));
    case "viewer_snapshot_file":
      return textResult(await snapshotFile(args.path, args.format || "jpeg"));
    case "scene_get":
      return textResult(await getJson("/visual_scene"));
    case "scene_read_mjcf":
      return textResult(await readActiveMjcf());
    case "scene_validate_mjcf":
      return textResult(validateMjcf(args.model_path));
    case "scene_add_box":
      return textResult(await addBoxToScene(args));
    case "unitree_sdk_scaffold_python":
      return textResult(await scaffoldPython(args));
    case "g1_list_actions":
      return textResult({
        actions: [
          { action: "raise_right_hand", sdk_action: "right hand up", action_id: 23 },
          { action: "release_arm", sdk_action: "release arm", action_id: 99 },
          { action: "neutral", sdk_action: "release arm", action_id: 99 },
        ],
      });
    case "g1_execute_action":
      return textResult(await executeG1Action(args.action));
    case "g1_loco_command":
      return textResult(await executeG1LocoCommand(args));
    case "safety_stop":
      return textResult({
        pause: await command({ command: "pause" }),
        neutral: await command({ command: "pose", pose: "neutral" }),
      });
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

async function repeatStep(count) {
  const results = [];
  for (let index = 0; index < Math.max(1, count); index += 1) {
    results.push(await command({ command: "step" }));
  }
  return { count: results.length, last: results.at(-1), results };
}

async function command(body) {
  return postJson("/command", body);
}

async function camera(args) {
  return postJson("/camera", {
    action: args.action || "state",
    dx: Number(args.dx || 0),
    dy: Number(args.dy || 0),
    delta: Number(args.delta || 0),
  });
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
  const actionByCommand = {
    damp: { action: "set_fsm_id", fsm_id: 1, mode: "damp" },
    start: { action: "set_fsm_id", fsm_id: 500, mode: "start" },
    zero_torque: { action: "set_fsm_id", fsm_id: 0, mode: "zero_torque" },
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

async function scaffoldPython(args) {
  const action = args.action === "release_arm" ? "release_arm" : "raise_hand";
  const code = sdkPythonTemplate(action);
  if (!args.path) {
    return { code };
  }

  const target = safeWorkspacePath(args.path);
  await fsp.mkdir(path.dirname(target), { recursive: true });
  await fsp.writeFile(target, code);
  return { path: target, code };
}

async function executeG1Action(action) {
  const pose = action === "release_arm" || action === "neutral" ? "neutral" : "raise_right_hand";
  return command({ command: "pose", pose });
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

function sdkPythonTemplate(action) {
  const actionName = action === "release_arm" ? "release arm" : "right hand up";
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

function tail(value, max) {
  return value.length <= max ? value : value.slice(-max);
}
