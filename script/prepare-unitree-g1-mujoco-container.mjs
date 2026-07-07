#!/usr/bin/env node
import { existsSync } from "node:fs";
import fs from "node:fs/promises";
import path from "node:path";
import { spawnSync } from "node:child_process";
import { fileURLToPath } from "node:url";

const root = path.resolve(path.dirname(fileURLToPath(import.meta.url)), "..");

const runtimeRoot = path.join(root, ".runtime/unitree-g1-mujoco");
const unitreeRepo = path.join(runtimeRoot, "unitree_mujoco");
const policyDir = path.join(runtimeRoot, "policy");
const unitreeRemote =
  process.env.UNITREE_G1_MUJOCO_REMOTE ||
  "https://github.com/unitreerobotics/unitree_mujoco.git";
const unitreeRevision =
  process.env.UNITREE_G1_MUJOCO_REVISION ||
  "ae6a8403e272733e9996ef59990880330496177f";
const image =
  process.env.UNITREE_G1_MUJOCO_IMAGE ||
  "cyber/unitree-g1-mujoco-protocol:0.1.0";
const platform = process.env.UNITREE_G1_MUJOCO_PLATFORM || "linux/arm64";
const modelPath =
  process.env.UNITREE_G1_MODEL_PATH ||
  "/opt/unitree_mujoco/unitree_robots/g1/scene_29dof.xml";

function run(command, args, options = {}) {
  const result = spawnSync(command, args, {
    cwd: options.cwd || root,
    stdio: options.stdio || "inherit",
    env: process.env,
  });
  if (result.status !== 0) {
    throw new Error(`${command} ${args.join(" ")} failed with status ${result.status}`);
  }
  return result;
}

function composeQuote(value) {
  return String(value).replaceAll("\\", "\\\\").replaceAll("\n", "\\n");
}

async function ensureUnitreeRepo() {
  await fs.mkdir(runtimeRoot, { recursive: true });
  if (existsSync(path.join(unitreeRepo, "unitree_robots/go2w"))) {
    await fs.rm(unitreeRepo, { recursive: true, force: true });
  }

  if (!existsSync(path.join(unitreeRepo, ".git"))) {
    await fs.rm(unitreeRepo, { recursive: true, force: true });
    run("git", [
      "clone",
      "--filter=blob:none",
      "--sparse",
      "--no-checkout",
      unitreeRemote,
      unitreeRepo,
    ]);
  } else {
    run("git", ["reset", "--hard"], { cwd: unitreeRepo });
  }

  run("git", ["fetch", "--depth", "1", "origin", unitreeRevision], {
    cwd: unitreeRepo,
  });
  run("git", ["sparse-checkout", "set", "unitree_robots/g1"], {
    cwd: unitreeRepo,
  });
  run("git", ["checkout", "--detach", "FETCH_HEAD"], { cwd: unitreeRepo });

  const model = path.join(unitreeRepo, "unitree_robots/g1/scene_29dof.xml");
  if (!existsSync(model)) {
    throw new Error(`Missing Unitree G1 scene after checkout: ${model}`);
  }
}

await ensureUnitreeRepo();
await fs.mkdir(policyDir, { recursive: true });

await fs.writeFile(
  path.join(runtimeRoot, "compose.env"),
  [
    `UNITREE_G1_MUJOCO_IMAGE=${composeQuote(image)}`,
    `UNITREE_G1_MUJOCO_PLATFORM=${composeQuote(platform)}`,
    `UNITREE_G1_MUJOCO_ASSET_ROOT=${composeQuote(unitreeRepo)}`,
    `UNITREE_G1_MODEL_PATH=${composeQuote(modelPath)}`,
    `UNITREE_G1_MODEL_REVISION=${composeQuote(
      `unitreerobotics/unitree_mujoco@${unitreeRevision}`,
    )}`,
    `UNITREE_G1_ROBOT_NAME=${composeQuote(process.env.UNITREE_G1_ROBOT_NAME || "g1")}`,
    `UNITREE_G1_AUTORUN=${composeQuote(process.env.UNITREE_G1_AUTORUN || "0")}`,
    `UNITREE_G1_FRAME_HZ=${composeQuote(process.env.UNITREE_G1_FRAME_HZ || "20")}`,
    `UNITREE_G1_RENDER_HZ=${composeQuote(process.env.UNITREE_G1_RENDER_HZ || "8")}`,
    `UNITREE_G1_HTTP_PORT=${composeQuote(process.env.UNITREE_G1_HTTP_PORT || "38383")}`,
    `UNITREE_G1_WS_PORT=${composeQuote(process.env.UNITREE_G1_WS_PORT || "8788")}`,
    `UNITREE_G1_RENDER_WIDTH=${composeQuote(
      process.env.UNITREE_G1_RENDER_WIDTH || "640",
    )}`,
    `UNITREE_G1_RENDER_HEIGHT=${composeQuote(
      process.env.UNITREE_G1_RENDER_HEIGHT || "480",
    )}`,
    `UNITREE_G1_POLICY_DIR_HOST=${composeQuote(process.env.UNITREE_G1_POLICY_DIR_HOST || policyDir)}`,
    `UNITREE_G1_POLICY_BUNDLE=${composeQuote(
      process.env.UNITREE_G1_POLICY_BUNDLE ||
        "/opt/unitree-g1-mujoco-protocol/policy/g1_yoga_policy.npz",
    )}`,
    "",
  ].join("\n"),
);

const composeFile = path.join(root, "overlays/unitree-g1-mujoco-container/compose.yaml");
console.log(`Prepared Unitree G1 MuJoCo runtime at ${runtimeRoot}`);
console.log(`Image: ${image}`);
console.log(`Unitree source: ${unitreeRemote}`);
console.log(`Unitree revision: ${unitreeRevision}`);
console.log(`Model: ${modelPath}`);
console.log("");
console.log("Run:");
console.log(
  `docker compose --env-file ${path.relative(
    root,
    path.join(runtimeRoot, "compose.env"),
  )} -f ${path.relative(root, composeFile)} up`,
);
