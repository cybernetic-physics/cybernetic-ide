#!/usr/bin/env node
import { existsSync } from "node:fs";
import fs from "node:fs/promises";
import path from "node:path";
import { spawnSync } from "node:child_process";
import { fileURLToPath } from "node:url";

const root = path.resolve(path.dirname(fileURLToPath(import.meta.url)), "..");
const runtimeRoot = path.join(root, ".runtime/unitree-g1-sdk2");
const mujocoVersion = process.env.CYBER_MUJOCO_VERSION || "3.3.6";
const mujocoArchiveName = process.env.CYBER_MUJOCO_ARCHIVE_NAME || `mujoco-${mujocoVersion}-linux-aarch64.tar.gz`;
const mujocoRemote =
  process.env.CYBER_MUJOCO_REMOTE ||
  `https://github.com/google-deepmind/mujoco/releases/download/${mujocoVersion}/${mujocoArchiveName}`;

const repos = [
  {
    name: "unitree_sdk2_python",
    envRemote: "UNITREE_SDK2_PYTHON_REMOTE",
    envRevision: "UNITREE_SDK2_PYTHON_REVISION",
    remote: "https://github.com/unitreerobotics/unitree_sdk2_python.git",
    revision: "37116c521f1588482e238d8450e471ba78ab9863",
  },
  {
    name: "unitree_sdk2",
    envRemote: "UNITREE_SDK2_REMOTE",
    envRevision: "UNITREE_SDK2_REVISION",
    remote: "https://github.com/unitreerobotics/unitree_sdk2.git",
    revision: "7740f8b67e386ab09c3b333187fd5f8582a75ddc",
  },
  {
    name: "unitree_mujoco",
    envRemote: "UNITREE_MUJOCO_REMOTE",
    envRevision: "UNITREE_MUJOCO_REVISION",
    remote: "https://github.com/unitreerobotics/unitree_mujoco.git",
    revision: "ae6a8403e272733e9996ef59990880330496177f",
  },
];

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

function quote(value) {
  return String(value).replaceAll("\\", "\\\\").replaceAll("\n", "\\n");
}

async function ensureRepo(repo) {
  const target = path.join(runtimeRoot, repo.name);
  const remote = process.env[repo.envRemote] || repo.remote;
  const revision = process.env[repo.envRevision] || repo.revision;
  if (!existsSync(path.join(target, ".git"))) {
    await fs.rm(target, { recursive: true, force: true });
    run("git", ["clone", "--filter=blob:none", remote, target]);
  } else {
    run("git", ["reset", "--hard"], { cwd: target });
  }
  run("git", ["fetch", "--depth", "1", "origin", revision], { cwd: target });
  run("git", ["checkout", "--detach", "FETCH_HEAD"], { cwd: target });
  return { ...repo, target, remote, revision };
}

async function ensureMujocoRelease() {
  const target = path.join(runtimeRoot, `mujoco-${mujocoVersion}`);
  const header = path.join(target, "include/mujoco/mujoco.h");
  if (existsSync(header)) {
    return { target, remote: mujocoRemote, version: mujocoVersion, cached: true };
  }

  await fs.rm(target, { recursive: true, force: true });
  const archive = path.join(runtimeRoot, mujocoArchiveName);
  run("curl", ["-L", "--fail", "-o", archive, mujocoRemote]);
  run("tar", ["-xzf", archive, "-C", runtimeRoot]);
  if (!existsSync(header)) {
    throw new Error(`MuJoCo release did not unpack with expected header: ${header}`);
  }
  return { target, remote: mujocoRemote, version: mujocoVersion, cached: false };
}

await fs.mkdir(runtimeRoot, { recursive: true });
const prepared = [];
for (const repo of repos) {
  prepared.push(await ensureRepo(repo));
}
const mujocoRelease = await ensureMujocoRelease();

const values = Object.fromEntries(prepared.map((repo) => [repo.name, repo]));
const mode = process.env.CYBER_UNITREE_MODE || "sim";
const domain = process.env.CYBER_UNITREE_DDS_DOMAIN || (mode === "sim" ? "1" : "0");
const networkInterface = process.env.CYBER_UNITREE_NETWORK_INTERFACE || (mode === "sim" ? "lo" : "");

await fs.writeFile(
  path.join(runtimeRoot, "compose.env"),
  [
    `CYBER_UNITREE_MODE=${quote(mode)}`,
    `CYBER_UNITREE_TRANSPORT=${quote(process.env.CYBER_UNITREE_TRANSPORT || "dds")}`,
    `CYBER_UNITREE_DDS_DOMAIN=${quote(domain)}`,
    `CYBER_UNITREE_NETWORK_INTERFACE=${quote(networkInterface)}`,
    `UNITREE_SDK2_PYTHON_ROOT=${quote("/opt/unitree_sdk2_python")}`,
    `UNITREE_SDK2_ROOT=${quote("/opt/unitree_sdk2")}`,
    `UNITREE_MUJOCO_ROOT=${quote("/opt/unitree_mujoco")}`,
    `UNITREE_SDK2_PYTHON_HOST_ROOT=${quote(values.unitree_sdk2_python.target)}`,
    `UNITREE_SDK2_HOST_ROOT=${quote(values.unitree_sdk2.target)}`,
    `UNITREE_MUJOCO_HOST_ROOT=${quote(values.unitree_mujoco.target)}`,
    `MUJOCO_HOST_ROOT=${quote(mujocoRelease.target)}`,
    `MUJOCO_ROOT=${quote("/opt/mujoco")}`,
    `UNITREE_SDK2_PYTHON_REVISION=${quote(values.unitree_sdk2_python.revision)}`,
    `UNITREE_SDK2_REVISION=${quote(values.unitree_sdk2.revision)}`,
    `UNITREE_MUJOCO_REVISION=${quote(values.unitree_mujoco.revision)}`,
    `CYBER_MUJOCO_VERSION=${quote(mujocoRelease.version)}`,
    "",
  ].join("\n"),
);

console.log(`Prepared Unitree G1 SDK2 sidecar runtime at ${runtimeRoot}`);
for (const repo of prepared) {
  console.log(`${repo.name}: ${repo.remote} @ ${repo.revision}`);
}
console.log(`mujoco: ${mujocoRelease.remote} @ ${mujocoRelease.version}${mujocoRelease.cached ? " (cached)" : ""}`);
console.log("");
console.log("Run:");
console.log(
  `docker compose --env-file ${path.relative(root, path.join(runtimeRoot, "compose.env"))} -f overlays/unitree-g1-sdk2-sidecar/compose.yaml run --rm unitree-g1-sdk2-sidecar`,
);
