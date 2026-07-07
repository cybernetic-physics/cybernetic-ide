#!/usr/bin/env node

const args = new Map();
for (let index = 2; index < process.argv.length; index += 1) {
  const arg = process.argv[index];
  if (!arg.startsWith("--")) {
    continue;
  }

  const key = arg.slice(2);
  const next = process.argv[index + 1];
  if (!next || next.startsWith("--")) {
    args.set(key, "true");
  } else {
    args.set(key, next);
    index += 1;
  }
}

const physicsUrl = args.get("url") ?? "ws://127.0.0.1:8788";
const gameControlUrl = args.get("game-control-url") ?? "http://127.0.0.1:38383";
const topic = args.get("topic") ?? "simulation_state";
const command = args.get("command");
const timeoutMs = Number(args.get("timeout-ms") ?? 5000);
const verbose = args.has("verbose");

const expectedTypes = new Map([
  ["camera_frame_0", 3],
  ["simulation_state", 6],
  ["visual_scene", 10],
  ["visual_frame", 11],
]);

if (args.has("help")) {
  console.log(`Usage:
  node script/probe-unitree-g1-mujoco-protocol.mjs
  node script/probe-unitree-g1-mujoco-protocol.mjs --topic visual_frame
  node script/probe-unitree-g1-mujoco-protocol.mjs --command pause
  node script/probe-unitree-g1-mujoco-protocol.mjs --verbose
`);
  process.exit(0);
}

if (typeof WebSocket === "undefined") {
  throw new Error("This probe needs a Node runtime with global WebSocket support.");
}

function readUInt64(view, offset) {
  const value = view.getBigUint64(offset);
  return value <= BigInt(Number.MAX_SAFE_INTEGER) ? Number(value) : value.toString();
}

function readInt64(view, offset) {
  const value = view.getBigInt64(offset);
  return value >= BigInt(Number.MIN_SAFE_INTEGER) && value <= BigInt(Number.MAX_SAFE_INTEGER)
    ? Number(value)
    : value.toString();
}

function decodeMsgpack(input) {
  const bytes = input instanceof Uint8Array ? input : new Uint8Array(input);
  const view = new DataView(bytes.buffer, bytes.byteOffset, bytes.byteLength);
  const textDecoder = new TextDecoder();
  let offset = 0;

  function take(length) {
    if (offset + length > bytes.length) {
      throw new Error(`MessagePack payload ended early at ${offset}, need ${length} bytes`);
    }
    const value = bytes.subarray(offset, offset + length);
    offset += length;
    return value;
  }

  function readMap(length) {
    const value = {};
    for (let index = 0; index < length; index += 1) {
      value[String(readValue())] = readValue();
    }
    return value;
  }

  function readArray(length) {
    const value = [];
    for (let index = 0; index < length; index += 1) {
      value.push(readValue());
    }
    return value;
  }

  function readString(length) {
    return textDecoder.decode(take(length));
  }

  function readValue() {
    const prefix = view.getUint8(offset);
    offset += 1;

    if (prefix <= 0x7f) return prefix;
    if (prefix >= 0x80 && prefix <= 0x8f) return readMap(prefix & 0x0f);
    if (prefix >= 0x90 && prefix <= 0x9f) return readArray(prefix & 0x0f);
    if (prefix >= 0xa0 && prefix <= 0xbf) return readString(prefix & 0x1f);
    if (prefix >= 0xe0) return prefix - 0x100;

    switch (prefix) {
      case 0xc0:
        return null;
      case 0xc2:
        return false;
      case 0xc3:
        return true;
      case 0xc4:
        return { binaryLength: take(view.getUint8(offset++)).length };
      case 0xc5: {
        const length = view.getUint16(offset);
        offset += 2;
        return { binaryLength: take(length).length };
      }
      case 0xc6: {
        const length = view.getUint32(offset);
        offset += 4;
        return { binaryLength: take(length).length };
      }
      case 0xca: {
        const value = view.getFloat32(offset);
        offset += 4;
        return value;
      }
      case 0xcb: {
        const value = view.getFloat64(offset);
        offset += 8;
        return value;
      }
      case 0xcc:
        return view.getUint8(offset++);
      case 0xcd: {
        const value = view.getUint16(offset);
        offset += 2;
        return value;
      }
      case 0xce: {
        const value = view.getUint32(offset);
        offset += 4;
        return value;
      }
      case 0xcf: {
        const value = readUInt64(view, offset);
        offset += 8;
        return value;
      }
      case 0xd0:
        return view.getInt8(offset++);
      case 0xd1: {
        const value = view.getInt16(offset);
        offset += 2;
        return value;
      }
      case 0xd2: {
        const value = view.getInt32(offset);
        offset += 4;
        return value;
      }
      case 0xd3: {
        const value = readInt64(view, offset);
        offset += 8;
        return value;
      }
      case 0xd9:
        return readString(view.getUint8(offset++));
      case 0xda: {
        const length = view.getUint16(offset);
        offset += 2;
        return readString(length);
      }
      case 0xdb: {
        const length = view.getUint32(offset);
        offset += 4;
        return readString(length);
      }
      case 0xdc: {
        const length = view.getUint16(offset);
        offset += 2;
        return readArray(length);
      }
      case 0xdd: {
        const length = view.getUint32(offset);
        offset += 4;
        return readArray(length);
      }
      case 0xde: {
        const length = view.getUint16(offset);
        offset += 2;
        return readMap(length);
      }
      case 0xdf: {
        const length = view.getUint32(offset);
        offset += 4;
        return readMap(length);
      }
      default:
        throw new Error(`Unsupported MessagePack prefix 0x${prefix.toString(16)} at ${offset - 1}`);
    }
  }

  const value = readValue();
  if (offset !== bytes.length) {
    return { value, trailingBytes: bytes.length - offset };
  }
  return value;
}

async function toBuffer(data) {
  if (typeof data === "string") return { text: data };
  if (data instanceof ArrayBuffer) return { buffer: Buffer.from(data) };
  if (ArrayBuffer.isView(data)) {
    return { buffer: Buffer.from(data.buffer, data.byteOffset, data.byteLength) };
  }
  if (data && typeof data.arrayBuffer === "function") {
    return { buffer: Buffer.from(await data.arrayBuffer()) };
  }
  throw new Error(`Unsupported websocket message payload: ${typeof data}`);
}

function waitForOpen(ws) {
  return new Promise((resolve, reject) => {
    const timeout = setTimeout(() => reject(new Error(`Timed out opening ${physicsUrl}`)), timeoutMs);
    ws.addEventListener("open", () => {
      clearTimeout(timeout);
      resolve();
    }, { once: true });
    ws.addEventListener("error", () => {
      clearTimeout(timeout);
      reject(new Error(`Failed to open ${physicsUrl}`));
    }, { once: true });
  });
}

function waitForBinaryFrame(ws) {
  return new Promise((resolve, reject) => {
    const timeout = setTimeout(() => reject(new Error(`Timed out waiting for ${topic}`)), timeoutMs);
    async function onMessage(event) {
      try {
        const payload = await toBuffer(event.data);
        if (payload.text !== undefined) {
          if (verbose) console.error(`[physics:text] ${payload.text}`);
          return;
        }
        clearTimeout(timeout);
        ws.removeEventListener("message", onMessage);
        resolve(payload.buffer);
      } catch (error) {
        clearTimeout(timeout);
        ws.removeEventListener("message", onMessage);
        reject(error);
      }
    }
    ws.addEventListener("message", onMessage);
  });
}

function parseEnvelope(frame) {
  if (frame.length < 8) {
    throw new Error(`Frame is too short: ${frame.length} bytes`);
  }

  const messageType = frame.readUInt32BE(0);
  const payloadLength = frame.readUInt32BE(4);
  const payload = frame.subarray(8, 8 + payloadLength);
  if (payload.length !== payloadLength) {
    throw new Error(`Frame payload is truncated: expected ${payloadLength}, got ${payload.length}`);
  }

  const expectedType = expectedTypes.get(topic);
  if (expectedType !== undefined && messageType !== expectedType) {
    throw new Error(`Expected ${topic} type ${expectedType}, got ${messageType}`);
  }

  return { messageType, payloadLength, payload };
}

function summarizeDecodedPayload(decoded) {
  if (verbose || !decoded || typeof decoded !== "object") return decoded;

  if (topic === "simulation_state") {
    return {
      paused: decoded.paused,
      actual_speed_factor: decoded.actual_speed_factor,
      model_path: decoded.model_path,
      model_revision: decoded.model_revision,
      robot_statuses: decoded.robot_statuses,
      robot_modes: decoded.robot_modes,
      all_robot_names: decoded.all_robot_names,
      mujoco: decoded.mujoco,
      render: decoded.render,
    };
  }

  if (topic === "visual_frame") {
    return {
      revision: decoded.revision,
      time: decoded.time,
      frame_id: decoded.frame_id,
      geoms: Array.isArray(decoded.geoms) ? decoded.geoms.length : null,
      bodies: Array.isArray(decoded.bodies) ? decoded.bodies.length : null,
      cameras: Array.isArray(decoded.cameras) ? decoded.cameras.length : null,
      robotLabels: decoded.robotLabels,
    };
  }

  if (topic === "visual_scene") {
    return {
      revision: decoded.revision,
      model_path: decoded.model_path,
      robot: decoded.robot,
      geoms: Array.isArray(decoded.geoms) ? decoded.geoms.length : null,
      bodies: Array.isArray(decoded.bodies) ? decoded.bodies.length : null,
      cameras: Array.isArray(decoded.cameras) ? decoded.cameras.length : null,
    };
  }

  return decoded;
}

async function probePhysics() {
  const ws = new WebSocket(physicsUrl);
  ws.binaryType = "arraybuffer";
  await waitForOpen(ws);

  if (command) {
    ws.send(JSON.stringify({ type: "command", command, params: {} }));
  }

  ws.send(`subscribe:${topic}`);
  const frame = await waitForBinaryFrame(ws);
  const envelope = parseEnvelope(frame);
  let decoded = null;
  let decodeError = null;
  try {
    decoded = decodeMsgpack(envelope.payload);
  } catch (error) {
    decodeError = error.message;
  }
  ws.send(`unsubscribe:${topic}`);
  ws.close();

  if (decodeError) {
    throw new Error(`Failed to decode ${topic}: ${decodeError}`);
  }

  return {
    physicsUrl,
    topic,
    command: command ?? null,
    messageType: envelope.messageType,
    payloadLength: envelope.payloadLength,
    decoded: summarizeDecodedPayload(decoded),
  };
}

async function probeGameControl() {
  const health = await fetch(`${gameControlUrl}/health`).then((response) => response.json());
  const status = await fetch(`${gameControlUrl}/status`).then((response) => response.json());
  return {
    gameControlUrl,
    health: {
      status: health.status,
      ready: health.ready,
      checks: health.checks,
    },
    status: {
      ready: status.ready,
      model_path: status.simulation?.model_path,
      robot_statuses: status.simulation?.robot_statuses,
      render: status.simulation?.render,
    },
  };
}

const result = {
  physics: await probePhysics(),
  gameControl: args.has("skip-game-control") ? null : await probeGameControl(),
};

console.log(JSON.stringify(result, null, 2));
