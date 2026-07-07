---
name: cybernetic-sim-protocol
description: Use when debugging or extending Cybernetic IDE's Booster-style simulator protocol, including GameControl HTTP, the physics WebSocket, MessagePack frame topics, camera control, and viewer snapshots.
---

# Cybernetic Simulator Protocol

Use this skill when a task mentions simulator protocol, WebSocket topics, GameControl HTTP, visual frames, camera control, or protocol probes.

## Endpoints

Default local endpoints:

```text
GameControl HTTP: http://127.0.0.1:38383
Physics WebSocket: ws://127.0.0.1:8788
```

Important HTTP routes:

- `GET /status`
- `GET /visual_scene`
- `GET /visual_frame`
- `GET /camera`
- `GET /camera_frame_0.jpg`
- `POST /command`
- `POST /camera`

Important WebSocket topics:

- `simulation_state`
- `visual_frame`
- `visual_scene`
- `camera_frame_0`

## Preferred MCP Tools

- `sim_status` for a full health/status snapshot.
- `protocol_probe_http` for HTTP endpoint checks.
- `protocol_probe_ws` for a one-frame WebSocket subscription.
- `viewer_camera_control` for camera state/orbit/pan/zoom/reset.
- `viewer_snapshot` for a model-visible image.

## Latency Caveat

The current compatible server renders camera frames in Docker using software MuJoCo rendering and cached JPEGs. Control acknowledgments can be fast while visible image feedback trails behind render cadence. For interaction bugs, separate:

- command acknowledgment latency,
- MuJoCo/render-loop latency,
- frame transport/decoding latency,
- IDE viewer rerender latency.

## Do Not

- Do not assume `camera_frame_0` WebSocket is faster than JPEG polling; in the current server it renders PNG synchronously.
- Do not send arbitrary raw protocol commands to a real robot mode without an explicit safety gate.
- Do not claim real Booster parity unless the behavior was probed against the documented reversed protocol.
