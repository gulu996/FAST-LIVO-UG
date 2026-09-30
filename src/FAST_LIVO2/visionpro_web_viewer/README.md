# VPPC live point-cloud Viewer

A static, offline-capable browser client for the existing FAST-LIVO2 VisionPro Gateway. It displays the current registered XYZI scan and can optionally build a bounded, voxel-deduplicated history for visualization. It does not use TF or enter WebXR. The Gateway accepts one client at a time, so close the Python receiver before connecting the Viewer.

## Run on the SLAM PC

Build and source the existing catkin workspace as usual. The current `config/mid360.yaml` already enables `/cloud_registered_frame`; `mapping_mid360.launch` loads it before starting the mapper. Start the mapper, Gateway, and HTTP server in separate terminals:

```bash
source /home/gulu/catkin_ws/devel/setup.bash
roslaunch fast_livo mapping_mid360.launch
```

```bash
source /home/gulu/catkin_ws/devel/setup.bash
roslaunch fast_livo visionpro_pointcloud_gateway.launch
```

```bash
cd /home/gulu/catkin_ws/src/FAST_LIVO2/visionpro_web_viewer
./start_server.sh
```

For bag playback, use a fourth terminal after the mapper and Gateway are ready:

```bash
source /home/gulu/catkin_ws/devel/setup.bash
rosbag play /path/to/your.bag
```

Use `hostname -I` on the SLAM PC to find its current LAN IP. On a desktop browser open `http://localhost:8080/` or `http://<SLAM_PC_IP>:8080/`. On Vision Pro, join the same reachable LAN and open `http://<SLAM_PC_IP>:8080/` in Safari. The URL field defaults to `ws://<page-host>:8765/`; press **Connect**. For example, if the PC is `192.168.1.100`, the page is `http://192.168.1.100:8080/` and the WebSocket is `ws://192.168.1.100:8765/`. Both TCP ports must be reachable. `PORT=8081 ./start_server.sh` changes only the HTTP port.

The page and all JavaScript assets are served locally. Neither the Viewer nor its HTTP server needs Internet, npm, a CDN, HTTPS, WSS, ROS on the browser device, or a Vision Pro native app. HTTP plus `ws://` is intended for a trusted LAN.

## Protocol and display

`src/protocol.js` follows the actual `src/visionpro_pointcloud_gateway.cpp` and `tools/visionpro_pointcloud_receiver.py`. Each WebSocket **binary message** is one VPPC v1 frame. WebSocket may fragment a message internally; browsers reassemble it before the `message` event. The fixed 36-byte little-endian header is: `VPPC` magic (4), version (u16=1), type (u16=1), sequence (u64), ROS timestamp nanoseconds (u64), point count (u32), stride (u16=16), UTF-8 frame ID length (u16), payload length (u32). The frame ID follows, then packed little-endian float32 `x,y,z,intensity` points. The decoder checks all lengths, point count, stride, finite values, and UTF-8 before rendering.

The source topic uses the SLAM `camera_init` frame. This MID360 profile gravity-aligns Z up. The Viewer leaves XYZ values intact, then rotates one scene root by -90 degrees around X: ROS `(x,y,z)` appears in Three.js as `(x,z,-y)`. Axes and the ROS XY ground grid share that root. Orbit drag rotates, wheel/pinch zooms, and right drag pans. **Reset View** frames the current scan.

Intensity comes from Livox reflectivity in this profile, but the page does not assume a fixed range. It computes each frame's min, max, P5, and P95 and maps P5 to black and P95 to white, clamping extremes. A near-constant range uses a visible gray fallback. **White** ignores intensity. Point size, axes, grid, pause, connect, and disconnect are adjustable.

The network handler stores only the latest unparsed `ArrayBuffer`. One animation tick decodes and replaces the current scan. Current and history are separate `THREE.Points` layers; their geometry buffers are reused until capacity must grow, and replaced buffers are disposed. The status panel reports receive/render rates, bandwidth, payload size, local viewer drops, malformed frames, and GPU geometry count. Pause stops rendering while the latest received frame remains available for resume. The **Show panels** switch at the bottom right hides or restores both information panels on that browser only; connect before hiding them. This changes only the page display. The Gateway still accepts one WebSocket client at a time, so two browser pages cannot both receive live frames concurrently.

## Current frame, viewer history, and ROI

By default **Show Current Frame** is on, **Show History** and **Accumulate** are off. This is the original current-frame-only display. Turn on **Accumulate** to insert decoded finite XYZI points into a viewer-side voxel map; turn it off to keep the map fixed while the current frame continues. **Show Current Frame** and **Show History** control the two visible layers independently. When both are visible, history uses the same per-cloud P5/P95 intensity grayscale rule as the current frame, while current points are drawn in brighter amber to distinguish the live scan. The original intensity values and network data are unchanged.

The voxel key is the string `floor(x/s),floor(y/s),floor(z/s)` in ROS coordinates, with one **first valid point** per voxel. The default voxel size is **0.010 m**; values such as 0.005, 0.020, and 0.050 m are accepted. Changing voxel size clears history after the visible warning. History is capped at **500,000** points; new voxels stop being added at the cap and the status panel says `LIMIT REACHED`. **Clear History** releases the JS voxel map and history GPU buffers without touching the current frame or WebSocket. History memory estimate is allocated position/intensity typed-array bytes plus an approximate 64 bytes per Map entry and 2 bytes per string-key character; browser engine and GPU overhead are excluded. GPU Buffer Estimate is the allocated position/color attribute bytes for both point layers, excluding driver overhead.

History Update Every N Frames defaults to 1. To keep the current frame responsive, at most 4096 history points and about 4 ms of insertion work are processed after each current-frame draw. If a newer decoded frame arrives before that work finishes, it replaces unfinished history work; **History Skipped Frames** counts these replacements. History GPU data is refreshed at most 5 times per second and only after new voxels are inserted. This is a visualization history, **not the official FAST-LIVO2 global map** or a ROS map topic. It exists only in the current browser page and is lost on refresh or close.

**Enable ROI** clips both visible point layers to inclusive ROS/SLAM XYZ min/max bounds. Each X, Y, and Z row has two independently draggable range thumbs. The full slider domain is derived from the history bounds, or the current frame when history is empty, with a 5% or 0.05 m margin. Dragging only updates GPU shader uniforms; it does not rebuild the point geometry. The cyan wireframe ROI box lives under the same ROS-to-Three.js root transform as the points. ROI is a display filter: disabling it restores all points without deleting history. **Reset ROI** restores full available bounds; **Fit ROI to Map** also frames the camera on those bounds. **Selected Points** counts points inside ROI across currently visible layers, refreshed at most about 10 Hz.

## Offline dependency

`vendor/three.module.js` and `vendor/OrbitControls.js` are pinned to the official three.js **r160** release; `vendor/LICENSE` is the upstream MIT license. The import map points to the local copy. No `node_modules` or build step is needed.

## Validation boundary

Run `python3 -m http.server` via `start_server.sh`, then test the browser against a running Gateway. Without ROS, open `http://localhost:8080/tests/protocol_test.html`, `http://localhost:8080/tests/renderer_test.html`, and `http://localhost:8080/tests/mapping_test.html`; all must show PASS. Desktop Chrome/Firefox and Vision Pro Safari results must be reported separately; a desktop result alone is not a Vision Pro result. This phase is a normal Safari window, not spatial AR alignment.
