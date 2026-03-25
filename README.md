# LiDAR Robot Arm

Your iPhone becomes the leader arm. No second robot arm needed.

This is a teleoperation system for the [HuggingFace LeRobot](https://github.com/huggingface/lerobot) SO-100/SO-101 robot arm. It uses iPhone LiDAR body tracking and Vision hand pose detection to stream your arm's joint angles over WebSocket to a Python server that (will eventually) drive the physical servos.

```
iPhone (ARKit + Vision)  ──WebSocket──▶  Python Server  ──USB──▶  SO-100 Arm
  body skeleton at 60Hz                    joint mapping           6× Feetech STS3215
  hand grip at 30Hz                        servo commands          servos via Waveshare board
  Bonjour auto-discovery                   Bonjour advertisement
```

## What works today

The iOS app tracks your right arm (shoulder, elbow, wrist) in 3D using ARKit's body tracking configuration with LiDAR scene depth. A parallel Vision pipeline detects hand open/close by measuring fingertip-to-wrist distances, normalized against hand scale so it works regardless of distance from the camera. Joint angles stream at ~30Hz over WebSocket with backpressure protection (stale frames are dropped — the latest position is always what matters for real-time control).

The Python server receives the angles, measures round-trip latency via pong echoes, and prints joint state to the console. Bonjour discovery means the phone finds the server automatically on your local network — no IP address configuration needed.

**What doesn't work yet:** The server has no actual servo control. `ConsoleArmController` is a stub that prints angles to stdout. The `ArmController` ABC is there, ready for a `FeetechArmController` that talks to the hardware via LeRobot's `FeetechMotorsBus`. Calibration, recording, and playback are all future work.

## Hardware

| Component | Notes |
|---|---|
| iPhone 12 Pro or later | LiDAR sensor required for `ARBodyTrackingConfiguration` |
| SO-100 follower arm | [3D printed parts](https://github.com/TheRobotStudio/SO-ARM100) |
| 6× Feetech STS3215 | 7.4V, 1/345 gear ratio (C001 high-torque variant) |
| Waveshare Bus Servo Adapter | USB-to-serial for STS3215 half-duplex bus |
| 5V 3A+ power supply | For the servo bus adapter |

## Build & Run

### iOS App

Requires Xcode 16+, iOS 18+, and a physical LiDAR-equipped iPhone (no simulator — ARKit body tracking needs real hardware).

```bash
cd ios-app
brew install xcodegen  # if you don't have it
xcodegen generate
open ArmTracker.xcodeproj
```

Set your development team in Signing & Capabilities, then build and deploy to your iPhone. The app will start AR body tracking immediately and begin searching for a server via Bonjour.

**No third-party dependencies.** The entire iOS app uses Apple frameworks only: ARKit, Vision, RealityKit, Network (for Bonjour), URLSession (for WebSocket), SwiftUI, and Combine.

### Python Server

```bash
cd server
python -m venv .venv && source .venv/bin/activate
pip install -r requirements.txt
python -m server
```

Options:
- `--port 8765` — WebSocket port (default: 8765)
- `--no-bonjour` — disable Bonjour/zeroconf advertisement

The server accepts one client at a time (second connections are rejected) and prints live joint angles to the terminal:

```
SY:  12.3° SP:  45.6° EP:  78.9° WP:  -5.2° WR:  10.1° G:  85%  | body:OK hand:OK
```

## Architecture

### iOS (Swift 6, strict concurrency)

All actor isolation is explicit. `@MainActor` classes publish state via Combine, `@preconcurrency` bridges ARKit delegates, and `nonisolated` static methods keep Vision processing off the main thread.

- **`BodyTrackingManager`** — owns the `ARSession`, extracts shoulder → elbow → wrist joint chain from the 91-joint ARKit skeleton, converts from model space to world space, and feeds frames to the hand detector
- **`JointAngleCalculator`** — pure geometry: computes 5 DOF angles from 3D positions and rotation matrices (shoulder yaw/pitch via vector math, elbow pitch via angle between upper arm and forearm vectors, wrist pitch/roll via Euler angle decomposition relative to the forearm)
- **`HandPoseDetector`** — runs `VNDetectHumanHandPoseRequest` on a detached task, computes grip by averaging normalized fingertip-to-wrist distances against the wrist-to-knuckle hand scale
- **`WebSocketClient`** — `URLSessionWebSocketTask` with exponential backoff reconnection and send-side backpressure (drops frames if a send is in-flight)
- **`BonjourDiscovery`** — `NWBrowser` for `_armtracker._tcp`, resolves endpoints via temporary `NWConnection`
- **`ArmState`** — the data model: 6 joint angles (radians) + tracking status, serializes to JSON for the wire

### Python Server

- **`server.py`** — async WebSocket server (single-client enforced), routes `arm_state` messages to the controller, echoes `pong` for latency measurement
- **`protocol.py`** — message parsing and data classes (`ArmAngles`, `TrackingStatus`, `ArmStateMessage`)
- **`arm_controller.py`** — `ArmController` ABC + `ConsoleArmController` stub
- **`discovery.py`** — Bonjour advertisement via `zeroconf` library

### Wire Protocol

The iPhone sends JSON at ~30Hz:

```json
{
  "type": "arm_state",
  "timestamp": 1711234567.890,
  "angles": {
    "shoulder_yaw": 0.215,
    "shoulder_pitch": 0.785,
    "elbow_pitch": 1.047,
    "wrist_pitch": -0.091,
    "wrist_roll": 0.176,
    "gripper": 0.85
  },
  "tracking": {
    "body": true,
    "hand": true
  }
}
```

Angles are in radians. Gripper is 0.0 (closed) to 1.0 (fully open). The server responds with `{"type": "pong", "timestamp": ...}` echoing the original timestamp for round-trip latency calculation.

## Project Structure

```
ios-app/
  project.yml                          XcodeGen project spec
  ArmTracker/Sources/
    ArmTrackerApp.swift                App entry point
    ContentView.swift                  Main SwiftUI view
    ARViewContainer.swift              UIViewRepresentable for ARView
    ArmState.swift                     6-DOF joint state model
    JointAngleCalculator.swift         3D → joint angle math
    BodyTrackingManager.swift          ARKit body tracking session
    HandPoseDetector.swift             Vision hand grip detection
    WebSocketClient.swift              WebSocket with backpressure
    BonjourDiscovery.swift             mDNS server discovery
    ConnectionStatusView.swift         Connection status UI
server/
  server.py                            WebSocket server
  arm_controller.py                    Controller ABC + console stub
  protocol.py                          Message parsing
  discovery.py                         Bonjour advertisement
  requirements.txt                     websockets, zeroconf
```

~1,487 lines total across Swift and Python.

## Roadmap

- [x] **Phase 1:** iOS body & hand tracking
- [x] **Phase 2:** WebSocket streaming + Bonjour discovery
- [ ] **Phase 3:** Servo control — implement `FeetechArmController` using LeRobot's `FeetechMotorsBus`, map human angle ranges to servo position ranges, add joint smoothing and safety clamping
- [ ] **Phase 4:** Calibration — record your arm's range of motion to build a proper human-to-robot joint mapping, per-joint gain/offset tuning, dead zones, emergency stop
- [ ] **Phase 5:** Recording & playback, multi-camera support, latency compensation

The biggest open question is the human-to-robot angle mapping. The SO-100's joint ranges don't match a human arm, and the ARKit skeleton's coordinate frame assumptions need careful calibration against the physical servo positions. This will probably take more iteration than the networking did.

## Dependencies

### iOS
None. Apple frameworks only.

### Python
- `websockets` >= 15.0
- `zeroconf` >= 0.146.0

Future: `lerobot` (for `FeetechMotorsBus` servo control), `numpy` (for angle smoothing/filtering).

## License

Experimental / personal project. No license yet.
