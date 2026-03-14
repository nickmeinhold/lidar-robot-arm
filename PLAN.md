# LiDAR Robot Arm — Implementation Plan

## Phase 1: iOS App — Body & Hand Tracking [DONE]
- [x] ARKit `ARBodyTrackingConfiguration` with LiDAR scene depth
- [x] Extract shoulder → elbow → wrist joint chain
- [x] `JointAngleCalculator` — 3D positions to servo-ready angles (yaw, pitch, elbow, wrist pitch/roll)
- [x] Vision `VNDetectHumanHandPoseRequest` for grip open/close detection
- [x] SwiftUI debug overlay showing live joint angles + tracking status
- [x] Left/right arm toggle
- [x] Xcode project via XcodeGen, builds clean with Swift 6 strict concurrency

## Phase 2: Network Streaming
- [ ] `WebSocketClient.swift` — streams `ArmState` as JSON over WebSocket (~30Hz)
- [ ] Auto-discovery via Bonjour/mDNS (find Python server on local network)
- [ ] Connection status indicator in the UI
- [ ] Configurable server address (fallback if Bonjour unavailable)
- [ ] Latency display in debug overlay

## Phase 3: Python Control Server
- [ ] WebSocket server (receives joint angle JSON from iPhone)
- [ ] `arm_controller.py` — maps human arm angles to SO-100 servo positions
- [ ] Joint angle smoothing / filtering (exponential moving average)
- [ ] Servo range clamping (prevent damage from out-of-range commands)
- [ ] LeRobot `FeetechMotorsBus` integration for servo control
- [ ] Gripper servo control from grip amount
- [ ] `requirements.txt` — lerobot, websockets, numpy

## Phase 4: Calibration & Tuning
- [ ] Calibration mode — record arm range of motion to map human→robot ranges
- [ ] Per-joint gain/offset adjustment
- [ ] Dead zone configuration (ignore small movements)
- [ ] Safety limits — max servo speed, emergency stop
- [ ] Latency compensation

## Phase 5: Polish & Extras
- [ ] Record & playback arm movements
- [ ] Multiple camera angle support (front camera for hand, rear for body)
- [ ] Object detection for autonomous grasping assist
- [ ] Direct USB connection option (bypassing network for lower latency)

## Hardware Checklist
- [ ] 3D print follower arm (SO-ARM100 STL from GitHub)
- [ ] Order 6x STS3215 7.4V C001 servos
- [ ] Order Waveshare Bus Servo Adapter board
- [ ] Order 5V 3A+ power supply
- [ ] Assemble arm per SO-ARM100 assembly guide
- [ ] Configure servo IDs (1-6) using LeRobot calibration script
