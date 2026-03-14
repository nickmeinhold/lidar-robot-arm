# LiDAR Robot Arm

iPhone LiDAR teleoperation system for the SO-100/SO-101 robot arm.

## Project Overview

Uses iPhone ARKit body tracking + Vision hand pose detection to control a HuggingFace LeRobot SO-100 robot arm in real-time. The iPhone replaces the traditional "leader arm" — your arm IS the leader.

## Architecture

```
iPhone App (Swift/SwiftUI)  →  WebSocket  →  Python Server  →  SO-100 Arm
  ARKit body tracking                         Joint mapping      Feetech STS3215
  Vision hand pose                            LeRobot API        servos via USB
```

## Project Structure

- `ios-app/` — Xcode project (Swift, iOS 18+, requires LiDAR-equipped iPhone)
  - `project.yml` — XcodeGen spec, run `xcodegen generate` to regenerate `.xcodeproj`
  - `ArmTracker/Sources/` — all Swift source files
- `server/` — Python control server (TODO)

## Build & Run

### iOS App
```bash
cd ios-app
xcodegen generate
open ArmTracker.xcodeproj
# Set development team in Signing & Capabilities, deploy to iPhone
```

### Python Server (TODO)
```bash
cd server
pip install -r requirements.txt
python server.py
```

## Key Technical Decisions

- **Swift 6 strict concurrency** — all actor isolation is explicit, uses `@MainActor`, `nonisolated`, and `@preconcurrency` for ARKit delegates
- **ARKit + Vision dual pipeline** — ARKit for 3D arm skeleton, Vision for hand grip (ARKit doesn't track fingers)
- **Follower arm only** — no leader arm needed, iPhone replaces it
- **STS3215 C001 (1/345 gear) servos** — high-torque variant for the follower arm

## Hardware

- iPhone 12 Pro+ (LiDAR required)
- SO-100 follower arm: 6x STS3215 7.4V servos + Waveshare driver board
- 3D printed parts from [SO-ARM100 repo](https://github.com/TheRobotStudio/SO-ARM100)
