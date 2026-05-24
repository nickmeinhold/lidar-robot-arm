import ARKit
import Combine
import RealityKit

/// Manages ARKit body tracking session and extracts arm joint data.
///
/// Uses `ARBodyTrackingConfiguration` which requires:
/// - iPhone 12 Pro or later (A14+ with LiDAR)
/// - Rear-facing camera
///
/// ARKit provides a full 3D skeleton with ~91 joints. We extract the
/// right arm chain: shoulder → elbow → wrist, and compute the joint
/// angles needed to drive the robot arm.
@MainActor
final class BodyTrackingManager: NSObject, ObservableObject {

    /// The AR session running body tracking.
    let arSession = ARSession()

    /// Published arm state, updated every frame (~60Hz).
    @Published var armState = ArmState()

    /// 3D mesh overlay rendered on the tracked arm in the RealityKit scene.
    let armMeshOverlay = ArmMeshOverlay()

    /// Reference to the ARView, set by `ARViewContainer` after creation.
    weak var arView: ARView?

    /// Which arm to track.
    var trackRightArm = true

    /// When true, the iPhone runs ARKit + Vision tracking but skips all on-device
    /// rendering: no AR camera view, no skeleton overlay, no 3D mesh generation.
    /// The phone behaves as a pure sensor streaming to remote viewers, dropping
    /// thermal load substantially (camera composite + RealityKit scene render
    /// + per-frame tube-mesh allocation are the biggest GPU/CPU costs).
    @Published var streamOnly: Bool = false

    /// 2D arm joint positions in normalized image coordinates (Vision convention:
    /// origin bottom-left, y up). Sourced from `ARSkeleton2D` via `ARFrame.detectedBody`,
    /// which gives us direct 2D landmarks without needing to project from 3D.
    @Published var armLandmarks2D: [CGPoint?] = [nil, nil, nil]

    /// The hand pose detector runs on the same camera frames.
    let handPoseDetector = HandPoseDetector()

    /// H.264 encoder for the AR camera feed. Lazily initialized on first frame
    /// because we need the actual buffer dimensions to configure the encoder.
    private var videoEncoder: VideoEncoder?

    /// WebSocket client for streaming arm state to the control server.
    @Published var webSocketClient = WebSocketClient()

    /// Bonjour discovery for finding control servers on the local network.
    @Published var bonjourDiscovery = BonjourDiscovery()

    private var cancellables = Set<AnyCancellable>()

    override init() {
        super.init()
        arSession.delegate = self

        // Forward hand pose grip updates into our arm state.
        handPoseDetector.$gripAmount
            .receive(on: RunLoop.main)
            .sink { [weak self] grip in
                self?.armState.gripperOpenAmount = grip
            }
            .store(in: &cancellables)

        handPoseDetector.$isHandDetected
            .receive(on: RunLoop.main)
            .sink { [weak self] detected in
                self?.armState.isHandTracked = detected
            }
            .store(in: &cancellables)

        // Throttle arm state from ~60Hz to ~30Hz and send over WebSocket.
        // 30Hz is plenty for servo control and halves network bandwidth.
        // Each send snapshots the current arm + hand skeleton so viewers
        // can reconstruct exactly what the phone is rendering.
        $armState
            .throttle(for: .milliseconds(33), scheduler: RunLoop.main, latest: true)
            .sink { [weak self] state in
                guard let self else { return }
                let frame = ArmTrackingFrame(
                    state: state,
                    armLandmarks2D: self.armLandmarks2D,
                    handSkeleton: self.handPoseDetector.handSkeleton
                )
                if let json = frame.toJSON() {
                    self.webSocketClient.send(rawJSON: json)
                }
            }
            .store(in: &cancellables)

        // Auto-connect when Bonjour discovers a server.
        // removeDuplicates() prevents reconnect loops when Bonjour
        // re-resolves the same endpoint (which kills the active connection).
        bonjourDiscovery.$selectedServerURL
            .compactMap { $0 }
            .removeDuplicates()
            .sink { [weak self] url in
                self?.webSocketClient.connect(to: url)
            }
            .store(in: &cancellables)
    }

    /// Start the AR body tracking session.
    func startTracking() {
        guard ARBodyTrackingConfiguration.isSupported else {
            print("Body tracking is not supported on this device.")
            return
        }

        let config = ARBodyTrackingConfiguration()
        config.automaticSkeletonScaleEstimationEnabled = true

        // Enable scene depth if LiDAR is available for better tracking.
        if ARBodyTrackingConfiguration.supportsFrameSemantics(.sceneDepth) {
            config.frameSemantics.insert(.sceneDepth)
        }

        arSession.run(config)
        bonjourDiscovery.startBrowsing()
        print("AR body tracking started.")
    }

    /// Stop the AR session and network services.
    func stopTracking() {
        arSession.pause()
        webSocketClient.disconnect()
        bonjourDiscovery.stopBrowsing()
    }

    /// Process body anchor updates — extracts arm joint angles.
    private func processBodyAnchor(_ bodyAnchor: ARBodyAnchor) {
        let skeleton = bodyAnchor.skeleton
        let bodyTransform = bodyAnchor.transform

        // Extract the arm joint chain based on which arm we're tracking.
        // ARKit skeleton joint names follow a hierarchy:
        //   root → spine → chest → shoulder → forearm → hand
        let shoulderJoint = trackRightArm
            ? ARSkeleton.JointName(rawValue: "right_shoulder_1_joint")
            : ARSkeleton.JointName(rawValue: "left_shoulder_1_joint")
        let elbowJoint = trackRightArm
            ? ARSkeleton.JointName(rawValue: "right_forearm_joint")
            : ARSkeleton.JointName(rawValue: "left_forearm_joint")
        let wristJoint = trackRightArm
            ? ARSkeleton.JointName(rawValue: "right_hand_joint")
            : ARSkeleton.JointName(rawValue: "left_hand_joint")

        // Get the model transforms (relative to root/hip).
        guard
            let shoulderTransform = skeleton.modelTransform(for: shoulderJoint),
            let elbowTransform = skeleton.modelTransform(for: elbowJoint),
            let wristTransform = skeleton.modelTransform(for: wristJoint)
        else {
            if !streamOnly { armMeshOverlay.hide() }
            return
        }

        // Convert from model space to world space (for angle computation + WebSocket).
        let worldShoulder = bodyTransform * shoulderTransform
        let worldElbow = bodyTransform * elbowTransform
        let worldWrist = bodyTransform * wristTransform

        // World-space positions for angle calculation.
        let shoulderPos = SIMD3<Float>(worldShoulder.columns.3.x, worldShoulder.columns.3.y, worldShoulder.columns.3.z)
        let elbowPos = SIMD3<Float>(worldElbow.columns.3.x, worldElbow.columns.3.y, worldElbow.columns.3.z)
        let wristPos = SIMD3<Float>(worldWrist.columns.3.x, worldWrist.columns.3.y, worldWrist.columns.3.z)

        // Compute joint angles from the 3D positions.
        let angles = JointAngleCalculator.computeAngles(
            shoulder: shoulderPos,
            elbow: elbowPos,
            wrist: wristPos,
            shoulderTransform: worldShoulder,
            elbowTransform: worldElbow,
            wristTransform: worldWrist
        )

        armState.isBodyTracked = true
        armState.shoulderPosition = shoulderPos
        armState.elbowPosition = elbowPos
        armState.wristPosition = wristPos
        armState.shoulderYaw = angles.shoulderYaw
        armState.shoulderPitch = angles.shoulderPitch
        armState.elbowPitch = angles.elbowPitch
        armState.wristPitch = angles.wristPitch
        armState.wristRoll = angles.wristRoll

        // Skip mesh-overlay work entirely in stream-only mode — generating a
        // fresh tube mesh per frame is the biggest CPU/allocation cost here.
        if !streamOnly {
            // Model-space positions for the 3D mesh overlay.
            // The overlay uses AnchorEntity(.body) which tracks the body root,
            // so positions must be relative to the body (not world space).
            let modelShoulderPos = SIMD3<Float>(shoulderTransform.columns.3.x, shoulderTransform.columns.3.y, shoulderTransform.columns.3.z)
            let modelElbowPos = SIMD3<Float>(elbowTransform.columns.3.x, elbowTransform.columns.3.y, elbowTransform.columns.3.z)
            let modelWristPos = SIMD3<Float>(wristTransform.columns.3.x, wristTransform.columns.3.y, wristTransform.columns.3.z)

            armMeshOverlay.update(
                shoulder: modelShoulderPos,
                elbow: modelElbowPos,
                wrist: modelWristPos,
                gripAmount: armState.gripperOpenAmount
            )
        }
    }
}

// MARK: - ARSessionDelegate

extension BodyTrackingManager: @preconcurrency ARSessionDelegate {

    nonisolated func session(_ session: ARSession, didUpdate frame: ARFrame) {
        // Read the body anchor straight from the frame so we get a fresh
        // skeleton every camera tick (~60 Hz). Doing it from
        // `didUpdate(anchors:)` froze the angles because ARKit only fires
        // that callback on anchor changes — when the body anchor isn't
        // re-emitted (lost confidence, occluded, etc.) the skeleton would
        // freeze at its last value while body tracking still reported OK.
        let bodyAnchor = frame.anchors.compactMap({ $0 as? ARBodyAnchor }).first
        Task { @MainActor [weak self] in
            guard let self else { return }
            if let bodyAnchor {
                self.processBodyAnchor(bodyAnchor)
            } else {
                self.armState.isBodyTracked = false
            }
        }

        // Feed each camera frame to the hand pose detector.
        handPoseDetector.processFrame(frame.capturedImage)

        // Stream-only mode: also encode + send the camera frame as H.264.
        // Encoding runs on dedicated AVE silicon, so it doesn't compete with
        // ARKit/Vision for CPU/GPU/Neural Engine.
        let captured = frame.capturedImage
        let timestamp = frame.timestamp
        Task { @MainActor [weak self] in
            self?.encodeFrameIfStreaming(captured, timestamp: timestamp)
        }

        // Extract 2D arm skeleton from the frame's detected body.
        // ARSkeleton2D gives normalized image coordinates directly —
        // no ARView.project() needed, so no dependency on the view lifecycle.
        if let skeleton2D = frame.detectedBody?.skeleton {
            let rightArm = Self.extractArmLandmarks(from: skeleton2D, side: "right")
            let leftArm = Self.extractArmLandmarks(from: skeleton2D, side: "left")
            Task { @MainActor [weak self] in
                guard let self else { return }
                self.armLandmarks2D = self.trackRightArm ? rightArm : leftArm
            }
        }
    }

    /// Encode and broadcast one frame if stream-only mode is active.
    /// Lazily creates the encoder on the first frame so it picks up the
    /// actual ARKit buffer dimensions (varies by device + orientation).
    @MainActor
    private func encodeFrameIfStreaming(_ pixelBuffer: CVPixelBuffer, timestamp: TimeInterval) {
        guard streamOnly else { return }

        if videoEncoder == nil {
            let width = Int32(CVPixelBufferGetWidth(pixelBuffer))
            let height = Int32(CVPixelBufferGetHeight(pixelBuffer))
            let client = self.webSocketClient
            videoEncoder = VideoEncoder(width: width, height: height) { data in
                Task { @MainActor in
                    client.send(rawData: data)
                }
            }
        }

        let pts = CMTime(seconds: timestamp, preferredTimescale: 600)
        videoEncoder?.encode(pixelBuffer, presentationTime: pts)
    }

    /// Extract 2D arm joint positions from an ARSkeleton2D.
    ///
    /// Returns [shoulder, elbow, wrist] as optional CGPoints in Vision-style
    /// normalized coordinates (origin bottom-left, y up) so the overlay view
    /// can use a single coordinate conversion for both arm and hand data.
    ///
    /// **Coordinate rotation:** ARSkeleton2D landmarks are in the camera
    /// sensor's native space (landscape). For a rear camera in portrait-up
    /// (CGImagePropertyOrientation `.right`), we rotate 90° CW:
    ///   - displayX = sensorY
    ///   - displayY = 1 - sensorX
    ///
    /// Then converting to Vision convention (bottom-left, y-up):
    ///   - visionX = displayX = sensorY
    ///   - visionY = 1 - displayY = sensorX
    private nonisolated static func extractArmLandmarks(
        from skeleton: ARSkeleton2D,
        side: String
    ) -> [CGPoint?] {
        let jointNames = [
            "\(side)_shoulder_1_joint",
            "\(side)_forearm_joint",
            "\(side)_hand_joint",
        ]

        return jointNames.map { name -> CGPoint? in
            let jointName = ARSkeleton.JointName(rawValue: name)
            let index = skeleton.definition.index(for: jointName)
            guard index != NSNotFound,
                  skeleton.isJointTracked(index) else {
                return nil
            }
            let landmark = skeleton.jointLandmarks[index]
            // Rotate from landscape sensor space to portrait display,
            // stored in Vision convention (bottom-left origin, y up).
            return CGPoint(x: CGFloat(landmark.y), y: CGFloat(landmark.x))
        }
    }
}
