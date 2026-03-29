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

    /// Which arm to track.
    var trackRightArm = true

    /// The hand pose detector runs on the same camera frames.
    let handPoseDetector = HandPoseDetector()

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
        $armState
            .throttle(for: .milliseconds(33), scheduler: RunLoop.main, latest: true)
            .sink { [weak self] state in
                self?.webSocketClient.send(state)
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
            return
        }

        // Convert from model space to world space.
        let worldShoulder = bodyTransform * shoulderTransform
        let worldElbow = bodyTransform * elbowTransform
        let worldWrist = bodyTransform * wristTransform

        // Extract positions from the 4x4 transform matrices.
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
    }
}

// MARK: - ARSessionDelegate

extension BodyTrackingManager: @preconcurrency ARSessionDelegate {

    nonisolated func session(_ session: ARSession, didUpdate anchors: [ARAnchor]) {
        guard let bodyAnchor = anchors.compactMap({ $0 as? ARBodyAnchor }).first else {
            return
        }
        // Hop to main actor to update published state.
        Task { @MainActor [weak self] in
            self?.processBodyAnchor(bodyAnchor)
        }
    }

    nonisolated func session(_ session: ARSession, didUpdate frame: ARFrame) {
        // Feed each camera frame to the hand pose detector.
        handPoseDetector.processFrame(frame.capturedImage)
    }
}
