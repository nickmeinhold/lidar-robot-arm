import Vision
import Combine

/// Detects hand open/close state using Apple's Vision framework.
///
/// Uses `VNDetectHumanHandPoseRequest` to track 21 hand landmarks per frame.
/// Grip amount is calculated by measuring the average distance from each
/// fingertip to the palm center — when fingers curl in, the distance
/// decreases, indicating a closed grip.
///
/// Vision processing runs on a background queue, and results are published
/// back to the main actor for UI consumption.
@MainActor
final class HandPoseDetector: ObservableObject {

    /// Grip amount: 1.0 = fully open, 0.0 = fully closed.
    @Published var gripAmount: Float = 1.0

    /// Whether a hand is currently detected in the frame.
    @Published var isHandDetected: Bool = false

    /// Throttle: skip frames to reduce CPU load.
    private var frameCount = 0
    private let processEveryNFrames = 2

    /// Process a camera frame for hand pose detection.
    /// - Parameter pixelBuffer: The camera frame from ARKit.
    nonisolated func processFrame(_ pixelBuffer: CVPixelBuffer) {
        // Wrap the non-Sendable CVPixelBuffer so we can pass it across isolation.
        let sendableBuffer = UncheckedSendableBox(pixelBuffer)
        Task.detached(priority: .userInitiated) { [weak self] in
            guard let self else { return }
            let result = HandPoseDetector.detectHandPose(in: sendableBuffer.value)
            await self.applyResult(result)
        }
    }

    /// Apply detection results on the main actor.
    private func applyResult(_ result: HandPoseResult) {
        switch result {
        case .noHand:
            isHandDetected = false
        case .detected(let grip):
            isHandDetected = true
            gripAmount = grip
        case .skip:
            break
        }
    }

    /// Result type for hand pose detection.
    private enum HandPoseResult: Sendable {
        case noHand
        case detected(Float)
        case skip
    }

    /// Pure function: run Vision hand pose detection on a pixel buffer.
    /// Returns the computed grip amount without touching any mutable state.
    private nonisolated static func detectHandPose(in pixelBuffer: CVPixelBuffer) -> HandPoseResult {
        let request = VNDetectHumanHandPoseRequest()
        request.maximumHandCount = 1

        let handler = VNImageRequestHandler(cvPixelBuffer: pixelBuffer, options: [:])
        do {
            try handler.perform([request])
        } catch {
            print("Hand pose detection failed: \(error)")
            return .skip
        }

        guard let observation = request.results?.first else {
            return .noHand
        }

        do {
            let grip = try computeGripAmount(from: observation)
            return .detected(grip)
        } catch {
            print("Failed to extract hand landmarks: \(error)")
            return .skip
        }
    }

    /// Compute grip amount from hand landmarks.
    ///
    /// Strategy: measure the average distance from each fingertip to the
    /// wrist point. When the hand is open, fingertips are far from the wrist.
    /// When closed (fist), they're close. We normalize to 0-1 range.
    private nonisolated static func computeGripAmount(
        from observation: VNHumanHandPoseObservation
    ) throws -> Float {
        // Get the wrist as our reference point.
        let wrist = try observation.recognizedPoint(.wrist)

        // Get all four fingertips (excluding thumb for more reliable grip detection).
        let indexTip = try observation.recognizedPoint(.indexTip)
        let middleTip = try observation.recognizedPoint(.middleTip)
        let ringTip = try observation.recognizedPoint(.ringTip)
        let littleTip = try observation.recognizedPoint(.littleTip)

        // Also get the middle finger MCP (knuckle) as a scale reference.
        let middleMCP = try observation.recognizedPoint(.middleMCP)

        let tips = [indexTip, middleTip, ringTip, littleTip]

        // Filter out low-confidence points.
        let confidentTips = tips.filter { $0.confidence > 0.3 }
        guard !confidentTips.isEmpty, wrist.confidence > 0.3, middleMCP.confidence > 0.3 else {
            return 0.5 // Unknown — return neutral.
        }

        // Hand scale: distance from wrist to middle knuckle.
        // This normalizes for different hand sizes and distances from camera.
        let handScale = pointDistance(wrist.location, middleMCP.location)
        guard handScale > 0.01 else { return 0.5 }

        // Average normalized distance from fingertips to wrist.
        let avgDistance = confidentTips.reduce(Float(0)) { sum, tip in
            sum + Float(pointDistance(tip.location, wrist.location)) / Float(handScale)
        } / Float(confidentTips.count)

        // Map to 0-1 range. Empirically:
        // - Open hand: avgDistance ~2.5-3.5 (fingers extended far from wrist)
        // - Closed fist: avgDistance ~1.0-1.5 (fingers curled near wrist)
        let openThreshold: Float = 3.0
        let closedThreshold: Float = 1.2
        let normalized = (avgDistance - closedThreshold) / (openThreshold - closedThreshold)
        return max(0, min(1, normalized))
    }

    /// Euclidean distance between two CGPoints.
    private nonisolated static func pointDistance(_ a: CGPoint, _ b: CGPoint) -> CGFloat {
        let dx = a.x - b.x
        let dy = a.y - b.y
        return sqrt(dx * dx + dy * dy)
    }
}
