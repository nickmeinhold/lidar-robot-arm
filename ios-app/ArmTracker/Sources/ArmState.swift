import Foundation
import simd

/// Wraps a non-Sendable value for use across concurrency boundaries.
/// Only safe when the value is not accessed concurrently after sending.
struct UncheckedSendableBox<T>: @unchecked Sendable {
    let value: T
    init(_ value: T) { self.value = value }
}

/// Represents the current state of the tracked arm, ready to send to the robot.
///
/// The SO-100 has 6 degrees of freedom:
///   1. Base rotation (shoulder yaw)
///   2. Shoulder pitch
///   3. Elbow pitch
///   4. Wrist pitch
///   5. Wrist roll
///   6. Gripper (open/close)
struct ArmState {
    /// Joint angles in radians, mapped to the 6 SO-100 servos.
    var shoulderYaw: Float = 0      // Base rotation: left/right
    var shoulderPitch: Float = 0    // Shoulder: up/down
    var elbowPitch: Float = 0       // Elbow bend
    var wristPitch: Float = 0       // Wrist up/down
    var wristRoll: Float = 0        // Wrist rotation
    var gripperOpenAmount: Float = 1 // 1.0 = fully open, 0.0 = closed

    /// Whether a body is currently being tracked.
    var isBodyTracked: Bool = false

    /// Whether a hand is currently being tracked.
    var isHandTracked: Bool = false

    /// Raw 3D positions of tracked joints (in AR world space).
    var shoulderPosition: SIMD3<Float>?
    var elbowPosition: SIMD3<Float>?
    var wristPosition: SIMD3<Float>?

    /// All angles as a dictionary, convenient for serialization.
    var anglesDictionary: [String: Float] {
        [
            "shoulder_yaw": shoulderYaw,
            "shoulder_pitch": shoulderPitch,
            "elbow_pitch": elbowPitch,
            "wrist_pitch": wristPitch,
            "wrist_roll": wristRoll,
            "gripper": gripperOpenAmount,
        ]
    }

    /// Angles converted to degrees for display.
    var anglesDegreesForDisplay: [(String, Float)] {
        [
            ("Shoulder Yaw", shoulderYaw.degrees),
            ("Shoulder Pitch", shoulderPitch.degrees),
            ("Elbow", elbowPitch.degrees),
            ("Wrist Pitch", wristPitch.degrees),
            ("Wrist Roll", wristRoll.degrees),
            ("Gripper %", gripperOpenAmount * 100),
        ]
    }
}

extension Float {
    /// Convert radians to degrees.
    var degrees: Float { self * 180 / .pi }

    /// Convert degrees to radians.
    var radians: Float { self * .pi / 180 }
}
