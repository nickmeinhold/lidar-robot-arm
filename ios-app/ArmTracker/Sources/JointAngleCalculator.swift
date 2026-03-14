import simd

/// Computes joint angles from 3D skeleton positions for the SO-100 robot arm.
///
/// The SO-100 has 6 DOF:
///   1. **Shoulder Yaw** (base rotation) — horizontal rotation of the whole arm
///   2. **Shoulder Pitch** — arm raising up/down
///   3. **Elbow Pitch** — forearm bend
///   4. **Wrist Pitch** — hand tilt up/down
///   5. **Wrist Roll** — hand rotation
///   6. **Gripper** — handled separately by HandPoseDetector
///
/// We derive these from the 3D joint positions and rotation matrices
/// provided by ARKit's body tracking.
enum JointAngleCalculator {

    struct ArmAngles {
        var shoulderYaw: Float
        var shoulderPitch: Float
        var elbowPitch: Float
        var wristPitch: Float
        var wristRoll: Float
    }

    /// Compute all arm angles from the tracked joint positions and transforms.
    ///
    /// - Parameters:
    ///   - shoulder: 3D world position of the shoulder joint.
    ///   - elbow: 3D world position of the elbow joint.
    ///   - wrist: 3D world position of the wrist joint.
    ///   - shoulderTransform: Full 4x4 transform of the shoulder.
    ///   - elbowTransform: Full 4x4 transform of the elbow.
    ///   - wristTransform: Full 4x4 transform of the wrist.
    static func computeAngles(
        shoulder: SIMD3<Float>,
        elbow: SIMD3<Float>,
        wrist: SIMD3<Float>,
        shoulderTransform: simd_float4x4,
        elbowTransform: simd_float4x4,
        wristTransform: simd_float4x4
    ) -> ArmAngles {

        // --- Shoulder Yaw (base rotation) ---
        // Project the shoulder→elbow vector onto the horizontal (XZ) plane.
        // atan2 gives us the angle from the forward direction.
        let upperArm = elbow - shoulder
        let shoulderYaw = atan2(upperArm.x, upperArm.z)

        // --- Shoulder Pitch (arm raise) ---
        // Angle of the upper arm relative to straight down (-Y axis).
        let upperArmLength = length(upperArm)
        let shoulderPitch: Float
        if upperArmLength > 0.001 {
            // Dot product with down vector gives cosine of the angle.
            let downVector = SIMD3<Float>(0, -1, 0)
            let cosAngle = dot(normalize(upperArm), downVector)
            shoulderPitch = acos(max(-1, min(1, cosAngle)))
        } else {
            shoulderPitch = 0
        }

        // --- Elbow Pitch (forearm bend) ---
        // The angle between the upper arm and forearm vectors.
        let forearm = wrist - elbow
        let elbowAngle = angleBetween(upperArm, forearm)
        // Convert from the full angle to the bend angle.
        // When arm is straight: angleBetween ≈ π, bend = 0
        // When fully bent: angleBetween ≈ 0, bend = π
        let elbowPitch = Float.pi - elbowAngle

        // --- Wrist Pitch ---
        // Extract from the wrist transform's rotation relative to the forearm.
        // We use the Y-axis rotation of the wrist transform.
        let wristRotation = extractEulerAngles(from: wristTransform)
        let elbowRotation = extractEulerAngles(from: elbowTransform)
        let wristPitch = wristRotation.x - elbowRotation.x

        // --- Wrist Roll ---
        let wristRoll = wristRotation.z - elbowRotation.z

        return ArmAngles(
            shoulderYaw: shoulderYaw,
            shoulderPitch: shoulderPitch,
            elbowPitch: elbowPitch,
            wristPitch: wristPitch,
            wristRoll: wristRoll
        )
    }

    /// Angle between two 3D vectors in radians.
    private static func angleBetween(_ a: SIMD3<Float>, _ b: SIMD3<Float>) -> Float {
        let lenA = length(a)
        let lenB = length(b)
        guard lenA > 0.001, lenB > 0.001 else { return 0 }
        let cosAngle = dot(a, b) / (lenA * lenB)
        return acos(max(-1, min(1, cosAngle)))
    }

    /// Extract Euler angles (pitch, yaw, roll) from a 4x4 transform matrix.
    ///
    /// Uses the ZYX convention to decompose the rotation matrix.
    private static func extractEulerAngles(from transform: simd_float4x4) -> SIMD3<Float> {
        let m = transform
        let sy = sqrt(m.columns.0.x * m.columns.0.x + m.columns.1.x * m.columns.1.x)

        let singular = sy < 1e-6

        let x: Float // pitch
        let y: Float // yaw
        let z: Float // roll

        if !singular {
            x = atan2(m.columns.2.y, m.columns.2.z)
            y = atan2(-m.columns.2.x, sy)
            z = atan2(m.columns.1.x, m.columns.0.x)
        } else {
            x = atan2(-m.columns.1.z, m.columns.1.y)
            y = atan2(-m.columns.2.x, sy)
            z = 0
        }

        return SIMD3<Float>(x, y, z)
    }
}
