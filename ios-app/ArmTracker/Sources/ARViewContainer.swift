import SwiftUI
import ARKit
import RealityKit

/// SwiftUI wrapper around `ARView` that uses body tracking.
///
/// This connects the RealityKit AR view to our `BodyTrackingManager`
/// so the camera feed is displayed while body tracking runs in the background.
struct ARViewContainer: UIViewRepresentable {
    let bodyTrackingManager: BodyTrackingManager

    func makeUIView(context: Context) -> ARView {
        let arView = ARView(frame: .zero)

        // Share the AR session with our body tracking manager.
        // This lets us display the camera feed while tracking runs.
        arView.session = bodyTrackingManager.arSession

        // Start body tracking.
        bodyTrackingManager.startTracking()

        return arView
    }

    func updateUIView(_ uiView: ARView, context: Context) {
        // No dynamic updates needed — the AR session runs continuously.
    }
}
