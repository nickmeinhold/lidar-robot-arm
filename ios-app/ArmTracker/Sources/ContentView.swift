import SwiftUI

/// Main app view: AR camera feed with a debug overlay showing tracked joint angles.
struct ContentView: View {
    @StateObject private var bodyTracker = BodyTrackingManager()

    var body: some View {
        ZStack {
            // Full-screen AR camera feed.
            ARViewContainer(bodyTrackingManager: bodyTracker)
                .ignoresSafeArea()

            // Debug overlay in the top-right corner.
            VStack {
                HStack {
                    Spacer()
                    debugOverlay
                }
                Spacer()
                trackingStatusBar
            }
            .padding()
        }
    }

    /// Shows joint angles and grip state.
    private var debugOverlay: some View {
        VStack(alignment: .leading, spacing: 4) {
            Text("Arm Tracker")
                .font(.headline)
                .foregroundColor(.white)

            Divider()
                .background(Color.white)

            ForEach(bodyTracker.armState.anglesDegreesForDisplay, id: \.0) { name, value in
                HStack {
                    Text(name)
                        .font(.caption)
                        .foregroundColor(.white.opacity(0.7))
                    Spacer()
                    Text(String(format: "%.1f", value))
                        .font(.caption.monospacedDigit())
                        .foregroundColor(.white)
                }
            }

            Divider()
                .background(Color.white)

            gripIndicator
        }
        .padding(12)
        .frame(width: 180)
        .background(.ultraThinMaterial)
        .cornerRadius(12)
    }

    /// Visual grip indicator.
    private var gripIndicator: some View {
        VStack(alignment: .leading, spacing: 4) {
            Text("Grip")
                .font(.caption)
                .foregroundColor(.white.opacity(0.7))

            GeometryReader { geo in
                ZStack(alignment: .leading) {
                    RoundedRectangle(cornerRadius: 4)
                        .fill(Color.white.opacity(0.2))

                    RoundedRectangle(cornerRadius: 4)
                        .fill(gripColor)
                        .frame(width: geo.size.width * CGFloat(bodyTracker.armState.gripperOpenAmount))
                }
            }
            .frame(height: 8)

            Text(gripLabel)
                .font(.caption2)
                .foregroundColor(gripColor)
        }
    }

    /// Bottom status bar showing tracking state.
    private var trackingStatusBar: some View {
        HStack(spacing: 16) {
            statusDot(
                label: "Body",
                active: bodyTracker.armState.isBodyTracked
            )
            statusDot(
                label: "Hand",
                active: bodyTracker.armState.isHandTracked
            )
            Spacer()
            armToggle
        }
        .padding(12)
        .background(.ultraThinMaterial)
        .cornerRadius(12)
    }

    private func statusDot(label: String, active: Bool) -> some View {
        HStack(spacing: 6) {
            Circle()
                .fill(active ? Color.green : Color.red)
                .frame(width: 8, height: 8)
            Text(label)
                .font(.caption)
                .foregroundColor(.white)
        }
    }

    private var armToggle: some View {
        Button {
            bodyTracker.trackRightArm.toggle()
        } label: {
            Text(bodyTracker.trackRightArm ? "R Arm" : "L Arm")
                .font(.caption.bold())
                .foregroundColor(.white)
                .padding(.horizontal, 12)
                .padding(.vertical, 6)
                .background(Color.blue.opacity(0.6))
                .cornerRadius(8)
        }
    }

    private var gripColor: Color {
        let grip = bodyTracker.armState.gripperOpenAmount
        if grip > 0.7 { return .green }
        if grip > 0.3 { return .yellow }
        return .red
    }

    private var gripLabel: String {
        let grip = bodyTracker.armState.gripperOpenAmount
        if grip > 0.7 { return "OPEN" }
        if grip > 0.3 { return "PARTIAL" }
        return "CLOSED"
    }
}
