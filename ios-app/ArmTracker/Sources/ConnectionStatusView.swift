import SwiftUI

/// Status indicator for the WebSocket connection to the robot control server.
///
/// Shows a colored dot (green/yellow/red) with latency info.
/// Long-press opens a manual server address entry sheet for when
/// Bonjour discovery isn't available (e.g., different subnets).
struct ConnectionStatusView: View {
    @ObservedObject var webSocketClient: WebSocketClient
    @ObservedObject var bonjourDiscovery: BonjourDiscovery

    @State private var showManualEntry = false
    @AppStorage("manualServerAddress") private var manualAddress = ""

    var body: some View {
        HStack(spacing: 6) {
            Circle()
                .fill(statusColor)
                .frame(width: 8, height: 8)
            Text("Server")
                .font(.caption)
                .foregroundColor(.white)
            if let latency = webSocketClient.latencyMs {
                Text(String(format: "%.0fms", latency))
                    .font(.caption2.monospacedDigit())
                    .foregroundColor(.white.opacity(0.7))
            }
        }
        .onLongPressGesture {
            showManualEntry = true
        }
        .sheet(isPresented: $showManualEntry) {
            manualEntrySheet
        }
    }

    private var statusColor: Color {
        switch webSocketClient.connectionState {
        case .connected: return .green
        case .connecting: return .yellow
        case .disconnected: return .red
        }
    }

    private var manualEntrySheet: some View {
        NavigationView {
            Form {
                Section(header: Text("Server Address")) {
                    TextField("ws://192.168.1.100:8765/ws", text: $manualAddress)
                        .keyboardType(.URL)
                        .autocapitalization(.none)
                        .disableAutocorrection(true)
                }

                Section {
                    Button("Connect") {
                        if let url = URL(string: manualAddress) {
                            webSocketClient.connect(to: url)
                            showManualEntry = false
                        }
                    }
                    .disabled(URL(string: manualAddress) == nil)

                    Button("Disconnect", role: .destructive) {
                        webSocketClient.disconnect()
                        showManualEntry = false
                    }
                    .disabled(webSocketClient.connectionState == .disconnected)
                }

                if !bonjourDiscovery.servers.isEmpty {
                    Section(header: Text("Discovered Servers")) {
                        ForEach(Array(bonjourDiscovery.servers.enumerated()), id: \.offset) { _, result in
                            Button {
                                bonjourDiscovery.resolveEndpoint(result.endpoint)
                                showManualEntry = false
                            } label: {
                                Text("\(result.endpoint)")
                                    .foregroundColor(.primary)
                            }
                        }
                    }
                }
            }
            .navigationTitle("Server Connection")
            .navigationBarTitleDisplayMode(.inline)
            .toolbar {
                ToolbarItem(placement: .cancellationAction) {
                    Button("Done") { showManualEntry = false }
                }
            }
        }
    }
}
