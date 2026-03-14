import Foundation
import Combine

/// Connection states for the WebSocket client.
enum ConnectionState: Sendable {
    case disconnected
    case connecting
    case connected
}

/// Manages a WebSocket connection to the robot control server.
///
/// Sends `ArmState` frames as JSON and measures round-trip latency via
/// server pong echoes. Uses `URLSessionWebSocketTask` (no third-party deps).
///
/// Backpressure protection: if a send is still in-flight, new frames are
/// dropped — stale position data is useless for real-time servo control.
@MainActor
final class WebSocketClient: ObservableObject {

    @Published var connectionState: ConnectionState = .disconnected
    @Published var latencyMs: Double?

    /// The URL we're connected (or reconnecting) to.
    private(set) var serverURL: URL?

    private var webSocketTask: URLSessionWebSocketTask?
    private let session = URLSession(configuration: .default)

    /// Prevents overlapping sends — drop frame if previous send hasn't completed.
    private var isSending = false

    /// Exponential backoff state for reconnection.
    private var reconnectDelay: TimeInterval = 1.0
    private var reconnectTask: Task<Void, Never>?
    private var isIntentionalDisconnect = false

    private static let maxReconnectDelay: TimeInterval = 10.0
    private static let initialReconnectDelay: TimeInterval = 1.0

    // MARK: - Connection Lifecycle

    /// Connect to the given WebSocket server URL.
    func connect(to url: URL) {
        disconnect()
        isIntentionalDisconnect = false
        serverURL = url
        reconnectDelay = Self.initialReconnectDelay
        establishConnection()
    }

    /// Gracefully disconnect and stop reconnection attempts.
    func disconnect() {
        isIntentionalDisconnect = true
        reconnectTask?.cancel()
        reconnectTask = nil
        webSocketTask?.cancel(with: .goingAway, reason: nil)
        webSocketTask = nil
        connectionState = .disconnected
        latencyMs = nil
    }

    // MARK: - Sending

    /// Send the current arm state as JSON over the WebSocket.
    ///
    /// Drops the frame if the previous send hasn't completed yet.
    /// This is intentional — for real-time servo control, the latest
    /// state is always more valuable than a queued stale state.
    func send(_ state: ArmState) {
        guard connectionState != .disconnected, !isSending else { return }
        guard let json = state.toJSON() else { return }

        isSending = true
        let message = URLSessionWebSocketTask.Message.string(json)
        webSocketTask?.send(message) { [weak self] error in
            Task { @MainActor [weak self] in
                self?.isSending = false
                if let error {
                    print("WebSocket send error: \(error.localizedDescription)")
                }
            }
        }
    }

    // MARK: - Private

    private func establishConnection() {
        guard let url = serverURL else { return }
        connectionState = .connecting

        let task = session.webSocketTask(with: url)
        webSocketTask = task
        task.resume()

        // URLSessionWebSocketTask doesn't have a connect callback —
        // stay in .connecting until the first successful receive confirms it.
        receiveLoop()
    }

    /// Continuously listen for server messages (pong/latency echoes).
    private func receiveLoop() {
        webSocketTask?.receive { [weak self] result in
            Task { @MainActor [weak self] in
                guard let self else { return }
                switch result {
                case .success(let message):
                    if self.connectionState != .connected {
                        self.connectionState = .connected
                        self.reconnectDelay = Self.initialReconnectDelay
                    }
                    self.handleMessage(message)
                    self.receiveLoop() // Continue listening
                case .failure(let error):
                    print("WebSocket receive error: \(error.localizedDescription)")
                    self.handleDisconnection()
                }
            }
        }
    }

    /// Parse server messages — currently only handles latency pongs.
    private func handleMessage(_ message: URLSessionWebSocketTask.Message) {
        switch message {
        case .string(let text):
            guard let data = text.data(using: .utf8),
                  let dict = try? JSONSerialization.jsonObject(with: data) as? [String: Any],
                  let type = dict["type"] as? String
            else { return }

            if type == "pong", let timestamp = dict["timestamp"] as? Double {
                let rtt = Date().timeIntervalSince1970 - timestamp
                latencyMs = rtt * 1000
            }

        case .data:
            break // Binary messages not expected

        @unknown default:
            break
        }
    }

    /// Handle unexpected disconnection with exponential backoff reconnect.
    private func handleDisconnection() {
        guard !isIntentionalDisconnect else { return }
        connectionState = .disconnected
        latencyMs = nil
        webSocketTask = nil

        reconnectTask = Task { [weak self] in
            guard let self else { return }
            let delay = self.reconnectDelay
            print("WebSocket disconnected. Reconnecting in \(delay)s...")
            try? await Task.sleep(for: .seconds(delay))

            guard !Task.isCancelled else { return }
            self.reconnectDelay = min(self.reconnectDelay * 2, Self.maxReconnectDelay)
            self.establishConnection()
        }
    }
}
