import Foundation
import Network

/// Discovers robot control servers on the local network via Bonjour (mDNS).
///
/// Browses for `_armtracker._tcp` services using `NWBrowser` (Apple's modern
/// Network framework). When a service is found, it resolves the endpoint to
/// an IP:port and constructs a WebSocket URL.
///
/// This replaces the legacy `NSNetServiceBrowser` API with a cleaner,
/// concurrency-friendly interface.
@MainActor
final class BonjourDiscovery: ObservableObject {

    /// Discovered servers on the local network.
    @Published var servers: [NWBrowser.Result] = []

    /// The resolved WebSocket URL of the selected server, ready for connection.
    @Published var selectedServerURL: URL?

    private var browser: NWBrowser?

    private static let serviceType = "_armtracker._tcp"

    // MARK: - Browsing Lifecycle

    /// Start browsing for ArmTracker servers on the local network.
    func startBrowsing() {
        stopBrowsing()

        let params = NWParameters()
        params.includePeerToPeer = true

        let browser = NWBrowser(
            for: .bonjour(type: Self.serviceType, domain: nil),
            using: params
        )

        browser.stateUpdateHandler = { [weak self] state in
            Task { @MainActor [weak self] in
                switch state {
                case .ready:
                    print("Bonjour browser ready, searching for servers...")
                case .failed(let error):
                    print("Bonjour browser failed: \(error)")
                    self?.stopBrowsing()
                case .cancelled:
                    break
                default:
                    break
                }
            }
        }

        browser.browseResultsChangedHandler = { [weak self] results, _ in
            Task { @MainActor [weak self] in
                guard let self else { return }
                self.servers = Array(results)

                // Auto-select the first server if none is selected.
                if self.selectedServerURL == nil, let first = results.first {
                    self.resolveEndpoint(first.endpoint)
                }
            }
        }

        browser.start(queue: .main)
        self.browser = browser
    }

    /// Stop browsing for servers.
    func stopBrowsing() {
        browser?.cancel()
        browser = nil
        servers = []
    }

    // MARK: - Endpoint Resolution

    /// Resolve a Bonjour endpoint to an IP:port, then build a WebSocket URL.
    ///
    /// Uses a temporary `NWConnection` to trigger DNS-SD resolution —
    /// once the connection reaches `.ready`, we can extract the resolved
    /// IP address and port from the connection's path.
    func resolveEndpoint(_ endpoint: NWEndpoint) {
        let connection = NWConnection(to: endpoint, using: .tcp)

        connection.stateUpdateHandler = { [weak self] state in
            Task { @MainActor [weak self] in
                switch state {
                case .ready:
                    if let resolved = connection.currentPath?.remoteEndpoint,
                       case let .hostPort(host, port) = resolved
                    {
                        let hostString = Self.cleanHostString("\(host)")
                        let url = URL(string: "ws://\(hostString):\(port)/ws")
                        self?.selectedServerURL = url
                        print("Resolved server: \(url?.absoluteString ?? "nil")")
                    }
                    connection.cancel()

                case .failed(let error):
                    print("Bonjour resolution failed: \(error)")
                    connection.cancel()

                default:
                    break
                }
            }
        }

        connection.start(queue: .main)
    }

    /// Strip interface scope IDs from IPv6 addresses (e.g., "fe80::1%en0" → "fe80::1").
    private static func cleanHostString(_ host: String) -> String {
        if let percentRange = host.range(of: "%") {
            return String(host[host.startIndex..<percentRange.lowerBound])
        }
        return host
    }
}
