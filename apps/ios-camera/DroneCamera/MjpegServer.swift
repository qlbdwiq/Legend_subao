import Foundation
import Network
import Security
import Darwin

final class MjpegServer {
    private let queue = DispatchQueue(label: "camera.http")
    private var listener: NWListener?
    private var clients: [UUID: NWConnection] = [:]
    private var active = false
    private let frames: LatestJpeg
    private(set) var token = ""
    private let preferredPort: UInt16?
    var ready: ((UInt16, String) -> Void)?
    var failed: (() -> Void)?
    /// Live-streaming modes JSON served at /capabilities (token required).
    var capabilitiesProvider: (() -> Data?)?

    init(frames: LatestJpeg, token: String? = nil, port: UInt16? = nil) {
        self.frames = frames; self.token = token ?? ""; self.preferredPort = port
    }

    func start() throws {
        if token.isEmpty {
            var random = [UInt8](repeating: 0, count: 16)
            guard SecRandomCopyBytes(kSecRandomDefault, random.count, &random) == errSecSuccess else {
                throw NSError(domain: "DroneCamera", code: 1)
            }
            token = random.map { String(format: "%02x", $0) }.joined()
        }
        let parameters = NWParameters.tcp
        parameters.allowLocalEndpointReuse = true
        let listener = try NWListener(using: parameters, on: preferredPort.flatMap(NWEndpoint.Port.init(rawValue:)) ?? .any)
        self.listener = listener
        queue.async { self.active = true }
        listener.stateUpdateHandler = { [weak self, weak listener] state in
            guard let self = self else { return }
            switch state {
            case .ready:
                if let port = listener?.port?.rawValue { self.ready?(port, self.token) }
            case .failed: self.failed?(); self.stop()
            default: break
            }
        }
        listener.newConnectionHandler = { [weak self] connection in self?.accept(connection) }
        listener.start(queue: queue)
    }
    func stop() {
        queue.async {
            self.active = false
            self.listener?.cancel(); self.listener = nil
            for connection in self.clients.values { connection.cancel() }
            self.clients.removeAll()
            self.streaming.removeAll()
        }
    }
    private func accept(_ connection: NWConnection) {
        guard active, clients.count < 2 else { connection.cancel(); return }
        let id = UUID()
        clients[id] = connection
        connection.stateUpdateHandler = { [weak self] state in
            if case .failed = state { self?.remove(id) }
            if case .cancelled = state { self?.clients.removeValue(forKey: id) }
        }
        connection.start(queue: queue)
        queue.asyncAfter(deadline: .now() + 3) { [weak self, weak connection] in
            guard let self = self, let connection = connection, self.clients[id] === connection else { return }
            // receiveRequest removes this timeout by transitioning to the stream set.
            if !self.streaming.contains(id) { self.remove(id) }
        }
        receiveRequest(connection, id: id, buffer: Data())
    }
    private var streaming: Set<UUID> = []
    private func remove(_ id: UUID) {
        clients.removeValue(forKey: id)?.cancel()
        streaming.remove(id)
    }
    private func receiveRequest(_ connection: NWConnection, id: UUID, buffer: Data) {
        connection.receive(minimumIncompleteLength: 1, maximumLength: 2048) { [weak self] data, _, complete, error in
            guard let self = self, self.active, self.clients[id] != nil else { return }
            var buffer = buffer; if let data = data { buffer.append(data) }
            if buffer.count > 8192 { self.remove(id); return }
            if buffer.range(of: Data("\r\n\r\n".utf8)) != nil {
                guard let request = String(data: buffer, encoding: .ascii) else {
                    connection.send(content: Data("HTTP/1.1 403 Forbidden\r\nContent-Length: 0\r\nConnection: close\r\n\r\n".utf8),
                                    completion: .contentProcessed { _ in self.remove(id) })
                    return
                }
                if StreamProtocol.isCapabilitiesRequest(request) {
                    guard StreamProtocol.authorizedCapabilities(request, token: self.token),
                          let body = self.capabilitiesProvider?() else {
                        connection.send(content: Data("HTTP/1.1 403 Forbidden\r\nContent-Length: 0\r\nConnection: close\r\n\r\n".utf8),
                                        completion: .contentProcessed { _ in self.remove(id) })
                        return
                    }
                    var response = Data("HTTP/1.1 200 OK\r\nContent-Type: application/json\r\nContent-Length: \(body.count)\r\nConnection: close\r\n\r\n".utf8)
                    response.append(body)
                    connection.send(content: response, completion: .contentProcessed { _ in self.remove(id) })
                    return
                }
                guard StreamProtocol.authorized(request, token: self.token) else {
                    connection.send(content: Data("HTTP/1.1 403 Forbidden\r\nContent-Length: 0\r\nConnection: close\r\n\r\n".utf8),
                                    completion: .contentProcessed { _ in self.remove(id) })
                    return
                }
                self.streaming.insert(id)
                let header = "HTTP/1.1 200 OK\r\nContent-Type: \(StreamProtocol.contentType)\r\nCache-Control: no-store\r\nConnection: close\r\n\r\n"
                connection.send(content: Data(header.utf8), completion: .contentProcessed { error in
                    if error == nil { self.sendLatest(connection, id: id, sequence: 0) } else { self.remove(id) }
                })
            } else if complete || error != nil { self.remove(id) }
            else { self.receiveRequest(connection, id: id, buffer: buffer) }
        }
    }
    private func sendLatest(_ connection: NWConnection, id: UUID, sequence: UInt64) {
        guard active, clients[id] != nil else { return }
        guard let frame = frames.after(sequence) else {
            queue.asyncAfter(deadline: .now() + 0.03) { [weak self] in self?.sendLatest(connection, id: id, sequence: sequence) }
            return
        }
        // Only one pending send. Slow clients skip frames instead of accumulating them.
        connection.send(content: StreamProtocol.packet(frame), completion: .contentProcessed { [weak self] error in
            guard let self = self else { return }
            if error != nil { self.remove(id) } else { self.sendLatest(connection, id: id, sequence: frame.sequence) }
        })
    }
    static func wifiAddress() -> String {
        var addresses: UnsafeMutablePointer<ifaddrs>?
        guard getifaddrs(&addresses) == 0 else { return "127.0.0.1" }
        defer { freeifaddrs(addresses) }
        var cursor = addresses
        while let entry = cursor {
            defer { cursor = entry.pointee.ifa_next }
            guard let address = entry.pointee.ifa_addr,
                  String(cString: entry.pointee.ifa_name) == "en0",
                  address.pointee.sa_family == UInt8(AF_INET) else { continue }
            var host = [CChar](repeating: 0, count: Int(NI_MAXHOST))
            if getnameinfo(address, socklen_t(address.pointee.sa_len), &host, socklen_t(host.count), nil, 0, NI_NUMERICHOST) == 0 {
                return String(cString: host)
            }
        }
        return "127.0.0.1" // USB forwarding does not require Wi-Fi.
    }
}
