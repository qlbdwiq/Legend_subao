import Foundation

struct JpegFrame {
    let sequence: UInt64
    let jpeg: Data
}

/// One latest packet, shared by capture and independently paced network clients.
final class LatestJpeg {
    private let lock = NSLock()
    private var value: JpegFrame?
    func publish(_ jpeg: Data) {
        lock.lock(); defer { lock.unlock() }
        value = JpegFrame(sequence: (value?.sequence ?? 0) + 1, jpeg: jpeg)
    }
    func after(_ sequence: UInt64) -> JpegFrame? {
        lock.lock(); defer { lock.unlock() }
        guard let frame = value, frame.sequence > sequence else { return nil }
        return frame
    }
}

enum StreamProtocol {
    static let contentType = "multipart/x-mixed-replace; boundary=droneframe"
    static func authorized(_ request: String, token: String) -> Bool {
        let line = request.components(separatedBy: "\r\n")[0]
        let expected = "GET /stream.mjpg?token=\(token) HTTP/1.1"
        let a = Array(line.utf8), b = Array(expected.utf8)
        guard a.count == b.count else { return false }
        return zip(a,b).reduce(UInt8(0)) { $0 | ($1.0 ^ $1.1) } == 0
    }
    static func authorizedCapabilities(_ request: String, token: String) -> Bool {
        let line = request.components(separatedBy: "\r\n")[0]
        let expected = "GET /capabilities?token=\(token) HTTP/1.1"
        let a = Array(line.utf8), b = Array(expected.utf8)
        guard a.count == b.count else { return false }
        return zip(a,b).reduce(UInt8(0)) { $0 | ($1.0 ^ $1.1) } == 0
    }
    static func isCapabilitiesRequest(_ request: String) -> Bool {
        request.components(separatedBy: "\r\n")[0].hasPrefix("GET /capabilities")
    }
    static func packet(_ frame: JpegFrame) -> Data {
        var result = Data("--droneframe\r\nContent-Type: image/jpeg\r\nContent-Length: \(frame.jpeg.count)\r\nX-Frame-Sequence: \(frame.sequence)\r\n\r\n".utf8)
        result.append(frame.jpeg); result.append(Data("\r\n".utf8))
        return result
    }
}
