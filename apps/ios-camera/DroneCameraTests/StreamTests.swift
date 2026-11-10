import XCTest
import Network
@testable import DroneCamera

final class StreamTests: XCTestCase {
    func testRealTcpServerStreamsMultipart() throws {
        let frames = LatestJpeg(); frames.publish(Data([255,216,255,217]))
        let server = MjpegServer(frames: frames)
        let done = expectation(description: "Authenticated HTTP MJPEG over real TCP")
        var client: NWConnection?
        server.ready = { port, token in
            let connection = NWConnection(host: "127.0.0.1", port: NWEndpoint.Port(rawValue: port)!, using: .tcp)
            client = connection
            connection.stateUpdateHandler = { state in
                if case .ready = state {
                    let request = Data("GET /stream.mjpg?token=\(token) HTTP/1.1\r\nHost: localhost\r\n\r\n".utf8)
                    connection.send(content: request, completion: .contentProcessed { error in
                        XCTAssertNil(error)
                        self.receive(connection, buffer: Data(), done: done)
                    })
                }
            }
            connection.start(queue: DispatchQueue(label: "test.http.client"))
        }
        defer { client?.cancel(); server.stop() }
        try server.start()
        wait(for: [done], timeout: 10)
    }
    private func receive(_ connection: NWConnection, buffer: Data, done: XCTestExpectation) {
        connection.receive(minimumIncompleteLength: 1, maximumLength: 4096) { data, _, complete, error in
            var buffer = buffer; if let data = data { buffer.append(data) }
            if buffer.range(of: Data([255,216,255,217])) != nil {
                XCTAssertTrue(buffer.starts(with: Data("HTTP/1.1 200 OK\r\n".utf8)))
                XCTAssertNotNil(buffer.range(of: Data("Content-Length: 4\r\n".utf8)))
                done.fulfill()
            } else if complete || error != nil || buffer.count > 8192 {
                XCTFail("TCP stream ended before an authenticated JPEG packet arrived")
                done.fulfill()
            } else { self.receive(connection, buffer: buffer, done: done) }
        }
    }
    func testPairingAndRejection() {
        let token = "0123456789abcdef0123456789abcdef"
        XCTAssertTrue(StreamProtocol.authorized("GET /stream.mjpg?token=\(token) HTTP/1.1\r\nHost: local\r\n\r\n", token: token))
        XCTAssertFalse(StreamProtocol.authorized("GET /stream.mjpg?token=wrong HTTP/1.1\r\n\r\n", token: token))
        XCTAssertFalse(StreamProtocol.authorized("GET /stream.mjpg HTTP/1.1\r\n\r\n", token: token))
    }
    func testLatestFrameAndMultipart() {
        let frames = LatestJpeg()
        frames.publish(Data([1,2])); frames.publish(Data([3,4,5]))
        let frame = frames.after(0)!
        XCTAssertEqual(frame.sequence, 2)
        XCTAssertEqual(frame.jpeg, Data([3,4,5]))
        XCTAssertNil(frames.after(2))
        let packet = StreamProtocol.packet(frame)
        XCTAssertTrue(packet.starts(with: Data("--droneframe\r\nContent-Type: image/jpeg\r\nContent-Length: 3\r\n".utf8)))
        XCTAssertTrue(packet.suffix(5) == Data([3,4,5,13,10]))
    }
}
