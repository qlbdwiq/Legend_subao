import XCTest
@testable import DroneCamera

final class StreamTests: XCTestCase {
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
