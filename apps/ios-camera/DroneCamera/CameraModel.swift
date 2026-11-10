import AVFoundation
import CoreImage
import ImageIO
import SwiftUI

final class CameraModel: NSObject, ObservableObject, AVCaptureVideoDataOutputSampleBufferDelegate {
    @Published var running = false
    @Published var status = "같은 Wi-Fi 또는 USB로 연결 · 후면 카메라"
    @Published var url = ""
    @Published var targetFps = 30
    let session = AVCaptureSession()
    private let captureQueue = DispatchQueue(label: "camera.capture")
    private let context = CIContext(options: [.cacheIntermediates: false])
    private var frames = LatestJpeg()
    private var server: MjpegServer?
    private var lastEncoded: Double = 0
    private var generation = UUID()
    private var sessionToken: String?
    private var sessionPort: UInt16?
    private var encodingFps = 30

    func start() {
        guard !running else { return }
        let generation = UUID(); self.generation = generation
        running = true; status = "카메라 권한 확인 중…"
        AVCaptureDevice.requestAccess(for: .video) { [weak self] allowed in
            DispatchQueue.main.async {
                guard let self = self, self.running, self.generation == generation else { return }
                if allowed { self.configure(generation) }
                else { self.stop(message: "설정에서 카메라 권한을 허용하세요.") }
            }
        }
    }
    private func configure(_ generation: UUID) {
        let requestedFps = targetFps
        let frames = LatestJpeg()
        let server = MjpegServer(frames: frames, token: sessionToken, port: sessionPort); self.server = server
        server.ready = { [weak self] port, token in
            DispatchQueue.main.async {
                guard let self = self, self.running, self.generation == generation else { return }
                self.sessionPort = port
                self.url = "http://\(MjpegServer.wifiAddress()):\(port)/stream.mjpg?token=\(token)"
                self.status = "MJPEG · 최대 1280×720 / \(requestedFps) FPS · 앱을 열린 상태로 유지"
            }
        }
        server.failed = { [weak self] in DispatchQueue.main.async {
            guard let self = self, self.generation == generation else { return }
            self.stop(message: "네트워크 송출을 시작할 수 없습니다. 다시 시작하세요.")
        } }
        do { try server.start(); sessionToken = server.token } catch { stop(message: "네트워크 서버 시작 실패"); return }
        UIApplication.shared.isIdleTimerDisabled = true
        captureQueue.async { [weak self] in
            guard let self = self else { return }
            self.frames = frames
            self.encodingFps = requestedFps
            do {
                self.session.beginConfiguration()
                defer { self.session.commitConfiguration() }
                self.session.inputs.forEach { self.session.removeInput($0) }
                self.session.outputs.forEach { self.session.removeOutput($0) }
                self.session.sessionPreset = self.session.canSetSessionPreset(.hd1280x720) ? .hd1280x720 : .medium
                guard let device = AVCaptureDevice.default(.builtInWideAngleCamera, for: .video, position: .back) else {
                    throw NSError(domain: "DroneCamera", code: 2)
                }
                let input = try AVCaptureDeviceInput(device: device)
                guard self.session.canAddInput(input) else { throw NSError(domain: "DroneCamera", code: 3) }
                self.session.addInput(input)
                let output = AVCaptureVideoDataOutput()
                output.alwaysDiscardsLateVideoFrames = true
                output.videoSettings = [kCVPixelBufferPixelFormatTypeKey as String: kCVPixelFormatType_32BGRA]
                output.setSampleBufferDelegate(self, queue: self.captureQueue)
                guard self.session.canAddOutput(output) else { throw NSError(domain: "DroneCamera", code: 4) }
                self.session.addOutput(output)
                if let connection = output.connection(with: .video), connection.isVideoOrientationSupported {
                    connection.videoOrientation = .landscapeRight
                }
                self.lastEncoded = 0
            } catch {
                DispatchQueue.main.async { self.stop(message: "후면 카메라를 열 수 없습니다. 다른 앱의 사용 여부를 확인하세요.") }
                return
            }
            self.session.startRunning()
        }
    }
    func captureOutput(_ output: AVCaptureOutput, didOutput sampleBuffer: CMSampleBuffer, from connection: AVCaptureConnection) {
        let now = ProcessInfo.processInfo.systemUptime
        guard now - lastEncoded >= 1.0 / Double(encodingFps), let pixel = CMSampleBufferGetImageBuffer(sampleBuffer) else { return }
        lastEncoded = now
        let image = CIImage(cvPixelBuffer: pixel)
        let colorSpace = CGColorSpaceCreateDeviceRGB()
        if let jpeg = context.jpegRepresentation(of: image, colorSpace: colorSpace,
                options: [CIImageRepresentationOption(rawValue: kCGImageDestinationLossyCompressionQuality as String): 0.7]) {
            frames.publish(jpeg)
        }
    }
    func stop(message: String = "송출 중지") {
        generation = UUID(); running = false; url = ""; status = message
        UIApplication.shared.isIdleTimerDisabled = false
        server?.stop(); server = nil
        captureQueue.async { [weak self] in self?.session.stopRunning() }
    }
}
