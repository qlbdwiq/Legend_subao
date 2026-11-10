import AVFoundation
import CoreImage
import ImageIO
import SwiftUI

final class CameraModel: NSObject, ObservableObject, AVCaptureVideoDataOutputSampleBufferDelegate {
    @Published var running = false
    @Published var status = "같은 Wi-Fi 또는 USB로 연결 · 후면 카메라"
    @Published var url = ""
    @Published var targetFps = 30
    @Published var modes: [StreamModeOption] = []
    @Published var selectedModeIndex = 0
    @Published var useFrontCamera = false
    let session = AVCaptureSession()
    private let captureQueue = DispatchQueue(label: "camera.capture")
    private let context = CIContext(options: [.cacheIntermediates: false])
    private var frames = LatestJpeg()
    private var server: MjpegServer?
    private var nextEncodeAt: Double = 0
    private var generation = UUID()
    private var sessionToken: String?
    private var sessionPort: UInt16?
    private var encodingFps = 30
    private var formatsByResolution: [String: AVCaptureDevice.Format] = [:]
    private var encodedInWindow = 0
    private var fpsWindowStart: Double = 0
    private var activeLabel = ""

    override init() {
        super.init()
        refreshModes()
    }

    var selectedMode: StreamModeOption? {
        modes.indices.contains(selectedModeIndex) ? modes[selectedModeIndex] : nil
    }

    var fpsChoices: [Int] {
        guard let mode = selectedMode else { return [15, 30] }
        return StreamCapabilities.fpsOptions(for: mode)
    }

    /// Enumerate AVFoundation data-output formats for the selected lens.
    /// Front and back cameras are enumerated separately; recording-only
    /// formats are never offered for live streaming.
    func refreshModes() {
        let position: AVCaptureDevice.Position = useFrontCamera ? .front : .back
        guard let device = AVCaptureDevice.default(.builtInWideAngleCamera, for: .video, position: position) else {
            modes = []
            status = "선택한 방향의 카메라를 찾을 수 없습니다."
            return
        }
        var options: [StreamModeOption] = []
        formatsByResolution = [:]
        for format in device.formats {
            let dimensions = CMVideoFormatDescriptionGetDimensions(format.formatDescription)
            let maxFps = Int(format.videoSupportedFrameRateRanges.map(\.maxFrameRate).max() ?? 0)
            let mode = StreamModeOption(width: Int(dimensions.width), height: Int(dimensions.height), maxFps: maxFps)
            options.append(mode)
            let key = "\(mode.width)x\(mode.height)"
            if let existing = formatsByResolution[key] {
                let existingMax = Int(existing.videoSupportedFrameRateRanges.map(\.maxFrameRate).max() ?? 0)
                if maxFps <= existingMax { continue }
            }
            formatsByResolution[key] = format
        }
        modes = StreamCapabilities.normalize(options)
        selectedModeIndex = StreamCapabilities.defaultIndex(modes)
        if let mode = selectedMode, !StreamCapabilities.fpsOptions(for: mode).contains(targetFps) {
            targetFps = StreamCapabilities.fpsOptions(for: mode).contains(30) ? 30 : mode.maxFps
        }
        if let best = StreamCapabilities.bestResolution(modes), let fastest = StreamCapabilities.bestFps(modes), !running {
            status = "같은 Wi-Fi 또는 USB로 연결 · \(useFrontCamera ? "전면" : "후면") 카메라 · 최고 \(best.label) @ \(best.maxFps) FPS · 최고 속도 \(fastest.label) @ \(fastest.maxFps) FPS"
        }
    }

    func start() {
        guard !running else { return }
        if modes.isEmpty { refreshModes() }
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
        let selected = selectedMode
        let allModes = modes
        let facingName = useFrontCamera ? "front" : "back"
        let frames = LatestJpeg()
        let server = MjpegServer(frames: frames, token: sessionToken, port: sessionPort); self.server = server
        server.capabilitiesProvider = {
            StreamCapabilities.capabilitiesData(facing: facingName, modes: allModes, selected: selected)
        }
        server.ready = { [weak self] port, token in
            DispatchQueue.main.async {
                guard let self = self, self.running, self.generation == generation else { return }
                self.sessionPort = port
                self.url = "http://\(MjpegServer.wifiAddress()):\(port)/stream.mjpg?token=\(token)"
                if let selected = selected {
                    self.status = "MJPEG · \(selected.label) / 설정 \(requestedFps) FPS · 앱을 열린 상태로 유지"
                } else {
                    self.status = "MJPEG · 설정 \(requestedFps) FPS · 앱을 열린 상태로 유지"
                }
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
                self.session.sessionPreset = .inputPriority
                let position: AVCaptureDevice.Position = self.useFrontCamera ? .front : .back
                guard let device = AVCaptureDevice.default(.builtInWideAngleCamera, for: .video, position: position) else {
                    throw NSError(domain: "DroneCamera", code: 2)
                }
                if let selected = selected,
                   let format = self.formatsByResolution["\(selected.width)x\(selected.height)"] {
                    try device.lockForConfiguration()
                    device.activeFormat = format
                    let duration = CMTime(value: 1, timescale: CMTimeScale(max(1, requestedFps)))
                    if format.videoSupportedFrameRateRanges.contains(where: {
                        Double(requestedFps) >= $0.minFrameRate && Double(requestedFps) <= $0.maxFrameRate
                    }) {
                        device.activeVideoMinFrameDuration = duration
                        device.activeVideoMaxFrameDuration = duration
                    }
                    device.unlockForConfiguration()
                    self.activeLabel = "\(selected.width) × \(selected.height) · 설정 \(requestedFps) FPS"
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
                self.nextEncodeAt = 0
                self.encodedInWindow = 0
                self.fpsWindowStart = 0
            } catch {
                DispatchQueue.main.async { self.stop(message: "선택한 해상도/FPS를 열 수 없습니다. 지원 범위에서 다시 선택하세요.") }
                return
            }
            self.session.startRunning()
        }
    }
    func captureOutput(_ output: AVCaptureOutput, didOutput sampleBuffer: CMSampleBuffer, from connection: AVCaptureConnection) {
        let now = ProcessInfo.processInfo.systemUptime
        guard now >= nextEncodeAt, let pixel = CMSampleBufferGetImageBuffer(sampleBuffer) else { return }
        nextEncodeAt = max(nextEncodeAt + 1.0 / Double(encodingFps), now)
        let image = CIImage(cvPixelBuffer: pixel)
        let colorSpace = CGColorSpaceCreateDeviceRGB()
        if let jpeg = context.jpegRepresentation(of: image, colorSpace: colorSpace,
                options: [CIImageRepresentationOption(rawValue: kCGImageDestinationLossyCompressionQuality as String): 0.7]) {
            frames.publish(jpeg)
            encodedInWindow += 1
            if fpsWindowStart == 0 { fpsWindowStart = now }
            let elapsed = now - fpsWindowStart
            if elapsed >= 1 {
                let measured = Double(encodedInWindow) / elapsed
                encodedInWindow = 0; fpsWindowStart = now
                let label = activeLabel
                DispatchQueue.main.async { [weak self] in
                    guard let self = self, self.running else { return }
                    self.status = "MJPEG · \(label) · 실제 인코딩 \(String(format: "%.1f", measured)) FPS · 앱을 열린 상태로 유지"
                }
            }
        }
    }
    func stop(message: String = "송출 중지") {
        generation = UUID(); running = false; url = ""; status = message
        UIApplication.shared.isIdleTimerDisabled = false
        server?.stop(); server = nil
        captureQueue.async { [weak self] in self?.session.stopRunning() }
    }
}
