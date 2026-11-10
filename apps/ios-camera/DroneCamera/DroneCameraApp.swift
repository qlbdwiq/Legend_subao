import AVFoundation
import SwiftUI

@main struct DroneCameraApp: App {
    @StateObject private var camera = CameraModel()
    @Environment(\.scenePhase) private var scenePhase
    var body: some Scene {
        WindowGroup {
            VStack(alignment: .leading, spacing: 12) {
                Text("DRONE CAMERA  ·  iPhone / iPad").font(.title2.bold())
                CameraPreview(session: camera.session).background(Color.black).clipShape(RoundedRectangle(cornerRadius: 12))
                Text(camera.status).font(.subheadline)
                Text(camera.url.isEmpty ? "Wi-Fi / USB 연결 주소는 송출 시작 후 표시됩니다." : camera.url)
                    .font(.system(.caption, design: .monospaced)).foregroundStyle(.cyan).textSelection(.enabled)
                HStack {
                    Button(camera.running ? "송출 중지 / Stop" : "송출 시작 / Start") {
                        if camera.running { camera.stop() } else { camera.start() }
                    }.buttonStyle(.borderedProminent)
                    Button("연결 주소 복사 / Copy URL") { UIPasteboard.general.string = camera.url }
                        .buttonStyle(.bordered).disabled(camera.url.isEmpty)
                    Text("USB: 데스크톱의 USB 연결 도우미 사용").font(.caption)
                }
                HStack {
                    Picker("카메라", selection: $camera.useFrontCamera) {
                        Text("후면").tag(false)
                        Text("전면").tag(true)
                    }.pickerStyle(.segmented).frame(width: 140).disabled(camera.running)
                        .onChange(of: camera.useFrontCamera) { _ in camera.refreshModes() }
                    Picker("해상도", selection: $camera.selectedModeIndex) {
                        ForEach(camera.modes.indices, id: \.self) { index in
                            Text(camera.modes[index].label).tag(index)
                        }
                    }.disabled(camera.running)
                        .onChange(of: camera.selectedModeIndex) { _ in
                            if !camera.fpsChoices.contains(camera.targetFps) {
                                camera.targetFps = camera.fpsChoices.contains(30) ? 30 : (camera.fpsChoices.last ?? 30)
                            }
                        }
                    Picker("송출 FPS", selection: $camera.targetFps) {
                        ForEach(camera.fpsChoices, id: \.self) { fps in
                            Text("\(fps) FPS").tag(fps)
                        }
                    }.disabled(camera.running)
                }
            }.padding().foregroundStyle(.white).background(Color(red: 0.067, green: 0.094, blue: 0.153))
                .preferredColorScheme(.dark)
                .onChange(of: scenePhase) { phase in
                    if phase != .active { camera.stop(message: "앱이 비활성화되어 송출을 중지했습니다.") }
                }
        }
    }
}

private final class PreviewView: UIView {
    override class var layerClass: AnyClass { AVCaptureVideoPreviewLayer.self }
    var preview: AVCaptureVideoPreviewLayer { layer as! AVCaptureVideoPreviewLayer }
    override func layoutSubviews() {
        super.layoutSubviews()
        if let connection = preview.connection, connection.isVideoOrientationSupported {
            connection.videoOrientation = .landscapeRight
        }
    }
}
private struct CameraPreview: UIViewRepresentable {
    let session: AVCaptureSession
    func makeUIView(context: Context) -> PreviewView {
        let view = PreviewView(); view.preview.session = session; view.preview.videoGravity = .resizeAspect
        return view
    }
    func updateUIView(_ uiView: PreviewView, context: Context) { }
}
