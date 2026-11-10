import Foundation

/// One live-streaming format enumerated from AVFoundation.
/// Recording formats are intentionally not used: a format that records
/// high-FPS video does not necessarily deliver data-output frames at
/// that rate, so only AVCaptureVideoDataOutput-capable formats appear.
struct StreamModeOption: Equatable {
    let width: Int
    let height: Int
    let maxFps: Int
    var pixels: Int { width * height }
    var label: String { "\(width) × \(height)" }
}

enum StreamCapabilities {
    static let standardFps = [15, 24, 30, 60, 120, 240]
    static let maxWidth = 3840
    static let maxHeight = 2160

    static func fpsOptions(for mode: StreamModeOption) -> [Int] {
        var options = standardFps.filter { $0 <= mode.maxFps }
        if mode.maxFps >= 1 && !options.contains(mode.maxFps) {
            options.append(mode.maxFps)
        }
        return options.sorted()
    }

    static func normalize(_ modes: [StreamModeOption]) -> [StreamModeOption] {
        var best: [String: StreamModeOption] = [:]
        for mode in modes {
            guard mode.width >= 16, mode.height >= 16,
                  mode.width <= maxWidth, mode.height <= maxHeight,
                  mode.width % 2 == 0, mode.height % 2 == 0, mode.maxFps >= 1 else { continue }
            let key = "\(mode.width)x\(mode.height)"
            if let existing = best[key], existing.maxFps >= mode.maxFps { continue }
            best[key] = mode
        }
        return best.values.sorted {
            $0.pixels != $1.pixels ? $0.pixels > $1.pixels : $0.maxFps > $1.maxFps
        }
    }

    static func bestResolution(_ modes: [StreamModeOption]) -> StreamModeOption? {
        normalize(modes).first
    }

    static func bestFps(_ modes: [StreamModeOption]) -> StreamModeOption? {
        normalize(modes).max { a, b in
            a.maxFps != b.maxFps ? a.maxFps < b.maxFps : a.pixels < b.pixels
        }
    }

    static func defaultIndex(_ modes: [StreamModeOption]) -> Int {
        if let index = modes.firstIndex(where: { $0.width == 1280 && $0.height == 720 }) {
            return index
        }
        if let index = modes.lastIndex(where: { $0.pixels <= 1280 * 720 }) {
            return index
        }
        return modes.isEmpty ? 0 : 0
    }

    static func capabilitiesData(facing: String, modes: [StreamModeOption],
                                 selected: StreamModeOption?) -> Data {
        var text = "{\"version\":1,\"facing\":\"\(facing)\",\"modes\":["
        text += modes.map {
            "{\"width\":\($0.width),\"height\":\($0.height),\"max_fps\":\($0.maxFps)}"
        }.joined(separator: ",")
        text += "]"
        if let selected = selected {
            text += ",\"selected\":{\"width\":\(selected.width),\"height\":\(selected.height),\"max_fps\":\(selected.maxFps)}"
        }
        return Data((text + "}").utf8)
    }
}
