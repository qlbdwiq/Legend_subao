package com.dronecollision.camera;

import java.util.ArrayList;
import java.util.List;
import java.util.Locale;

/** Pure resolution/FPS combination logic for the live MJPEG stream.
 *
 * Modes come from Camera2 streaming formats (YUV output sizes and their
 * minimum frame durations), never from video-recording profiles: a size
 * that records at 120 FPS does not necessarily stream at 120 FPS.
 */
public final class StreamModes {
    public static final int[] STANDARD_FPS = {15, 24, 30, 60, 120, 240};
    // Desktop receiver ingest buffer limit (3840x2160 BGR).
    public static final int MAX_WIDTH = 3840;
    public static final int MAX_HEIGHT = 2160;

    private StreamModes() { }

    public record Mode(int width, int height, int maxFps) {
        public int pixels() { return width * height; }
        public String label() { return width + " × " + height; }
    }

    /** Standard frame rates a mode can sustain, plus its exact maximum. */
    public static List<Integer> fpsOptions(int maxFps) {
        List<Integer> options = new ArrayList<>();
        for (int fps : STANDARD_FPS) if (fps <= maxFps) options.add(fps);
        if (maxFps >= 1 && !options.contains(maxFps)) options.add(maxFps);
        options.sort(Integer::compareTo);
        return options;
    }

    /** Highest sustainable FPS from a minimum frame duration and AE ceiling. */
    public static int maxFpsFromMinDuration(long minDurationNs, int aeMaxFps) {
        int fromDuration = minDurationNs > 0
                ? (int) Math.floor(1_000_000_000.0 / minDurationNs) : 0;
        if (fromDuration <= 0) return Math.max(aeMaxFps, 0);
        return aeMaxFps > 0 ? Math.min(fromDuration, aeMaxFps) : fromDuration;
    }

    public static boolean streamable(int width, int height) {
        return width >= 16 && height >= 16 && width <= MAX_WIDTH && height <= MAX_HEIGHT
                && width % 2 == 0 && height % 2 == 0;
    }

    /** Sort by pixels then FPS, descending; duplicate sizes keep max FPS. */
    public static List<Mode> normalize(List<Mode> modes) {
        List<Mode> result = new ArrayList<>();
        for (Mode candidate : modes) {
            if (!streamable(candidate.width(), candidate.height()) || candidate.maxFps() < 1) continue;
            int existing = -1;
            for (int i = 0; i < result.size(); i++) {
                Mode mode = result.get(i);
                if (mode.width() == candidate.width() && mode.height() == candidate.height()) existing = i;
            }
            if (existing < 0) result.add(candidate);
            else if (candidate.maxFps() > result.get(existing).maxFps()) result.set(existing, candidate);
        }
        result.sort((a, b) -> a.pixels() != b.pixels()
                ? Integer.compare(b.pixels(), a.pixels())
                : Integer.compare(b.maxFps(), a.maxFps()));
        return result;
    }

    /** Prefer 720p for the default (thermal/bandwidth), else the best fit. */
    public static int defaultIndex(List<Mode> modes) {
        for (int i = 0; i < modes.size(); i++)
            if (modes.get(i).width() == 1280 && modes.get(i).height() == 720) return i;
        for (int i = modes.size() - 1; i >= 0; i--)
            if (modes.get(i).pixels() <= 1280 * 720) return i;
        return modes.isEmpty() ? -1 : 0;
    }

    public static Mode bestResolution(List<Mode> modes) {
        return modes.isEmpty() ? null : modes.get(0);
    }

    public static Mode bestFps(List<Mode> modes) {
        Mode best = null;
        for (Mode mode : modes)
            if (best == null || mode.maxFps() > best.maxFps()
                    || (mode.maxFps() == best.maxFps() && mode.pixels() > best.pixels())) best = mode;
        return best;
    }

    /** JSON served at /capabilities so the desktop can mirror real support. */
    public static String capabilitiesJson(String facing, List<Mode> modes, Mode selected) {
        StringBuilder json = new StringBuilder();
        json.append("{\"version\":1,\"facing\":\"").append(facing).append("\",\"modes\":[");
        for (int i = 0; i < modes.size(); i++) {
            Mode mode = modes.get(i);
            if (i > 0) json.append(',');
            json.append(String.format(Locale.ROOT, "{\"width\":%d,\"height\":%d,\"max_fps\":%d}",
                    mode.width(), mode.height(), mode.maxFps()));
        }
        json.append(']');
        if (selected != null) json.append(String.format(Locale.ROOT,
                ",\"selected\":{\"width\":%d,\"height\":%d,\"max_fps\":%d}",
                selected.width(), selected.height(), selected.maxFps()));
        return json.append('}').toString();
    }
}
