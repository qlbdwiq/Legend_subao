package com.dronecollision.camera;

/** Exactly one immutable JPEG packet; consumers never accumulate a frame queue. */
public final class FrameHub {
    public record Frame(long sequence, byte[] jpeg) { }
    private Frame latest;
    private boolean stopped;
    public synchronized void publish(byte[] jpeg) {
        if (!stopped) {
            latest = new Frame(latest == null ? 1 : latest.sequence() + 1, jpeg.clone());
            notifyAll();
        }
    }
    public synchronized Frame latest() { return latest; }
    public synchronized Frame after(long sequence) throws InterruptedException {
        long until = System.nanoTime() + 2_000_000_000L;
        while (!stopped && (latest == null || latest.sequence() <= sequence)) {
            long left = until - System.nanoTime();
            if (left <= 0) return null;
            wait(Math.max(1, left / 1_000_000));
        }
        return stopped ? null : latest;
    }
    public synchronized void close() { stopped = true; notifyAll(); }
}
