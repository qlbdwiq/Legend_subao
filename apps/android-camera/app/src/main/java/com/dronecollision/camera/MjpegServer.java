package com.dronecollision.camera;

import java.io.*;
import java.net.*;
import java.nio.charset.StandardCharsets;
import java.security.*;
import java.util.*;
import java.util.concurrent.*;

/** Small authenticated HTTP MJPEG server, shared by Wi-Fi and USB port forwarding. */
public final class MjpegServer implements AutoCloseable {
    private final FrameHub frames;
    private final ServerSocket listener;
    private final Set<Socket> clients = ConcurrentHashMap.newKeySet();
    private final ExecutorService workers = Executors.newFixedThreadPool(2);
    private final Semaphore slots = new Semaphore(2);
    private final String token;
    private final String capabilitiesJson;
    private volatile boolean running = true;
    private final Thread acceptor;

    public MjpegServer(FrameHub frames) throws IOException {
        this(frames, 0, newToken());
    }
    public static String newToken() {
        byte[] random = new byte[16]; new SecureRandom().nextBytes(random);
        StringBuilder value = new StringBuilder();
        for (byte b : random) value.append(String.format(Locale.ROOT, "%02x", b & 255));
        return value.toString();
    }
    public MjpegServer(FrameHub frames, int port, String sessionToken) throws IOException {
        this(frames, port, sessionToken, null);
    }
    public MjpegServer(FrameHub frames, int port, String sessionToken, String capabilities) throws IOException {
        this.frames = frames;
        this.capabilitiesJson = capabilities;
        token = sessionToken;
        listener = new ServerSocket(); listener.setReuseAddress(true);
        listener.bind(new InetSocketAddress(InetAddress.getByName("0.0.0.0"), port), 4);
        acceptor = new Thread(this::accept, "camera-http");
        acceptor.setDaemon(true);
        acceptor.start();
    }
    public int port() { return listener.getLocalPort(); }
    public String url(String address) { return "http://" + address + ":" + port() + "/stream.mjpg?token=" + token; }
    private void accept() {
        while (running) {
            try {
                Socket socket = listener.accept();
                if (!slots.tryAcquire()) { socket.close(); continue; }
                clients.add(socket);
                try { workers.execute(() -> serve(socket)); }
                catch (RejectedExecutionException e) { clients.remove(socket); slots.release(); socket.close(); }
            } catch (IOException e) { if (running) close(); }
        }
    }
    private void serve(Socket socket) {
        try (socket) {
            socket.setSoTimeout(3000);
            socket.setTcpNoDelay(true);
            ByteArrayOutputStream header = new ByteArrayOutputStream();
            InputStream input = socket.getInputStream();
            int last = 0;
            while (header.size() < 8192) {
                int b = input.read();
                if (b < 0) return;
                header.write(b);
                last = (last << 8) | b;
                if (last == 0x0d0a0d0a) break;
            }
            String[] request = header.toString(StandardCharsets.US_ASCII.name()).split("\r\n", 2)[0].split(" ");
            OutputStream output = socket.getOutputStream();
            if (last != 0x0d0a0d0a || request.length != 3 || !request[0].equals("GET")) {
                reply(output, "400 Bad Request"); return;
            }
            String target = request[1];
            if (target.startsWith("/capabilities")) {
                String expectedCaps = "/capabilities?token=" + token;
                if (capabilitiesJson == null || !MessageDigest.isEqual(
                        target.getBytes(StandardCharsets.US_ASCII), expectedCaps.getBytes(StandardCharsets.US_ASCII))) {
                    reply(output, "403 Forbidden"); return;
                }
                byte[] body = capabilitiesJson.getBytes(StandardCharsets.UTF_8);
                output.write(("HTTP/1.1 200 OK\r\nContent-Type: application/json\r\nContent-Length: "
                        + body.length + "\r\nConnection: close\r\n\r\n").getBytes(StandardCharsets.US_ASCII));
                output.write(body); output.flush();
                return;
            }
            String expected = "/stream.mjpg?token=" + token;
            if (!MessageDigest.isEqual(target.getBytes(StandardCharsets.US_ASCII), expected.getBytes(StandardCharsets.US_ASCII))) {
                reply(output, "403 Forbidden"); return;
            }
            output.write(("HTTP/1.1 200 OK\r\nContent-Type: multipart/x-mixed-replace; boundary=droneframe\r\n" +
                    "Cache-Control: no-store\r\nConnection: close\r\n\r\n").getBytes(StandardCharsets.US_ASCII));
            long sequence = 0;
            while (running) {
                FrameHub.Frame frame = frames.after(sequence);
                if (frame == null) continue;
                output.write(("--droneframe\r\nContent-Type: image/jpeg\r\nContent-Length: " + frame.jpeg().length +
                        "\r\nX-Frame-Sequence: " + frame.sequence() + "\r\n\r\n").getBytes(StandardCharsets.US_ASCII));
                output.write(frame.jpeg());
                output.write("\r\n".getBytes(StandardCharsets.US_ASCII));
                output.flush();
                sequence = frame.sequence();
            }
        } catch (IOException | InterruptedException e) {
            if (e instanceof InterruptedException) Thread.currentThread().interrupt();
        } finally { clients.remove(socket); slots.release(); }
    }
    private static void reply(OutputStream out, String code) throws IOException {
        out.write(("HTTP/1.1 " + code + "\r\nContent-Length: 0\r\nConnection: close\r\n\r\n").getBytes(StandardCharsets.US_ASCII));
    }
    @Override public void close() {
        running = false;
        try { listener.close(); } catch (IOException ignored) { }
        for (Socket socket : clients) { try { socket.close(); } catch (IOException ignored) { } }
        workers.shutdownNow();
    }
}
