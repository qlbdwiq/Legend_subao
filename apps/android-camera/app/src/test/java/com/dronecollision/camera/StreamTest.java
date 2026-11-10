package com.dronecollision.camera;

import org.junit.Test;
import static org.junit.Assert.*;
import java.net.*;
import java.io.*;
import java.nio.ByteBuffer;
import java.nio.charset.StandardCharsets;

public class StreamTest {
    @Test public void stopAndRestartRetainsTheCopiedUrl() throws Exception {
        String token = MjpegServer.newToken();
        int port;
        String url;
        try (MjpegServer first = new MjpegServer(new FrameHub(),0,token)) {
            port = first.port(); url = first.url("127.0.0.1");
        }
        try (MjpegServer second = new MjpegServer(new FrameHub(),port,token)) {
            assertEquals(url, second.url("127.0.0.1"));
        }
    }
    @Test public void latestFrameSkipsOldPackets() throws Exception {
        FrameHub frames = new FrameHub();
        byte[] input = {1,2,3}; frames.publish(input); input[0] = 99;
        assertEquals(1, frames.latest().jpeg()[0]);
        frames.publish(new byte[]{4,5});
        assertEquals(2, frames.after(0).sequence());
        frames.close(); assertNull(frames.after(2));
    }
    @Test public void arbitraryYuvStridesProduceNv21() {
        byte[] result = Yuv420.nv21(4,2,
                new ByteBuffer[]{ByteBuffer.wrap(new byte[]{1,2,3,4,0,0,5,6,7,8}),
                    ByteBuffer.wrap(new byte[]{9,0,10}), ByteBuffer.wrap(new byte[]{11,0,12})},
                new int[]{6,4,4},new int[]{1,2,2});
        assertArrayEquals(new byte[]{1,2,3,4,5,6,7,8,11,9,12,10}, result);
    }
    @Test public void realHttpAuthenticationAndMultipart() throws Exception {
        FrameHub frames = new FrameHub();
        try (MjpegServer server = new MjpegServer(frames)) {
            HttpURLConnection denied = (HttpURLConnection)new URL("http://127.0.0.1:"+server.port()+"/stream.mjpg?token=wrong").openConnection();
            denied.setReadTimeout(3000); assertEquals(403, denied.getResponseCode()); denied.disconnect();
            frames.publish(new byte[]{1,2,3});
            HttpURLConnection allowed = (HttpURLConnection)new URL(server.url("127.0.0.1")).openConnection();
            allowed.setReadTimeout(3000); assertEquals(200, allowed.getResponseCode());
            assertTrue(allowed.getContentType().contains("multipart/x-mixed-replace"));
            BufferedReader reader = new BufferedReader(new InputStreamReader(allowed.getInputStream(),StandardCharsets.US_ASCII));
            assertEquals("--droneframe", reader.readLine());
            assertEquals("Content-Type: image/jpeg", reader.readLine());
            assertEquals("Content-Length: 3", reader.readLine());
            allowed.disconnect();
        } finally { frames.close(); }
    }
}
