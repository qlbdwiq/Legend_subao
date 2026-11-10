package com.dronecollision.camera;

import java.nio.ByteBuffer;

/** Packs arbitrary Android YUV_420_888 row/pixel strides as NV21 for JPEG encoding. */
public final class Yuv420 {
    private Yuv420() { }
    public static byte[] nv21(int width, int height, ByteBuffer[] planes, int[] rows, int[] pixels) {
        byte[] output = new byte[width * height * 3 / 2];
        int index = 0;
        for (int y = 0; y < height; y++)
            for (int x = 0; x < width; x++)
                output[index++] = planes[0].get(planes[0].position() + y * rows[0] + x * pixels[0]);
        for (int y = 0; y < height / 2; y++)
            for (int x = 0; x < width / 2; x++) {
                output[index++] = planes[2].get(planes[2].position() + y * rows[2] + x * pixels[2]);
                output[index++] = planes[1].get(planes[1].position() + y * rows[1] + x * pixels[1]);
            }
        return output;
    }
}
