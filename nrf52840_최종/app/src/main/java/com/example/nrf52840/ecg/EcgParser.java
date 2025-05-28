package com.example.nrf52840.ecg;

import java.nio.ByteBuffer;
import java.nio.ByteOrder;

public class EcgParser {
    public static float[] parseToMv(byte[] data) {
        int len = data.length / 2;
        float[] result = new float[len];
        for (int i = 0; i < data.length - 1; i += 2) {
            short adc = ByteBuffer.wrap(data, i, 2)
                    .order(ByteOrder.LITTLE_ENDIAN)
                    .getShort();
            result[i / 2] = (adc / 4096.0f) * 3300f;
        }
        return result;
    }
}
