package com.example.nrf52840.ecg;

public class EcgHighPassFilter {
    private final double[] firCoefficients = {
            -0.000609, -0.000679, -0.000882, -0.001211,
            -0.001654, -0.002197, -0.002817, -0.003491,
            -0.004194, -0.004899, -0.005577, -0.006204,
            -0.006753, -0.007205, -0.007541, -0.007747,
            0.972791,
            -0.007747, -0.007541, -0.007205, -0.006753,
            -0.006204, -0.005577, -0.004899, -0.004194,
            -0.003491, -0.002817, -0.002197, -0.001654,
            -0.001211, -0.000882, -0.000679, -0.000609
    };

    private final double[] buffer = new double[firCoefficients.length];
    private int offset = 0;

    public float filter(float input) {
        buffer[offset] = input;
        double output = 0.0;
        int idx = offset;

        for (int i = 0; i < firCoefficients.length; i++) {
            output += firCoefficients[i] * buffer[idx];
            idx = (idx - 1 + firCoefficients.length) % firCoefficients.length;
        }

        offset = (offset + 1) % firCoefficients.length;
        return (float) output;
    }
}
