package com.example.nrf52840.ecg;

public class EcgLowPassFilter {
    private final double[] firCoefficients = {
            -0.001594, -0.000185, 0.002569, 0.001121,
            -0.005270, -0.003886, 0.009647, 0.010118,
            -0.015216, -0.022170, 0.021136, 0.044734,
            -0.026390, -0.093805, 0.030008, 0.314452,
            0.469459,
            0.314452, 0.030008, -0.093805, -0.026390,
            0.044734, 0.021136, -0.022170, -0.015216,
            0.010118, 0.009647, -0.003886, -0.005270,
            0.001121, 0.002569, -0.000185, -0.001594
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
