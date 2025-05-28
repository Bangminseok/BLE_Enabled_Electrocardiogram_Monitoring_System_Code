package com.example.nrf52840.battery;

import android.graphics.Color;

public class BatteryUtils {

    /**
     * 배터리 단계(1~5)에 따라 색상을 반환
     * 1단계는 빨강, 2~5단계는 초록, 0 또는 이상 값은 회색
     */
    public static int getColorForLevel(int step) {
        if (step <= 0) return Color.LTGRAY;
        if (step == 1) return Color.RED;
        if (step <= 5) return Color.GREEN;
        return Color.LTGRAY;
    }

    /**
     * 배터리 단계 유효성 검사
     */
    public static boolean isValidStep(int step) {
        return step >= 1 && step <= 5;
    }
}
