package com.example.nrf52840.ui;

import android.graphics.Color;
import android.view.View;

public class BatteryViewUpdater {

    private final View[] batteryBars;

    public BatteryViewUpdater(View[] batteryBars) {
        this.batteryBars = batteryBars;
    }

    public void update(int step) {
        for (int i = 0; i < batteryBars.length; i++) {
            if (i < step) {
                batteryBars[i].setBackgroundColor(step == 1 && i == 0 ? Color.RED : Color.GREEN);
            } else {
                batteryBars[i].setBackgroundColor(Color.LTGRAY);
            }
        }
    }
}
