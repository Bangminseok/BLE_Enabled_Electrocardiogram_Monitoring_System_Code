package com.example.nrf52840.ui;

import android.widget.RadioGroup;

import com.example.nrf52840.R;

public class UIModeManager {

    private final RadioGroup modeGroup;

    public enum Mode {
        RAW,
        FILTERED
    }

    public UIModeManager(RadioGroup modeGroup) {
        this.modeGroup = modeGroup;
    }

    public Mode getCurrentMode() {
        int checkedId = modeGroup.getCheckedRadioButtonId();
        if (checkedId == R.id.raw_radio) {
            return Mode.RAW;
        } else if (checkedId == R.id.filtered_radio) {
            return Mode.FILTERED;
        }
        return Mode.RAW;  // 기본값
    }
}
