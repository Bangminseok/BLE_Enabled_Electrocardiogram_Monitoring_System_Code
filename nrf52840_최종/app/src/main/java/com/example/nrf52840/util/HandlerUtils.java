package com.example.nrf52840.util;

import android.os.Handler;
import android.os.Looper;
import android.util.Log;

public class HandlerUtils {

    /**
     * ECG 샘플 수를 초당 로그로 출력하는 Runnable을 반환
     */
    public static Runnable createEcgSampleLogger(Handler handler, SampleCounter counter) {
        return new Runnable() {
            @Override
            public void run() {
                Log.i("BLE_ECG", "Samples/sec: " + counter.getCount());
                counter.reset();
                handler.postDelayed(this, 1000);
            }
        };
    }

    /**
     * 샘플 수 카운터 인터페이스
     */
    public interface SampleCounter {
        int getCount();
        void reset();
    }
}
