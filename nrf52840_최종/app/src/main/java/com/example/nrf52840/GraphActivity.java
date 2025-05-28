package com.example.nrf52840;

import android.Manifest;
import android.bluetooth.BluetoothDevice;
import android.content.Context;
import android.content.Intent;
import android.content.pm.PackageManager;
import android.os.*;
import android.view.View;
import android.widget.*;

import androidx.annotation.NonNull;
import androidx.appcompat.app.AppCompatActivity;
import androidx.core.app.ActivityCompat;

import com.example.nrf52840.ble.*;
import com.example.nrf52840.ecg.*;
import com.example.nrf52840.ui.*;
import com.example.nrf52840.util.*;

import com.github.mikephil.charting.charts.LineChart;
import com.github.mikephil.charting.data.*;

public class GraphActivity extends AppCompatActivity implements BleCallbackHandler.BleCallbackListener {

    private static final int REQ_BT = 1001;

    private static Toast currentToast; // ✅ 모든 곳에서 공유하는 Toast
    public static void showToast(Context context, String message) {
        if (currentToast != null) {
            currentToast.cancel();
        }
        currentToast = Toast.makeText(context, message, Toast.LENGTH_SHORT);
        currentToast.show();
    }

    private Button saveButton;
    private boolean isSaving = false;

    private LineChart chart;
    private TextView bpmText;
    private RadioGroup modeGroup;
    private RadioButton rawRadio, filteredRadio;
    private View[] batteryBars;

    private LineDataSet rawSet, filteredSet;
    private EcgBuffer ecgBuffer;

    private float xValue = 0f;

    private EcgLowPassFilter lowPassFilter;
    private EcgHighPassFilter highPassFilter;

    private ChartController chartController;
    private BatteryViewUpdater batteryViewUpdater;
    private UIModeManager uiModeManager;
    private BleManager bleManager;

    private final Handler graphHandler = new Handler(Looper.getMainLooper());

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_graph);

        initViews();
        initChart();
        initHelpers();

        graphHandler.post(graphUpdater);
        requestBluetoothPermission();
    }

    private void initViews() {
        chart = findViewById(R.id.Chart);
        bpmText = findViewById(R.id.bpm_text);
        modeGroup = findViewById(R.id.mode_group);
        rawRadio = findViewById(R.id.raw_radio);
        filteredRadio = findViewById(R.id.filtered_radio);

        batteryBars = new View[]{
                findViewById(R.id.battery_bar_1),
                findViewById(R.id.battery_bar_2),
                findViewById(R.id.battery_bar_3),
                findViewById(R.id.battery_bar_4),
                findViewById(R.id.battery_bar_5)
        };

        saveButton = findViewById(R.id.save_button);
        saveButton.setOnClickListener(v -> toggleDataSaving());

        Button pause = findViewById(R.id.pause_button);
        Button resume = findViewById(R.id.resume_button);
        ImageButton capture = findViewById(R.id.capture_button);
        Button home = findViewById(R.id.home_button);

        pause.setOnClickListener(v -> graphHandler.removeCallbacks(graphUpdater));
        resume.setOnClickListener(v -> graphHandler.post(graphUpdater));
        capture.setOnClickListener(v -> ChartUtils.captureFullScreen(this, "ecg_full_capture"));
        home.setOnClickListener(v -> {
            bleManager.disconnect();
            startActivity(new Intent(this, MainActivity.class)
                    .setFlags(Intent.FLAG_ACTIVITY_CLEAR_TOP | Intent.FLAG_ACTIVITY_NEW_TASK));
            finish();
        });
    }

    private void toggleDataSaving() {
        isSaving = !isSaving;
        BleCallbackHandler.setSavingEnabled(isSaving);

        if (isSaving) {
            saveButton.setText("저장 중지");
            saveButton.setBackgroundColor(getResources().getColor(android.R.color.holo_green_dark));
            showToast(this, "데이터 저장 시작");
        } else {
            saveButton.setText("데이터 저장");
            saveButton.setBackgroundResource(android.R.drawable.btn_default);
            showToast(this, "데이터 저장 중지");
            saveButton.setPressed(false);
            saveButton.invalidate();
        }
    }

    private void initChart() {
        rawSet = new LineDataSet(null, "Raw ECG");
        rawSet.setDrawCircles(false);
        rawSet.setColor(android.graphics.Color.RED);

        filteredSet = new LineDataSet(null, "Filtered ECG");
        filteredSet.setDrawCircles(false);
        filteredSet.setColor(android.graphics.Color.BLUE);

        chart.setData(new LineData());
        chart.getDescription().setEnabled(false);
        chart.getAxisRight().setEnabled(false);
        chart.setAutoScaleMinMaxEnabled(true);
    }

    private void initHelpers() {
        ecgBuffer = new EcgBuffer();
        lowPassFilter = new EcgLowPassFilter();
        highPassFilter = new EcgHighPassFilter();
        chartController = new ChartController(chart, rawSet, filteredSet);
        batteryViewUpdater = new BatteryViewUpdater(batteryBars);
        uiModeManager = new UIModeManager(modeGroup);
    }

    private void requestBluetoothPermission() {
        if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.S &&
                !PermissionUtils.hasBluetoothPermission(this)) {
            ActivityCompat.requestPermissions(this,
                    new String[]{Manifest.permission.BLUETOOTH_CONNECT}, REQ_BT);
        } else {
            connectToBleDevice();
        }
    }

    private void connectToBleDevice() {
        BluetoothDevice device = getIntent().getParcelableExtra("device");
        if (device == null) return;

        bleManager = new BleManager();
        bleManager.connect(this, device, new BleCallbackHandler(this, this));
    }

    private final Runnable graphUpdater = new Runnable() {
        @Override
        public void run() {
            LineData data = chart.getData();
            data.clearValues();

            int mode = modeGroup.getCheckedRadioButtonId();
            boolean updated = false;

            if (mode == R.id.raw_radio) {
                Float rawVal = ecgBuffer.pollRaw();
                if (rawVal != null) {
                    rawSet.addEntry(new Entry(xValue, rawVal));
                    data.addDataSet(rawSet);
                    updated = true;
                }

            } else if (mode == R.id.filtered_radio) {
                Float filteredVal = ecgBuffer.pollFiltered();
                if (filteredVal != null) {
                    filteredSet.addEntry(new Entry(xValue, filteredVal));
                    data.addDataSet(filteredSet);
                    updated = true;
                }

            } else if (mode == R.id.both_radio) {
                Float rawVal = ecgBuffer.pollRaw();
                Float filteredVal = ecgBuffer.pollFiltered();
                if (rawVal != null) rawSet.addEntry(new Entry(xValue, rawVal));
                if (filteredVal != null) filteredSet.addEntry(new Entry(xValue, filteredVal));
                data.addDataSet(rawSet);
                data.addDataSet(filteredSet);
                updated = true;
            }

            chart.getAxisLeft().resetAxisMinimum();
            chart.getAxisLeft().resetAxisMaximum();

            if (updated) {
                data.notifyDataChanged();
                chart.notifyDataSetChanged();
                chart.setVisibleXRangeMaximum(5);
                chart.moveViewToX(xValue);
            }

            xValue += 0.016f;
            graphHandler.postDelayed(this, 10);
        }
    };

    @Override
    public void onEcgReceived(float[] values) {
        for (float mv : values) {
            float centered = mv - 1650f; //	DC 성분 제거 → 필터 성능 안정화
            float lowPassed = lowPassFilter.filter(centered);
            float bandPassed = highPassFilter.filter(lowPassed);
            float mvFiltered = bandPassed + 1650f; //필터 후 신호를 다시 시각화 기준 위치로 복원

            ecgBuffer.addRaw(mv);
            ecgBuffer.addFiltered(mvFiltered);
        }
    }

    @Override
    public void onBpmReceived(int bpm) {
        runOnUiThread(() -> bpmText.setText("BPM: " + bpm));
    }

    @Override
    public void onBatteryReceived(int level) {
        runOnUiThread(() -> batteryViewUpdater.update(level));
    }

    @Override
    public void onDisconnected() {
        runOnUiThread(() ->
                showToast(this, "BLE 연결 끊김"));
    }

    @Override
    protected void onDestroy() {
        super.onDestroy();
        graphHandler.removeCallbacksAndMessages(null);
        if (bleManager != null) bleManager.disconnect();
    }

    @Override
    public void onRequestPermissionsResult(int requestCode, @NonNull String[] permissions, @NonNull int[] grantResults) {
        super.onRequestPermissionsResult(requestCode, permissions, grantResults);
        if (requestCode == REQ_BT &&
                grantResults.length > 0 &&
                grantResults[0] == PackageManager.PERMISSION_GRANTED) {
            connectToBleDevice();
        }
    }

    @Override
    public void onBackPressed() {
        showToast(this, "뒤로가기는 비활성화되어 있습니다.");
    }
}
