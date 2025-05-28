package com.example.nrf52840;

import android.Manifest;
import android.bluetooth.BluetoothAdapter;
import android.content.Intent;
import android.content.pm.PackageManager;
import android.os.Build;
import android.os.Bundle;
import android.widget.TextView;
import android.widget.ImageButton;
import android.view.animation.Animation;
import android.view.animation.AnimationUtils;

import androidx.appcompat.app.AppCompatActivity;
import androidx.core.app.ActivityCompat;
import android.widget.Button;

public class MainActivity extends AppCompatActivity {
    private BluetoothAdapter bluetoothAdapter;
    private TextView statusText, connectingText;
    private static final String DEVICE_NAME = "SPAL";
    private BluetoothAdapter.LeScanCallback scanCallback;

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);

        statusText = findViewById(R.id.status_text);
        connectingText = findViewById(R.id.connecting_text);

        ImageButton bleButton = findViewById(R.id.ble_connect_button);
        Animation blink = AnimationUtils.loadAnimation(this, R.anim.blink);
        bleButton.startAnimation(blink);

        bluetoothAdapter = BluetoothAdapter.getDefaultAdapter();
        bleButton.setOnClickListener(v -> startScan());

        // ✅ QR 스캔 버튼 연결 → QrScanActivity로 이동하도록 수정
        Button qrScanButton = findViewById(R.id.qr_scan_button);
        qrScanButton.setOnClickListener(v -> {
            Intent intent = new Intent(MainActivity.this, QrScanActivity.class);
            startActivity(intent);
        });
    }

    private void startScan() {
        connectingText.setText("BLE 연결 중...");

        if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.S &&
                (ActivityCompat.checkSelfPermission(this, Manifest.permission.BLUETOOTH_SCAN) != PackageManager.PERMISSION_GRANTED ||
                        ActivityCompat.checkSelfPermission(this, Manifest.permission.BLUETOOTH_CONNECT) != PackageManager.PERMISSION_GRANTED)) {
            ActivityCompat.requestPermissions(this, new String[]{
                    Manifest.permission.BLUETOOTH_SCAN,
                    Manifest.permission.BLUETOOTH_CONNECT,
                    Manifest.permission.ACCESS_FINE_LOCATION
            }, 1);
            return;
        }

        scanCallback = (device, rssi, scanRecord) -> {
            if (device.getName() != null && device.getName().contains(DEVICE_NAME)) {
                if (ActivityCompat.checkSelfPermission(this, Manifest.permission.BLUETOOTH_SCAN) != PackageManager.PERMISSION_GRANTED)
                    return;
                bluetoothAdapter.stopLeScan(scanCallback);
                Intent intent = new Intent(this, GraphActivity.class);
                intent.putExtra("device", device);
                startActivity(intent);
            }
        };

        bluetoothAdapter.startLeScan(scanCallback);
    }

    @Override
    public void onRequestPermissionsResult(int requestCode, String[] permissions, int[] grantResults) {
        super.onRequestPermissionsResult(requestCode, permissions, grantResults);
        if (requestCode == 1) {
            boolean allGranted = true;
            for (int result : grantResults) {
                if (result != PackageManager.PERMISSION_GRANTED) {
                    allGranted = false;
                    break;
                }
            }
            if (allGranted) {
                startScan();
            } else {
                statusText.setText("BLE 권한이 필요합니다.");
            }
        }
    }
}
