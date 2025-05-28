package com.example.nrf52840.ble;

import android.Manifest;
import android.bluetooth.*;
import android.content.Context;
import android.content.pm.PackageManager;
import android.os.Build;
import android.util.Log;

import androidx.core.app.ActivityCompat;

public class BleManager {

    private static final String TAG = "BleManager";
    private BluetoothGatt bluetoothGatt;

    public interface BleConnectionCallback {
        void onGattReady(BluetoothGatt gatt);
    }

    /**
     * BLE 장치와 연결을 시도합니다.
     * Android 12 이상에서는 BLUETOOTH_CONNECT 권한 확인 필수
     */
    public void connect(Context context, BluetoothDevice device, BluetoothGattCallback callback) {
        if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.S) {
            if (ActivityCompat.checkSelfPermission(context, Manifest.permission.BLUETOOTH_CONNECT)
                    != PackageManager.PERMISSION_GRANTED) {
                Log.e(TAG, "BLUETOOTH_CONNECT permission not granted");
                return;
            }
        }

        try {
            bluetoothGatt = device.connectGatt(context, false, callback);
        } catch (SecurityException e) {
            Log.e(TAG, "connectGatt() failed with SecurityException", e);
        }
    }

    /**
     * GATT 연결을 안전하게 해제
     */
    public void disconnect() {
        if (bluetoothGatt != null) {
            try {
                bluetoothGatt.disconnect();
                bluetoothGatt.close();
            } catch (SecurityException e) {
                Log.e(TAG, "disconnect() failed with SecurityException", e);
            }
            bluetoothGatt = null;
        }
    }

    public BluetoothGatt getGatt() {
        return bluetoothGatt;
    }
}
