package com.example.nrf52840.ble;

import android.bluetooth.*;
import android.content.Context;
import android.os.Handler;
import android.os.Looper;
import android.util.Log;
import android.content.SharedPreferences;

import com.example.nrf52840.ecg.EcgParser;
import com.example.nrf52840.util.PermissionUtils;

import org.json.JSONArray;
import org.json.JSONException;
import org.json.JSONObject;

import java.util.Arrays;
import java.util.UUID;

import okhttp3.Call;
import okhttp3.Callback;
import okhttp3.MediaType;
import okhttp3.OkHttpClient;
import okhttp3.Request;
import okhttp3.RequestBody;
import okhttp3.Response;

import java.io.IOException;
import java.text.SimpleDateFormat;
import java.util.Date;


public class BleCallbackHandler extends BluetoothGattCallback {

    private static final String TAG = "BleCallbackHandler";

    private final Context context;
    private final Handler uiHandler = new Handler(Looper.getMainLooper());
    private final BleCallbackListener listener;

    private int ecgSampleCount = 0;
    private final Handler countHandler = new Handler(Looper.getMainLooper());
    private static boolean savingEnabled = false;

    public static void setSavingEnabled(boolean enabled) {
        savingEnabled = enabled;
    }

    public interface BleCallbackListener {
        void onEcgReceived(float[] values);
        void onBpmReceived(int bpm);
        void onBatteryReceived(int level);
        void onDisconnected();
    }

    public BleCallbackHandler(Context context, BleCallbackListener listener) {
        this.context = context.getApplicationContext();
        this.listener = listener;
        startEcgCountLogger();
    }

    private void startEcgCountLogger() {
        countHandler.postDelayed(new Runnable() {
            @Override
            public void run() {
                Log.i("BLE_ECG", "Samples/sec: " + ecgSampleCount);
                ecgSampleCount = 0;
                countHandler.postDelayed(this, 1000);
            }
        }, 1000);
    }

    @Override
    public void onConnectionStateChange(BluetoothGatt gatt, int status, int newState) {
        Log.d(TAG, "onConnectionStateChange: newState = " + newState);
        if (newState == BluetoothProfile.STATE_CONNECTED) {
            try {
                Log.d(TAG, "BLE 연결됨 → 서비스 검색 시작");
                gatt.discoverServices();
            } catch (SecurityException e) {
                Log.e(TAG, "discoverServices() 실패", e);
            }
        } else if (newState == BluetoothProfile.STATE_DISCONNECTED) {
            Log.w(TAG, "BLE 연결 끊김");
            uiHandler.post(listener::onDisconnected);
        }
    }

    @Override
    public void onServicesDiscovered(BluetoothGatt gatt, int status) {
        if (status != BluetoothGatt.GATT_SUCCESS) {
            Log.e(TAG, "서비스 검색 실패: " + status);
            return;
        }

        try {
            BluetoothGattService service = gatt.getService(BleConstants.SERVICE_UUID);
            if (service == null) {
                Log.e(TAG, "서비스 UUID 찾을 수 없음");
                return;
            }

            BluetoothGattCharacteristic ecgChar = service.getCharacteristic(BleConstants.ECG_CHAR_UUID);
            BluetoothGattCharacteristic bpmChar = service.getCharacteristic(BleConstants.BPM_CHAR_UUID);
            BluetoothGattCharacteristic batteryChar = service.getCharacteristic(BleConstants.BATTERY_CHAR_UUID);

            enableNotify(gatt, ecgChar);  // 즉시
            new Handler(Looper.getMainLooper()).postDelayed(() ->
                    enableNotify(gatt, bpmChar), 300);
            new Handler(Looper.getMainLooper()).postDelayed(() ->
                    enableNotify(gatt, batteryChar), 600);

        } catch (SecurityException e) {
            Log.e(TAG, "onServicesDiscovered 예외", e);
        }
    }

    private void enableNotify(BluetoothGatt gatt, BluetoothGattCharacteristic c) {
        if (c == null) {
            Log.w("BLE_NOTIFY", "특성 null, notify 설정 생략");
            return;
        }

        try {
            Log.d("BLE_NOTIFY", "Notify 설정 시작: " + c.getUuid());
            gatt.setCharacteristicNotification(c, true);
            BluetoothGattDescriptor d = c.getDescriptor(BleConstants.CCCD_UUID);
            if (d != null) {
                d.setValue(BluetoothGattDescriptor.ENABLE_NOTIFICATION_VALUE);
                boolean result = gatt.writeDescriptor(d);
                Log.d("BLE_NOTIFY", "Descriptor 쓰기 결과: " + result);
            } else {
                Log.e("BLE_NOTIFY", "Descriptor가 null임");
            }
        } catch (SecurityException e) {
            Log.e("BLE_NOTIFY", "notify 설정 중 SecurityException 발생", e);
        }
    }

    @Override
    public void onCharacteristicChanged(BluetoothGatt gatt, BluetoothGattCharacteristic c) {
        if (!PermissionUtils.hasBluetoothPermission(context)) return;

        UUID uuid = c.getUuid();
        byte[] value = c.getValue();

        if (BleConstants.ECG_CHAR_UUID.equals(uuid)) {
            float[] mv = EcgParser.parseToMv(value);
            Log.i("test", "test" + Arrays.toString(mv));

            // 저장 중일 때만 서버 전송
            if (savingEnabled) {
                try {
                    sendEcgDataToServer(mv);
                } catch (JSONException e) {
                    throw new RuntimeException(e);
                }
            }

            ecgSampleCount += mv.length;
            uiHandler.post(() -> listener.onEcgReceived(mv));
        }
        else if (BleConstants.BPM_CHAR_UUID.equals(uuid) && value.length >= 2) {
            int bpm = (value[0] & 0xFF) | ((value[1] & 0xFF) << 8);
            Log.i("BLE_BPM", "❤️ BPM: " + bpm);
            uiHandler.post(() -> listener.onBpmReceived(bpm));

        } else if (BleConstants.BATTERY_CHAR_UUID.equals(uuid) && value.length == 1) {
            int battery = value[0] & 0xFF;
            Log.i("BLE_BATTERY", "🔋 배터리 단계: " + battery);
            uiHandler.post(() -> listener.onBatteryReceived(battery));
        }
    }
    // NodeJS 백엔드 데이터베이스 저장 함수
    private void sendEcgDataToServer(float[] mv) throws JSONException {
        String timestamp = new SimpleDateFormat("yyyy-MM-dd HH:mm:ss").format(new Date());

        // 1️⃣ mv[] 배열을 JSON 배열 형태로 변환
        JSONArray mvArray = new JSONArray();
        for (float value : mv) {
            mvArray.put(value);
        }

        // 2️⃣ 전체 JSON 객체 생성
        JSONObject jsonObject = new JSONObject();
        try {
            jsonObject.put("row_data", mvArray);
            jsonObject.put("sensor_measure_time", timestamp);
        } catch (JSONException e) {
            e.printStackTrace();
            return;
        }

        // 3️⃣ JSON 문자열로 전송
        OkHttpClient client = new OkHttpClient();
        RequestBody body = RequestBody.create(
                jsonObject.toString(), MediaType.get("application/json; charset=utf-8")
        );

        SharedPreferences prefs = context.getSharedPreferences("config", Context.MODE_PRIVATE);
        String serverUrl = prefs.getString("server_url", "http://192.168.0.10:3000/ecg/store"); // 기본값 설정

        Request request = new Request.Builder()
                .url(serverUrl)
                .post(body)
                .build();


        client.newCall(request).enqueue(new Callback() {
            @Override
            public void onFailure(Call call, IOException e) {
                Log.e("ECG_API", "전송 실패", e);
            }

            @Override
            public void onResponse(Call call, Response response) throws IOException {
                if (response.isSuccessful()) {
                    Log.i("ECG_API", "전송 성공: " + response.body().string());
                } else {
                    Log.w("ECG_API", "응답 오류: " + response.code());
                }
            }
        });
    }

//    private void sendEcgDataToServer(float[] mv) {
//        float firstSample = mv[0];
//        int bpm = 88; // 예시값
//        String timestamp = new SimpleDateFormat("yyyy-MM-dd HH:mm:ss").format(new Date());
//
//        String json = String.format(
//                "{\"row_data\": %.3f, \"bpm_data\": %d, \"sensor_measure_time\": \"%s\"}",
//                firstSample, bpm, timestamp
//        );
//
//        OkHttpClient client = new OkHttpClient();
//        RequestBody body = RequestBody.create(
//                json, MediaType.get("application/json; charset=utf-8")
//        );
//
//        Request request = new Request.Builder()
//                .url("http://192.168.219.110:3000/ecg/store")
//                .post(body)
//                .build();
//
//        client.newCall(request).enqueue(new Callback() {
//            @Override
//            public void onFailure(Call call, IOException e) {
//                Log.e("ECG_API", "전송 실패", e);
//            }
//
//            @Override
//            public void onResponse(Call call, Response response) throws IOException {
//                if (response.isSuccessful()) {
//                    Log.i("ECG_API", "전송 성공: " + response.body().string());
//                } else {
//                    Log.w("ECG_API", "응답 오류: " + response.code());
//                }
//            }
//        });
//    }


}
