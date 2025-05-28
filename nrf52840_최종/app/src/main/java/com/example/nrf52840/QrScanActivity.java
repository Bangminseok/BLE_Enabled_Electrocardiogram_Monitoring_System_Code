// QrScanActivity.java
package com.example.nrf52840;
import android.app.Activity;
import android.os.Bundle;
import androidx.appcompat.app.AppCompatActivity;

import com.journeyapps.barcodescanner.CaptureActivity;
import com.journeyapps.barcodescanner.ScanContract;
import com.journeyapps.barcodescanner.ScanOptions;

public class QrScanActivity extends AppCompatActivity {
    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        startQrScanner();
    }

    private void startQrScanner() {
        ScanOptions options = new ScanOptions();
        options.setPrompt("서버 주소가 포함된 QR코드를 스캔하세요");
        options.setBeepEnabled(true);
        options.setOrientationLocked(false);
        options.setCaptureActivity(CaptureActivity.class);
        barcodeLauncher.launch(options);
    }

    private final androidx.activity.result.ActivityResultLauncher<ScanOptions> barcodeLauncher =
            registerForActivityResult(new ScanContract(), result -> {
                if (result.getContents() != null) {
                    String scannedUrl = result.getContents();

                    // SharedPreferences에 저장
                    getSharedPreferences("config", MODE_PRIVATE)
                            .edit()
                            .putString("server_url", scannedUrl)
                            .apply();
                }
                finish(); // 스캔 끝나면 종료
            });
}
