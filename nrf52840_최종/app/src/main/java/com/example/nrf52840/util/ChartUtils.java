package com.example.nrf52840.util;

import android.app.Activity;
import android.content.ContentResolver;
import android.content.ContentValues;
import android.graphics.Bitmap;
import android.net.Uri;
import android.os.Environment;
import android.provider.MediaStore;
import android.view.View;

import com.example.nrf52840.GraphActivity; // ✅ 추가

import java.io.OutputStream;
import java.io.IOException;

public class ChartUtils {

    /**
     * 현재 전체 화면을 캡처하여 갤러리에 저장
     */
    public static void captureFullScreen(Activity activity, String title) {
        View rootView = activity.getWindow().getDecorView().getRootView();
        rootView.setDrawingCacheEnabled(true);
        Bitmap bitmap = Bitmap.createBitmap(rootView.getDrawingCache());
        rootView.setDrawingCacheEnabled(false);

        String fileName = title + "_" + System.currentTimeMillis() + ".jpg";

        ContentValues values = new ContentValues();
        values.put(MediaStore.Images.Media.DISPLAY_NAME, fileName);
        values.put(MediaStore.Images.Media.MIME_TYPE, "image/jpeg");
        values.put(MediaStore.Images.Media.RELATIVE_PATH, Environment.DIRECTORY_PICTURES);

        try {
            ContentResolver resolver = activity.getContentResolver();
            Uri uri = resolver.insert(MediaStore.Images.Media.EXTERNAL_CONTENT_URI, values);
            if (uri != null) {
                try (OutputStream out = resolver.openOutputStream(uri)) {
                    bitmap.compress(Bitmap.CompressFormat.JPEG, 100, out);
                }
                // ✅ GraphActivity의 static 메서드로 Toast 처리
                GraphActivity.showToast(activity, "전체 화면 캡처 완료");
            } else {
                GraphActivity.showToast(activity, "갤러리에 저장 실패");
            }
        } catch (IOException e) {
            e.printStackTrace();
            GraphActivity.showToast(activity, "캡처 실패: " + e.getMessage());
        }
    }
}
