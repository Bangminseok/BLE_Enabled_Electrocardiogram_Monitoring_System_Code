package com.example.nrf52840.ble;

import java.util.UUID;

public class BleConstants {
    public static final UUID SERVICE_UUID = UUID.fromString("6e400001-b5a3-f393-e0a9-e50e24dcca9e");
    public static final UUID ECG_CHAR_UUID = UUID.fromString("6e400003-b5a3-f393-e0a9-e50e24dcca9e");
    public static final UUID BPM_CHAR_UUID = UUID.fromString("6e400004-b5a3-f393-e0a9-e50e24dcca9e");
    public static final UUID BATTERY_CHAR_UUID = UUID.fromString("6e400005-b5a3-f393-e0a9-e50e24dcca9e");
    public static final UUID CCCD_UUID = UUID.fromString("00002902-0000-1000-8000-00805f9b34fb");
}
