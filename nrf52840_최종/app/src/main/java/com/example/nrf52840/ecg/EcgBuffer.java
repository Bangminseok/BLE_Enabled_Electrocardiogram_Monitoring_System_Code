package com.example.nrf52840.ecg;

import java.util.LinkedList;
import java.util.Queue;

public class EcgBuffer {
    private final Queue<Float> rawQueue = new LinkedList<>();
    private final Queue<Float> filteredQueue = new LinkedList<>();
    private final int MAX_SIZE = 500;

    public synchronized void addRaw(float val) {
        if (rawQueue.size() >= MAX_SIZE) rawQueue.poll();
        rawQueue.offer(val);
    }

    public synchronized void addFiltered(float val) {
        if (filteredQueue.size() >= MAX_SIZE) filteredQueue.poll();
        filteredQueue.offer(val);
    }

    public synchronized Float pollRaw() {
        return rawQueue.poll();
    }

    public synchronized Float pollFiltered() {
        return filteredQueue.poll();
    }
}
