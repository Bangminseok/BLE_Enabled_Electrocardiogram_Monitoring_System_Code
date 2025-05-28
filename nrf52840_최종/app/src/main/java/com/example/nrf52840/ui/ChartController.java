package com.example.nrf52840.ui;

import com.github.mikephil.charting.charts.LineChart;
import com.github.mikephil.charting.data.*;

public class ChartController {

    private final LineChart chart;
    private final LineDataSet rawSet;
    private final LineDataSet filteredSet;

    public ChartController(LineChart chart, LineDataSet rawSet, LineDataSet filteredSet) {
        this.chart = chart;
        this.rawSet = rawSet;
        this.filteredSet = filteredSet;

        chart.setData(new LineData());
        chart.getDescription().setEnabled(false);
        chart.getAxisRight().setEnabled(false);
    }

    public void addRawEntry(float x, float y) {
        rawSet.addEntry(new Entry(x, y));
        updateChart(rawSet, x);
    }

    public void addFilteredEntry(float x, float y) {
        filteredSet.addEntry(new Entry(x, y));
        updateChart(filteredSet, x);
    }

    private void updateChart(LineDataSet set, float x) {
        LineData data = chart.getData();
        data.clearValues();
        data.addDataSet(set);
        data.notifyDataChanged();

        chart.notifyDataSetChanged();
        chart.setVisibleXRangeMaximum(5);
        chart.moveViewToX(x);
    }

    public void clearOldEntries(int maxSize) {
        if (rawSet.getEntryCount() > maxSize) rawSet.removeFirst();
        if (filteredSet.getEntryCount() > maxSize) filteredSet.removeFirst();
    }
}
