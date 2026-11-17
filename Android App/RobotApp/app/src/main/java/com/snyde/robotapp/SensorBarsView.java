package com.snyde.robotapp;

import android.content.Context;
import android.graphics.Canvas;
import android.graphics.DashPathEffect;
import android.graphics.Paint;
import android.graphics.Path;
import android.graphics.RectF;
import android.graphics.Typeface;
import android.view.View;

/**
 * Line sensors from the robot's left to right: raw ADC value above each bar, sensor number
 * below, dashed threshold line and the line position marker. 12 (GRUZIK4.0) or 16 (REGIN V3).
 */
class SensorBarsView extends View {
    private static final float ADC_MAX = 4095.0f;

    private int count = 12;
    private final int[] values = new int[32];
    private int valueCount;
    private float threshold = 3300.0f;
    private int displayPosition;
    private int actives;

    private final Paint fillPaint = new Paint(Paint.ANTI_ALIAS_FLAG);
    private final Paint textPaint = new Paint(Paint.ANTI_ALIAS_FLAG);
    private final Paint thresholdPaint = new Paint(Paint.ANTI_ALIAS_FLAG);
    private final RectF rect = new RectF();
    private final Path marker = new Path();

    SensorBarsView(Context context) {
        super(context);
        setMinimumHeight(Math.round(dp(220)));
        textPaint.setTextAlign(Paint.Align.CENTER);
        textPaint.setTypeface(Typeface.create("sans-serif-condensed", Typeface.NORMAL));
        thresholdPaint.setStyle(Paint.Style.STROKE);
        thresholdPaint.setStrokeWidth(dp(1));
        thresholdPaint.setColor(Ui.WARNING);
        thresholdPaint.setPathEffect(new DashPathEffect(new float[]{dp(4), dp(3)}, 0));
    }

    void setCount(int newCount) {
        int clamped = Math.max(1, Math.min(values.length, newCount));
        if (clamped != count) {
            count = clamped;
            valueCount = 0;
            invalidate();
        }
    }

    int count() {
        return count;
    }

    void setThreshold(float value) {
        threshold = value;
        invalidate();
    }

    /** values from left to right, position measured from the left edge (1000 = first sensor). */
    void setData(int[] newValues, int newDisplayPosition, int newActives) {
        valueCount = Math.min(count, newValues.length);
        System.arraycopy(newValues, 0, values, 0, valueCount);
        displayPosition = newDisplayPosition;
        actives = newActives;
        invalidate();
    }

    void clear() {
        valueCount = 0;
        invalidate();
    }

    @Override
    protected void onDraw(Canvas canvas) {
        super.onDraw(canvas);
        float width = getWidth();
        float height = getHeight();
        float column = width / count;

        float textSize = sp(12);
        textPaint.setTextSize(textSize);
        float widest = textPaint.measureText("0000");
        float maxWidth = column - dp(2);
        if (widest > maxWidth) {
            textSize = textSize * maxWidth / widest;
            textPaint.setTextSize(textSize);
        }

        float valueRow = textSize + dp(8);
        float labelRow = textSize + dp(6);
        float markerRow = dp(12);
        float top = valueRow;
        float bottom = height - labelRow - markerRow;
        float chartHeight = Math.max(1.0f, bottom - top);
        float barWidth = Math.max(dp(4), column * 0.62f);
        float radius = dp(2);

        for (int i = 0; i < count; i++) {
            float cx = column * (i + 0.5f);
            float left = cx - barWidth * 0.5f;
            float right = cx + barWidth * 0.5f;

            fillPaint.setColor(Ui.FIELD);
            rect.set(left, top, right, bottom);
            canvas.drawRoundRect(rect, radius, radius, fillPaint);

            if (i < valueCount) {
                int value = Math.max(0, Math.min((int) ADC_MAX, values[i]));
                boolean onLine = value > threshold;
                float barTop = bottom - chartHeight * (value / ADC_MAX);
                fillPaint.setColor(onLine ? Ui.TEXT : Ui.DIM);
                rect.set(left, barTop, right, bottom);
                canvas.drawRoundRect(rect, radius, radius, fillPaint);
                textPaint.setColor(onLine ? Ui.TEXT : Ui.MUTED);
                canvas.drawText(String.valueOf(value), cx, valueRow - dp(5) - textPaint.descent(), textPaint);
            }

            textPaint.setColor(Ui.MUTED);
            canvas.drawText(String.valueOf(i + 1), cx, height - dp(2) - textPaint.descent(), textPaint);
        }

        float thresholdY = bottom - chartHeight * (Math.max(0.0f, Math.min(ADC_MAX, threshold)) / ADC_MAX);
        canvas.drawLine(0.0f, thresholdY, width, thresholdY, thresholdPaint);

        float centreX = column * count * 0.5f;
        fillPaint.setColor(Ui.MUTED);
        rect.set(centreX - dp(1), bottom + dp(3), centreX + dp(1), bottom + markerRow);
        canvas.drawRect(rect, fillPaint);

        if ((valueCount > 0) && (actives > 0)) {
            float markerX = column * (displayPosition / 1000.0f - 0.5f);
            float markerTop = bottom + dp(3);
            marker.reset();
            marker.moveTo(markerX, markerTop);
            marker.lineTo(markerX - dp(5), markerTop + dp(7));
            marker.lineTo(markerX + dp(5), markerTop + dp(7));
            marker.close();
            fillPaint.setColor(Ui.TEXT);
            canvas.drawPath(marker, fillPaint);
        }
    }

    private float dp(int value) {
        return value * getResources().getDisplayMetrics().density;
    }

    private float sp(int value) {
        return value * getResources().getDisplayMetrics().scaledDensity;
    }
}
