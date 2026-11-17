package com.snyde.robotapp;

import android.content.Context;
import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;
import android.graphics.Path;
import android.graphics.RectF;
import android.view.View;

import java.util.Locale;

/**
 * Speed profile over the lap: target speed (line), curvature (bars at the bottom),
 * measured speed of the current run (amber) and the robot position (cursor).
 */
class SpeedProfileView extends View {
    private static final int TRACE_MAX = 4000;

    private final Paint paint = new Paint(Paint.ANTI_ALIAS_FLAG);
    private final Paint textPaint = new Paint(Paint.ANTI_ALIAS_FLAG);
    private final Path path = new Path();
    private final RectF rect = new RectF();

    private SpeedMapModel model;
    private final float[] traceS = new float[TRACE_MAX];
    private final float[] traceV = new float[TRACE_MAX];
    private int traceCount;
    private float cursorS = -1;

    SpeedProfileView(Context context) {
        super(context);
        textPaint.setTextSize(11 * getResources().getDisplayMetrics().density);
        textPaint.setColor(Ui.MUTED);
    }

    void setModel(SpeedMapModel newModel) {
        model = newModel;
        invalidate();
    }

    void clearTrace() {
        traceCount = 0;
        cursorS = -1;
        invalidate();
    }

    void addTrace(float s, float v) {
        if (traceCount > 0 && s < traceS[traceCount - 1] - 0.5f) {
            traceCount = 0; // new lap
        }
        if (traceCount < TRACE_MAX) {
            traceS[traceCount] = s;
            traceV[traceCount] = v;
            traceCount++;
        }
        cursorS = s;
        invalidate();
    }

    @Override
    protected void onDraw(Canvas canvas) {
        super.onDraw(canvas);
        float w = getWidth();
        float h = getHeight();
        paint.setStyle(Paint.Style.FILL);
        paint.setColor(Ui.SURFACE);
        rect.set(0, 0, w, h);
        canvas.drawRoundRect(rect, dp(6), dp(6), paint);

        if (model == null || model.isEmpty() || model.v.length != model.n) {
            textPaint.setTextAlign(Paint.Align.CENTER);
            canvas.drawText("No speed profile", w / 2, h / 2, textPaint);
            textPaint.setTextAlign(Paint.Align.LEFT);
            return;
        }

        float left = dp(34);
        float right = w - dp(10);
        float top = dp(10);
        float bottom = h - dp(20);
        float vTop = (float) Math.ceil(Math.max(model.vPeak, 0.5f) * 1.1f * 2) / 2;
        float length = Math.max(0.01f, model.length);

        // grid and speed labels
        paint.setStyle(Paint.Style.STROKE);
        paint.setStrokeWidth(dp(1));
        float stepV = vTop > 3 ? 1.0f : 0.5f;
        for (float v = 0; v <= vTop + 1e-3f; v += stepV) {
            float y = bottom - (bottom - top) * v / vTop;
            paint.setColor(Ui.GRID);
            canvas.drawLine(left, y, right, y, paint);
            canvas.drawText(String.format(Locale.US, "%.1f", v), dp(4), y + dp(4), textPaint);
        }
        canvas.drawText("0 m", left, h - dp(5), textPaint);
        textPaint.setTextAlign(Paint.Align.RIGHT);
        canvas.drawText(String.format(Locale.US, "%.2f m", length), right, h - dp(5), textPaint);
        textPaint.setTextAlign(Paint.Align.LEFT);

        // curvature, scaled to 15 1/m
        paint.setStyle(Paint.Style.FILL);
        paint.setColor(Color.rgb(46, 46, 52));
        float barBottom = bottom;
        float barHeight = (bottom - top) * 0.35f;
        int n = model.n;
        int stride = Math.max(1, n / Math.max(1, (int) (right - left)));
        for (int i = 0; i < n; i += stride) {
            float x = left + (right - left) * (i * model.ds) / length;
            float k = Math.min(15.0f, Math.abs(model.kappa[i])) / 15.0f;
            rect.set(x, barBottom - barHeight * k, x + Math.max(1, (right - left) / n * stride), barBottom);
            canvas.drawRect(rect, paint);
        }

        // target speed
        paint.setStyle(Paint.Style.STROKE);
        paint.setStrokeWidth(dp(2));
        paint.setColor(Ui.TEXT);
        path.reset();
        for (int i = 0; i < n; i += stride) {
            float x = left + (right - left) * (i * model.ds) / length;
            float y = bottom - (bottom - top) * Math.min(model.v[i], vTop) / vTop;
            if (i == 0) {
                path.moveTo(x, y);
            } else {
                path.lineTo(x, y);
            }
        }
        canvas.drawPath(path, paint);

        // measured speed in this lap
        if (traceCount > 1) {
            paint.setColor(Ui.WARNING);
            paint.setStrokeWidth(dp(1.5f));
            path.reset();
            for (int i = 0; i < traceCount; i++) {
                float x = left + (right - left) * Math.min(1, traceS[i] / length);
                float y = bottom - (bottom - top) * Math.min(traceV[i], vTop) / vTop;
                if (i == 0) {
                    path.moveTo(x, y);
                } else {
                    path.lineTo(x, y);
                }
            }
            canvas.drawPath(path, paint);
        }

        if (cursorS >= 0) {
            float x = left + (right - left) * Math.min(1, cursorS / length);
            paint.setColor(Ui.MUTED);
            paint.setStrokeWidth(dp(1));
            canvas.drawLine(x, top, x, bottom, paint);
        }
    }

    private float dp(float value) {
        return value * getResources().getDisplayMetrics().density;
    }
}
