package com.snyde.robotapp;

import android.content.Context;
import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.DashPathEffect;
import android.graphics.Paint;
import android.graphics.Path;
import android.graphics.RectF;
import android.view.View;

import java.util.ArrayList;
import java.util.List;
import java.util.Locale;

/**
 * Track view for the Mapping tab: speed map coloured by target speed, the legacy odometry
 * routes, the live trail while mapping and the robot position while driving.
 * Keeps the aspect ratio, so corners are drawn as they are.
 */
class TrackView extends View {
    private final Paint paint = new Paint(Paint.ANTI_ALIAS_FLAG);
    private final Paint textPaint = new Paint(Paint.ANTI_ALIAS_FLAG);
    private final RectF rect = new RectF();
    private final Path path = new Path();
    private final DashPathEffect dash;

    private SpeedMapModel speedMap;
    private final ArrayList<RoutePoint> recorded = new ArrayList<>();
    private final ArrayList<RoutePoint> optimized = new ArrayList<>();
    private final ArrayList<RoutePoint> robotRoute = new ArrayList<>();
    private final ArrayList<float[]> trail = new ArrayList<>();

    private boolean hasMarker;
    private float markerX;
    private float markerY;
    private float markerYaw;

    private float minX;
    private float maxX;
    private float minY;
    private float maxY;
    private float scale;
    private float offsetX;
    private float offsetY;

    TrackView(Context context) {
        super(context);
        float d = getResources().getDisplayMetrics().density;
        dash = new DashPathEffect(new float[]{10 * d, 7 * d}, 0);
        textPaint.setTextSize(12 * d);
        textPaint.setColor(Ui.MUTED);
    }

    void setSpeedMap(SpeedMapModel model) {
        speedMap = model;
        invalidate();
    }

    void setRoutes(List<RoutePoint> recordedRoute, List<RoutePoint> optimizedRoute, List<RoutePoint> robotMap) {
        copy(recorded, recordedRoute);
        copy(optimized, optimizedRoute);
        copy(robotRoute, robotMap);
        invalidate();
    }

    void clearTrail() {
        trail.clear();
        hasMarker = false;
        invalidate();
    }

    void addTrailPoint(float x, float y) {
        if (!trail.isEmpty()) {
            float[] last = trail.get(trail.size() - 1);
            if (Math.hypot(last[0] - x, last[1] - y) < 0.004) {
                return;
            }
        }
        if (trail.size() > 6000) {
            trail.remove(0);
        }
        trail.add(new float[]{x, y});
        invalidate();
    }

    void setMarker(float x, float y, float yawRad) {
        hasMarker = true;
        markerX = x;
        markerY = y;
        markerYaw = yawRad;
        invalidate();
    }

    void hideMarker() {
        hasMarker = false;
        invalidate();
    }

    private static void copy(ArrayList<RoutePoint> target, List<RoutePoint> source) {
        target.clear();
        if (source != null) {
            target.addAll(source);
        }
    }

    @Override
    protected void onDraw(Canvas canvas) {
        super.onDraw(canvas);
        float w = getWidth();
        float h = getHeight();
        paint.setStyle(Paint.Style.FILL);
        paint.setPathEffect(null);
        paint.setColor(Ui.SURFACE);
        rect.set(0, 0, w, h);
        canvas.drawRoundRect(rect, dp(6), dp(6), paint);

        if (!computeBounds()) {
            textPaint.setTextAlign(Paint.Align.CENTER);
            canvas.drawText("No map", w / 2, h / 2, textPaint);
            textPaint.setTextAlign(Paint.Align.LEFT);
            return;
        }
        fit(w, h);
        drawScaleBar(canvas, w, h);

        paint.setStyle(Paint.Style.STROKE);
        paint.setStrokeCap(Paint.Cap.ROUND);
        paint.setStrokeJoin(Paint.Join.ROUND);

        drawRoute(canvas, recorded, Ui.DIM, 2, false, false);
        drawRoute(canvas, robotRoute, Ui.WARNING, 2, true, false);
        drawRoute(canvas, optimized, Ui.MUTED, 3, false, true);

        float vMin = 0;
        float vMax = 0;
        if (speedMap != null && !speedMap.isEmpty() && speedMap.x.length == speedMap.n) {
            vMin = speedMap.vLow;
            vMax = speedMap.vPeak;
            paint.setStrokeWidth(dp(4));
            for (int i = 1; i < speedMap.n; i++) {
                paint.setColor(speedColor(speedMap.v[i], vMin, vMax));
                canvas.drawLine(sx(speedMap.x[i - 1]), sy(speedMap.y[i - 1]), sx(speedMap.x[i]), sy(speedMap.y[i]),
                        paint);
            }
            if (speedMap.closed) {
                int last = speedMap.n - 1;
                paint.setColor(speedColor(speedMap.v[0], vMin, vMax));
                canvas.drawLine(sx(speedMap.x[last]), sy(speedMap.y[last]), sx(speedMap.x[0]), sy(speedMap.y[0]),
                        paint);
            }
            // start line
            paint.setStrokeWidth(dp(2));
            paint.setColor(Ui.TEXT);
            float a = speedMap.n > 1 ? (float) Math.atan2(speedMap.y[1] - speedMap.y[0], speedMap.x[1] - speedMap.x[0]) : 0;
            float px = (float) -Math.sin(a) * dp(9);
            float py = (float) -Math.cos(a) * dp(9);
            float cx = sx(speedMap.x[0]);
            float cy = sy(speedMap.y[0]);
            canvas.drawLine(cx - px, cy - py, cx + px, cy + py, paint);
        }

        if (trail.size() > 1) {
            paint.setStrokeWidth(dp(2));
            paint.setColor(Color.argb(200, 236, 236, 238));
            path.reset();
            path.moveTo(sx(trail.get(0)[0]), sy(trail.get(0)[1]));
            for (int i = 1; i < trail.size(); i++) {
                path.lineTo(sx(trail.get(i)[0]), sy(trail.get(i)[1]));
            }
            canvas.drawPath(path, paint);
        }

        if (hasMarker) {
            float cx = sx(markerX);
            float cy = sy(markerY);
            paint.setStyle(Paint.Style.FILL);
            paint.setColor(Ui.BG);
            canvas.drawCircle(cx, cy, dp(8), paint);
            paint.setColor(Ui.TEXT);
            canvas.drawCircle(cx, cy, dp(6), paint);
            paint.setStyle(Paint.Style.STROKE);
            paint.setStrokeWidth(dp(2));
            canvas.drawLine(cx, cy, cx + (float) Math.cos(markerYaw) * dp(16), cy - (float) Math.sin(markerYaw) * dp(16),
                    paint);
        }

        if (vMax > vMin) {
            drawLegend(canvas, h, vMin, vMax);
        }
    }

    private void drawRoute(Canvas canvas, ArrayList<RoutePoint> route, int color, int widthDp, boolean dashed,
                           boolean bySpeed) {
        if (route.size() < 2) {
            return;
        }
        float lo = Float.MAX_VALUE;
        float hi = -Float.MAX_VALUE;
        if (bySpeed) {
            for (RoutePoint p : route) {
                lo = Math.min(lo, p.speed);
                hi = Math.max(hi, p.speed);
            }
        }
        paint.setStrokeWidth(dp(widthDp));
        paint.setPathEffect(dashed ? dash : null);
        for (int i = 1; i < route.size(); i++) {
            RoutePoint a = route.get(i - 1);
            RoutePoint b = route.get(i);
            paint.setColor(bySpeed && hi > lo ? speedColor(b.speed, lo, hi) : color);
            canvas.drawLine(sx(a.x), sy(a.y), sx(b.x), sy(b.y), paint);
        }
        paint.setPathEffect(null);
    }

    private void drawLegend(Canvas canvas, float h, float vMin, float vMax) {
        float left = dp(12);
        float y = h - dp(14);
        float width = dp(90);
        paint.setStyle(Paint.Style.STROKE);
        paint.setStrokeWidth(dp(4));
        int steps = 12;
        for (int i = 0; i < steps; i++) {
            float t = i / (float) (steps - 1);
            paint.setColor(speedColor(vMin + (vMax - vMin) * t, vMin, vMax));
            canvas.drawLine(left + width * i / steps, y, left + width * (i + 1) / steps, y, paint);
        }
        canvas.drawText(String.format(Locale.US, "%.1f - %.1f m/s", vMin, vMax), left + width + dp(8), y + dp(4),
                textPaint);
    }

    private void drawScaleBar(Canvas canvas, float w, float h) {
        // 0.5 m or 1 m bar in the top right corner
        float metres = (maxX - minX) > 3 || (maxY - minY) > 3 ? 1.0f : 0.5f;
        float length = metres * scale;
        float right = w - dp(12);
        float y = dp(16);
        paint.setStyle(Paint.Style.STROKE);
        paint.setStrokeWidth(dp(1));
        paint.setColor(Ui.MUTED);
        canvas.drawLine(right - length, y, right, y, paint);
        canvas.drawLine(right - length, y - dp(3), right - length, y + dp(3), paint);
        canvas.drawLine(right, y - dp(3), right, y + dp(3), paint);
        textPaint.setTextAlign(Paint.Align.RIGHT);
        canvas.drawText(metres >= 1 ? "1 m" : "50 cm", right, y + dp(16), textPaint);
        textPaint.setTextAlign(Paint.Align.LEFT);
    }

    static int speedColor(float v, float lo, float hi) {
        float t = hi > lo ? Math.max(0, Math.min(1, (v - lo) / (hi - lo))) : 1;
        // slow red -> amber -> light grey fast
        int r;
        int g;
        int b;
        if (t < 0.5f) {
            float u = t * 2;
            r = Math.round(229 + (245 - 229) * u);
            g = Math.round(72 + (165 - 72) * u);
            b = Math.round(77 + (36 - 77) * u);
        } else {
            float u = (t - 0.5f) * 2;
            r = Math.round(245 + (236 - 245) * u);
            g = Math.round(165 + (236 - 165) * u);
            b = Math.round(36 + (238 - 36) * u);
        }
        return Color.rgb(r, g, b);
    }

    private boolean computeBounds() {
        minX = Float.MAX_VALUE;
        maxX = -Float.MAX_VALUE;
        minY = Float.MAX_VALUE;
        maxY = -Float.MAX_VALUE;
        boolean any = false;
        if (speedMap != null && !speedMap.isEmpty() && speedMap.x.length == speedMap.n) {
            for (int i = 0; i < speedMap.n; i++) {
                include(speedMap.x[i], speedMap.y[i]);
            }
            any = true;
        }
        any |= includeRoute(recorded);
        any |= includeRoute(optimized);
        any |= includeRoute(robotRoute);
        for (float[] p : trail) {
            include(p[0], p[1]);
            any = true;
        }
        if (hasMarker) {
            include(markerX, markerY);
            any = true;
        }
        if (!any) {
            return false;
        }
        float cx = (minX + maxX) / 2;
        float cy = (minY + maxY) / 2;
        float half = Math.max(0.25f, Math.max(maxX - minX, maxY - minY) / 2);
        if (maxX - minX < 0.5f) {
            minX = cx - Math.max(0.25f, (maxX - minX) / 2);
            maxX = cx + Math.max(0.25f, (maxX - minX) / 2);
        }
        if (maxY - minY < 0.5f) {
            minY = cy - Math.max(0.25f, (maxY - minY) / 2);
            maxY = cy + Math.max(0.25f, (maxY - minY) / 2);
        }
        return half > 0;
    }

    private boolean includeRoute(ArrayList<RoutePoint> route) {
        for (RoutePoint p : route) {
            include(p.x, p.y);
        }
        return !route.isEmpty();
    }

    private void include(float x, float y) {
        minX = Math.min(minX, x);
        maxX = Math.max(maxX, x);
        minY = Math.min(minY, y);
        maxY = Math.max(maxY, y);
    }

    private void fit(float w, float h) {
        float pad = dp(28);
        float spanX = Math.max(0.01f, maxX - minX);
        float spanY = Math.max(0.01f, maxY - minY);
        scale = Math.min((w - 2 * pad) / spanX, (h - 2 * pad) / spanY);
        offsetX = (w - spanX * scale) / 2 - minX * scale;
        offsetY = (h - spanY * scale) / 2 + maxY * scale;
    }

    private float sx(float x) {
        return offsetX + x * scale;
    }

    private float sy(float y) {
        return offsetY - y * scale;
    }

    private float dp(float value) {
        return value * getResources().getDisplayMetrics().density;
    }
}
