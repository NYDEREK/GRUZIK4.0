package com.snyde.robotapp;

/**
 * Speed map downloaded from the robot (heading every few cm of driven distance) and the
 * speed profile built from it. The profile maths is the same as speed_map.c in the firmware,
 * so the preview here matches what the robot drives.
 */
final class SpeedMapModel {
    static final class Params {
        float vMax = 2.5f;
        float vMin = 0.8f;
        float accel = 6.0f;
        float brake = 8.0f;
        float lat = 7.0f;
        float lagS = 0.06f;
        float windowM = 0.10f;
        float kp = 120.0f;
        float ff = 0.0f;
        int laps = 0;
    }

    private static final float TWO_PI = (float) (2.0 * Math.PI);

    // received data
    private int sourceCount;
    private int step = 2;
    private float sourceDs = 0.01f;
    private float[] incoming;
    private int received;

    // map
    float ds = 0.02f;
    int n;
    float[] heading = new float[0];
    boolean closed;
    float totalTurn;
    float ffAuto;

    // profile and shape
    float[] kappa = new float[0];
    float[] v = new float[0];
    float[] x = new float[0];
    float[] y = new float[0];
    float length;
    float lapTime;
    float vPeak;
    float vLow;
    float closeErrorM;
    float turnCorrection = 1.0f;
    boolean valid;

    boolean isEmpty() {
        return !valid || n < 2;
    }

    // ------------------------------------------------------------------ transfer

    /** SM_BEGIN,count,ds_0.1mm,closed,step,ff_x10,total_turn_mrad */
    void begin(String[] t) {
        sourceCount = parseInt(t, 1, 0);
        sourceDs = parseInt(t, 2, 100) / 10000.0f;
        closed = parseInt(t, 3, 0) != 0;
        step = Math.max(1, parseInt(t, 4, 2));
        ffAuto = parseInt(t, 5, 0) / 10.0f;
        totalTurn = parseInt(t, 6, 0) / 1000.0f;
        int samples = (sourceCount + step - 1) / step;
        incoming = new float[Math.max(1, samples)];
        received = 0;
    }

    /** SMH,index,heading_mrad,d1,d2,... */
    void addLine(String[] t) {
        if (incoming == null || t.length < 3) {
            return;
        }
        int index = parseInt(t, 1, -1);
        if (index < 0 || index % step != 0) {
            return;
        }
        int slot = index / step;
        long value = parseInt(t, 2, 0);
        for (int k = 2; k < t.length; k++) {
            if (k > 2) {
                value += parseInt(t, k, 0);
            }
            if (slot >= incoming.length) {
                break;
            }
            incoming[slot] = value / 1000.0f;
            slot++;
            received = Math.max(received, slot);
        }
    }

    int receivedSamples() {
        return received;
    }

    int expectedSamples() {
        return incoming == null ? 0 : incoming.length;
    }

    /** SM_END: keeps the new map when it arrived complete. */
    boolean finish() {
        if (incoming == null || received < Math.max(10, incoming.length - 1)) {
            incoming = null;
            return false;
        }
        heading = new float[received];
        System.arraycopy(incoming, 0, heading, 0, received);
        n = received;
        ds = sourceDs * step;
        incoming = null;
        valid = n >= 10;
        return valid;
    }

    void clear() {
        valid = false;
        n = 0;
        heading = new float[0];
        kappa = new float[0];
        v = new float[0];
        x = new float[0];
        y = new float[0];
    }

    // ------------------------------------------------------------------ profile

    private float headingAt(int index) {
        if (closed) {
            float turns = 0.0f;
            while (index < 0) {
                index += n;
                turns -= 1.0f;
            }
            while (index >= n) {
                index -= n;
                turns += 1.0f;
            }
            return heading[index] + turns * totalTurn;
        }
        return heading[Math.max(0, Math.min(n - 1, index))];
    }

    void build(Params p) {
        if (isEmpty()) {
            return;
        }
        float vMax = clamp(p.vMax, 0.2f, 8.0f);
        float vMin = clamp(p.vMin, 0.1f, vMax);
        float accel = clamp(p.accel, 0.2f, 60.0f);
        float brake = clamp(p.brake, 0.2f, 60.0f);
        float lat = clamp(p.lat, 0.2f, 60.0f);
        float window = clamp(p.windowM, 0.02f, 0.40f);
        int span = Math.max(2, Math.round(window / ds));

        kappa = new float[n];
        v = new float[n];
        for (int i = 0; i < n; i++) {
            int a = i - span / 2;
            int b = a + span;
            if (!closed) {
                a = Math.max(0, a);
                b = Math.min(n - 1, b);
            }
            float len = (b - a) * ds;
            float k = len > 0 ? (headingAt(b) - headingAt(a)) / len : 0.0f;
            kappa[i] = k;
            float speed = Math.abs(k) > 1e-4f ? (float) Math.sqrt(lat / Math.abs(k)) : vMax;
            v[i] = clamp(speed, vMin, vMax);
        }

        int rounds = closed ? 2 : 1;
        float next = closed ? v[0] : vMin;
        for (int r = 0; r < rounds; r++) {
            for (int i = n - 1; i >= 0; i--) {
                float limit = (float) Math.sqrt(next * next + 2.0f * brake * ds);
                if (v[i] > limit) {
                    v[i] = limit;
                }
                next = v[i];
            }
        }
        float prev = closed ? v[n - 1] : vMin;
        for (int r = 0; r < rounds; r++) {
            for (int i = 0; i < n; i++) {
                float limit = (float) Math.sqrt(prev * prev + 2.0f * accel * ds);
                if (v[i] > limit) {
                    v[i] = limit;
                }
                prev = v[i];
            }
        }

        lapTime = 0.0f;
        vPeak = 0.0f;
        vLow = Float.MAX_VALUE;
        for (int i = 0; i < n; i++) {
            lapTime += ds / Math.max(0.05f, v[i]);
            vPeak = Math.max(vPeak, v[i]);
            vLow = Math.min(vLow, v[i]);
        }
        length = n * ds;
        buildShape();
    }

    /**
     * x/y from the heading. On a closed lap the heading is scaled to a whole number of turns
     * (gyro scale error) and the remaining position gap is spread along the lap.
     */
    private void buildShape() {
        float[] h = new float[n];
        System.arraycopy(heading, 0, h, 0, n);
        turnCorrection = 1.0f;
        if (closed) {
            int turns = Math.round(totalTurn / TWO_PI);
            if (turns != 0 && Math.abs(totalTurn - turns * TWO_PI) < 0.6f && Math.abs(totalTurn) > 1e-3f) {
                turnCorrection = (turns * TWO_PI) / totalTurn;
                for (int i = 0; i < n; i++) {
                    h[i] = h[0] + (h[i] - h[0]) * turnCorrection;
                }
            }
        }

        x = new float[n];
        y = new float[n];
        for (int i = 1; i < n; i++) {
            double mid = 0.5 * (h[i - 1] + h[i]);
            x[i] = x[i - 1] + (float) (ds * Math.cos(mid));
            y[i] = y[i - 1] + (float) (ds * Math.sin(mid));
        }

        closeErrorM = 0.0f;
        if (closed && n > 2) {
            double last = h[n - 1];
            float ex = x[n - 1] + (float) (ds * Math.cos(last)) - x[0];
            float ey = y[n - 1] + (float) (ds * Math.sin(last)) - y[0];
            closeErrorM = (float) Math.hypot(ex, ey);
            for (int i = 0; i < n; i++) {
                float f = (float) i / n;
                x[i] -= ex * f;
                y[i] -= ey * f;
            }
        }
    }

    /** Map index for a distance along the lap. */
    int indexAt(float s) {
        if (isEmpty()) {
            return 0;
        }
        int i = (int) (s / ds);
        if (closed) {
            i %= n;
            if (i < 0) {
                i += n;
            }
            return i;
        }
        return Math.max(0, Math.min(n - 1, i));
    }

    float speedAt(float s) {
        return (v.length == n && n > 0) ? v[indexAt(s)] : 0.0f;
    }

    private static float clamp(float value, float min, float max) {
        return Math.max(min, Math.min(max, value));
    }

    private static int parseInt(String[] t, int index, int fallback) {
        if (index < 0 || index >= t.length) {
            return fallback;
        }
        try {
            return Integer.parseInt(t[index].trim());
        } catch (NumberFormatException ex) {
            return fallback;
        }
    }
}
