package com.snyde.robotapp;

import java.util.ArrayList;
import java.util.List;

/**
 * Odometry route playback (map.txt): picks every n-th recorded point, smooths the route and
 * gives each point a PWM speed from the turn angle. Same maths as the previous app version.
 */
final class RouteOptimizer {
    static final class Params {
        int step = 80;
        int smoothing = 5;
        float baseSpeed = 40;
        float gain = 60;
        float minSpeed = 35;
        float maxSpeed = 95;
    }

    private RouteOptimizer() {
    }

    static boolean isClosed(List<RoutePoint> route) {
        return route.size() > 2 && distance(route.get(0), route.get(route.size() - 1)) < 0.08f;
    }

    static ArrayList<RoutePoint> optimize(List<RoutePoint> recorded, Params p) {
        ArrayList<RoutePoint> result = new ArrayList<>();
        if (recorded.size() < 2) {
            return result;
        }
        int step = Math.max(1, Math.min(500, p.step));
        int smoothing = Math.max(1, Math.min(101, p.smoothing));
        float minSpeed = Math.min(p.minSpeed, p.maxSpeed);
        float maxSpeed = Math.max(p.minSpeed, p.maxSpeed);

        boolean closed = isClosed(recorded);
        ArrayList<RoutePoint> source = new ArrayList<>(recorded);
        if (closed) {
            source.remove(source.size() - 1);
        }
        if (source.size() < 2) {
            return result;
        }

        ArrayList<RoutePoint> sampled = new ArrayList<>();
        for (int i = 0; i < source.size(); i += step) {
            sampled.add(source.get(i));
        }
        RoutePoint last = source.get(source.size() - 1);
        if (distance(sampled.get(sampled.size() - 1), last) > 0.001f) {
            sampled.add(last);
        }

        ArrayList<RoutePoint> smoothed = closed ? smoothClosed(sampled, smoothing) : smoothOpen(sampled, smoothing);
        int n = smoothed.size();
        for (int i = 0; i < n; i++) {
            RoutePoint point = smoothed.get(i);
            RoutePoint previous = closed ? smoothed.get((i + n - 1) % n) : smoothed.get(Math.max(0, i - 1));
            RoutePoint next = closed ? smoothed.get((i + 1) % n) : smoothed.get(Math.min(n - 1, i + 1));
            float turn = turnAmount(previous, point, next);
            float speed = clamp(maxSpeed - p.gain * turn, minSpeed, maxSpeed);
            if (turn < 0.20f) {
                speed = Math.max(speed, Math.min(p.baseSpeed, maxSpeed));
            }
            result.add(new RoutePoint(point.x, point.y, clamp(speed, minSpeed, maxSpeed)));
        }
        return result;
    }

    static float length(List<RoutePoint> route) {
        float total = 0;
        for (int i = 1; i < route.size(); i++) {
            total += distance(route.get(i - 1), route.get(i));
        }
        return total;
    }

    private static float turnAmount(RoutePoint previous, RoutePoint point, RoutePoint next) {
        double a = Math.atan2(point.y - previous.y, point.x - previous.x);
        double b = Math.atan2(next.y - point.y, next.x - point.x);
        double d = b - a;
        while (d > Math.PI) {
            d -= 2 * Math.PI;
        }
        while (d < -Math.PI) {
            d += 2 * Math.PI;
        }
        return (float) (Math.abs(d) / Math.PI);
    }

    private static ArrayList<RoutePoint> smoothClosed(List<RoutePoint> input, int window) {
        ArrayList<RoutePoint> output = new ArrayList<>();
        int count = Math.min(Math.max(1, window), input.size());
        int radius = count / 2;
        for (int i = 0; i < input.size(); i++) {
            float x = 0;
            float y = 0;
            for (int j = 0; j < count; j++) {
                RoutePoint p = input.get((i - radius + j + input.size()) % input.size());
                x += p.x;
                y += p.y;
            }
            output.add(new RoutePoint(x / count, y / count, 0));
        }
        return output;
    }

    private static ArrayList<RoutePoint> smoothOpen(List<RoutePoint> input, int window) {
        ArrayList<RoutePoint> output = new ArrayList<>();
        int radius = Math.max(0, window / 2);
        for (int i = 0; i < input.size(); i++) {
            int from = Math.max(0, i - radius);
            int to = Math.min(input.size() - 1, i + radius);
            float x = 0;
            float y = 0;
            for (int j = from; j <= to; j++) {
                x += input.get(j).x;
                y += input.get(j).y;
            }
            int count = to - from + 1;
            output.add(new RoutePoint(x / count, y / count, 0));
        }
        return output;
    }

    static float distance(RoutePoint a, RoutePoint b) {
        return (float) Math.hypot(b.x - a.x, b.y - a.y);
    }

    private static float clamp(float v, float min, float max) {
        return Math.max(min, Math.min(max, v));
    }
}
