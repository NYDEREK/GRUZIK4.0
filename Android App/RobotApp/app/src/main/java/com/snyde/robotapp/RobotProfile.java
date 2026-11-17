package com.snyde.robotapp;

import java.util.Locale;

/**
 * What the connected robot can do. Firmware with the Info command reports itself,
 * older firmware is recognised by the Bluetooth name.
 */
final class RobotProfile {
    static final int AUTO = 0;
    static final int GRUZIK = 1;
    static final int REGIN = 2;
    static final String[] OVERRIDE_NAMES = {"Auto", "GRUZIK4.0", "REGIN V3"};

    final String key;
    final String title;
    final int sensorCount;
    /** Legacy DBG frames list the sensors from the robot's right edge. */
    final boolean dbgRightToLeft;
    final boolean turbine;
    final boolean odometry;
    final boolean routeMap;
    final boolean speedMap;
    final boolean liveFrames;
    final boolean sensorFrames;
    final boolean cornerOptions;
    final int firmware;

    private RobotProfile(String key, String title, int sensorCount, boolean dbgRightToLeft, boolean turbine,
                         boolean odometry, boolean routeMap, boolean speedMap, boolean liveFrames,
                         boolean sensorFrames, boolean cornerOptions, int firmware) {
        this.key = key;
        this.title = title;
        this.sensorCount = sensorCount;
        this.dbgRightToLeft = dbgRightToLeft;
        this.turbine = turbine;
        this.odometry = odometry;
        this.routeMap = routeMap;
        this.speedMap = speedMap;
        this.liveFrames = liveFrames;
        this.sensorFrames = sensorFrames;
        this.cornerOptions = cornerOptions;
        this.firmware = firmware;
    }

    static RobotProfile gruzik(int firmware, String caps) {
        boolean v2 = firmware >= 2;
        String c = caps == null ? "" : caps;
        return new RobotProfile("gruzik", "GRUZIK4.0", 12, false, false, true, true,
                v2 && c.contains("speedmap"), v2 && c.contains("live"), v2 && c.contains("sensors"),
                v2 && c.contains("corner"), firmware);
    }

    static RobotProfile regin() {
        return new RobotProfile("regin", "REGIN V3", 16, true, true, false, false, false, false, false, false, 0);
    }

    static RobotProfile generic(String name, int sensors) {
        String title = (name == null || name.isEmpty()) ? "Robot" : name;
        return new RobotProfile("generic", title, sensors, false, false, false, false, false, false, false, false, 0);
    }

    /** Guess from the Bluetooth device name when the firmware does not answer Info. */
    static RobotProfile fromName(String name) {
        String n = name == null ? "" : name.toUpperCase(Locale.US);
        if (n.contains("REGIN")) {
            return regin();
        }
        if (n.contains("GRUZIK")) {
            return gruzik(1, "");
        }
        return generic(name, 12);
    }

    static RobotProfile forOverride(int override) {
        if (override == GRUZIK) {
            return gruzik(2, "odom|sd|map|speedmap|live|sensors|corner");
        }
        if (override == REGIN) {
            return regin();
        }
        return null;
    }

    /** Line position of the middle of the bar, sensor k weighs k * 1000. */
    int centre() {
        return (sensorCount + 1) * 500;
    }
}
