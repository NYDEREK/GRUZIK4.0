package com.snyde.robotapp;

import android.Manifest;
import android.app.Activity;
import android.app.AlertDialog;
import android.bluetooth.BluetoothAdapter;
import android.bluetooth.BluetoothDevice;
import android.content.Context;
import android.content.Intent;
import android.content.SharedPreferences;
import android.content.pm.PackageManager;
import android.graphics.Typeface;
import android.net.Uri;
import android.os.Build;
import android.os.Bundle;
import android.os.Handler;
import android.os.Looper;
import android.os.SystemClock;
import android.provider.Settings;
import android.text.Editable;
import android.text.InputType;
import android.text.TextUtils;
import android.text.TextWatcher;
import android.view.Gravity;
import android.view.MotionEvent;
import android.view.View;
import android.view.WindowInsets;
import android.widget.AdapterView;
import android.widget.ArrayAdapter;
import android.widget.Button;
import android.widget.EditText;
import android.widget.HorizontalScrollView;
import android.widget.LinearLayout;
import android.widget.ScrollView;
import android.widget.Spinner;
import android.widget.TextView;

import java.util.ArrayList;
import java.util.List;
import java.util.Locale;
import java.util.Set;

/**
 * Bluetooth controller for the NYDER line followers: GRUZIK4.0 (odometry, speed map) and
 * REGIN V3 (turbine). Text protocol of SimpleParser.c, features follow what the robot reports.
 */
@SuppressWarnings("deprecation")
public class MainActivity extends Activity implements BtLink.Listener {
    private static final String PREFS = "nyder_lf";
    private static final int REQUEST_BT = 41;
    private static final int BT_NAME_MAX_LEN = 20;

    private static final int TAB_DRIVE = 0;
    private static final int TAB_SENSORS = 1;
    private static final int TAB_MAPPING = 2;
    private static final int TAB_JOYSTICK = 3;
    private static final int TAB_ODOMETRY = 4;
    private static final int TAB_LOG = 5;
    private static final int TAB_SETTINGS = 6;
    private static final String[] TAB_NAMES = {"Drive", "Sensors", "Mapping", "Joystick", "Odometry", "Log", "Settings"};

    // parameter groups
    private static final int G_COMMON = 0;
    private static final int G_TURBINE = 1;
    private static final int G_CORNER = 2;
    private static final int G_SPEED = 3;
    private static final int G_ODOM = 4;
    private static final int G_ROUTE = 5;
    private static final int G_LOCAL = 6;

    private static final String START_NORMAL = "StartNormal";
    private static final String START_MAPPING = "StartMapping";
    private static final String START_ROUTE = "StartPlayback";
    private static final String START_SPEED = "StartSpeedRun";

    private final class Param {
        final String key;
        final String label;
        final String def;
        final String reginDef;
        final int group;
        final EditText input;
        View view;

        Param(String key, String label, String def, String reginDef, int group) {
            this.key = key;
            this.label = label;
            this.def = def;
            this.reginDef = reginDef;
            this.group = group;
            this.input = ui.numberInput(def);
        }

        String value() {
            return input.getText().toString().trim().replace(',', '.');
        }

        float floatValue(float fallback) {
            try {
                return Float.parseFloat(value());
            } catch (NumberFormatException ex) {
                return fallback;
            }
        }
    }

    private final Handler main = new Handler(Looper.getMainLooper());
    private Ui ui;
    private BtLink link;
    private BluetoothAdapter adapter;
    private final ArrayList<BluetoothDevice> paired = new ArrayList<>();
    private final ArrayList<Param> params = new ArrayList<>();
    private final ArrayList<String> presetNames = new ArrayList<>();
    private final ArrayList<TextView> tabLabels = new ArrayList<>();
    private final ArrayList<View> tabViews = new ArrayList<>();
    private final ArrayList<View> tabUnderlines = new ArrayList<>();
    private final ArrayList<View> pages = new ArrayList<>();

    private RobotProfile profile;
    private int currentTab = TAB_DRIVE;
    private boolean robotRunning;
    private int robotState;
    private String requestedTelemetry = "off";
    private boolean streamPaused;
    private boolean infoReceived;
    private long lastStartAckMs;
    private boolean autoDownloadSpeedMap;
    private float gyroSuggestion = -1;

    // header
    private TextView titleText;
    private TextView voltageText;
    private TextView warningText;
    private Spinner deviceSpinner;
    private Button connectButton;

    // drive
    private View turbineSection;
    private View cornerSection;
    private Spinner presetSpinner;
    private EditText presetNameInput;
    private boolean presetTouched;
    private Param cleanSpeed;

    // sensors
    private SensorBarsView sensorBars;
    private TextView sensorErrorText;
    private TextView sensorPositionText;
    private TextView sensorSummaryText;
    private Button sensorStreamButton;
    private volatile String pendingSensorFrame;
    private volatile boolean sensorFramePosted;
    private int frameCount;
    private long frameWindowStart;
    private float frameRate;

    // mapping
    private TrackView trackView;
    private SpeedProfileView profileView;
    private final SpeedMapModel speedMap = new SpeedMapModel();
    private TextView mapStatusText;
    private TextView transferText;
    private TextView liveSpeedText;
    private TextView liveTargetText;
    private TextView liveLapText;
    private TextView liveSyncText;
    private View speedMapSection;
    private View liveReadouts;
    private View routeSection;
    private View gyroRow;
    private TextView gyroText;
    private Button speedRunButton;
    private final Runnable rebuildProfile = this::rebuildSpeedProfile;
    private String robotReadyText = "";

    // legacy route
    private final ArrayList<RoutePoint> recordedRoute = new ArrayList<>();
    private final ArrayList<RoutePoint> optimizedRoute = new ArrayList<>();
    private final ArrayList<RoutePoint> robotRoute = new ArrayList<>();
    private final ArrayList<RoutePoint> incomingRoute = new ArrayList<>();
    private String incomingRouteKind;
    private TextView routeInfoText;
    private Param optStep;
    private Param optSmooth;
    private Param optBase;
    private Param optGain;
    private Param optMin;
    private Param optMax;

    // odometry
    private TextView odomX;
    private TextView odomY;
    private TextView odomYaw;
    private TextView odomDistance;
    private TextView odomWheels;
    private TextView odomGyro;
    private TextView odomFusion;
    private Param gyroScale;

    // joystick, log, settings
    private Param joystickSpeed;
    private TextView joystickStatus;
    private long lastManualMs;
    private int lastManualLeft;
    private int lastManualRight;
    private TextView logText;
    private final StringBuilder logBuffer = new StringBuilder();
    private boolean logPosted;
    private EditText btNameInput;
    private TextView btNameStatus;
    private Spinner robotTypeSpinner;

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        ui = new Ui(this);
        link = new BtLink(this);
        adapter = BluetoothAdapter.getDefaultAdapter();
        profile = initialProfile();
        buildUi();
        loadParams();
        loadPresetNames();
        applyProfile();
        requestBluetoothPermissionIfNeeded();
        selectTab(prefs().getInt("tab", TAB_DRIVE));
    }

    @Override
    protected void onResume() {
        super.onResume();
        if (link.state() == BtLink.DISCONNECTED) {
            loadPairedDevices();
        }
    }

    @Override
    protected void onPause() {
        saveParams();
        super.onPause();
    }

    @Override
    protected void onDestroy() {
        if (link.isConnected()) {
            link.sendNow("Manual=0,0");
            link.sendNow("Clean=0");
            link.sendNow("Telemetry=off");
        }
        link.close();
        super.onDestroy();
    }

    // ================================================================== UI

    private void buildUi() {
        ScrollView scroll = new ScrollView(this);
        scroll.setFillViewport(true);
        scroll.setBackgroundColor(Ui.BG);
        scroll.setOnApplyWindowInsetsListener(this::applySystemInsets);

        LinearLayout root = ui.column();
        root.setPadding(ui.dp(16), ui.dp(12), ui.dp(16), ui.dp(24));
        scroll.addView(root, ui.matchWrap());

        LinearLayout header = ui.row();
        titleText = ui.label("", 20, Ui.TEXT);
        titleText.setTypeface(Typeface.create("sans-serif-medium", Typeface.NORMAL));
        voltageText = ui.label("", 15, Ui.TEXT);
        voltageText.setGravity(Gravity.END);
        header.addView(titleText, new LinearLayout.LayoutParams(0, LinearLayout.LayoutParams.WRAP_CONTENT, 1f));
        header.addView(voltageText, new LinearLayout.LayoutParams(LinearLayout.LayoutParams.WRAP_CONTENT,
                LinearLayout.LayoutParams.WRAP_CONTENT));
        root.addView(header, ui.matchWrap());

        LinearLayout connection = ui.row();
        deviceSpinner = new Spinner(this);
        connectButton = ui.button("Connect", Ui.STYLE_QUIET);
        connection.addView(deviceSpinner, new LinearLayout.LayoutParams(0, ui.dp(48), 1f));
        LinearLayout.LayoutParams connectParams = new LinearLayout.LayoutParams(ui.dp(116), ui.dp(48));
        connectParams.setMargins(ui.dp(8), 0, 0, 0);
        connection.addView(connectButton, connectParams);
        root.addView(connection, ui.top(8));
        connectButton.setOnClickListener(v -> onConnectClicked());

        Button start = ui.button("Start", Ui.STYLE_PRIMARY);
        Button stop = ui.button("Stop", Ui.STYLE_DANGER);
        root.addView(ui.rowOf(start, stop), ui.top(8));
        start.setOnClickListener(v -> startRun(START_NORMAL, true));
        stop.setOnClickListener(v -> stopRobot());

        warningText = ui.label("", 13, Ui.WARNING);
        warningText.setVisibility(View.GONE);
        root.addView(warningText, ui.top(6));

        root.addView(tabBar(), ui.top(10));
        root.addView(ui.hairline());

        pages.add(buildDrivePage());
        pages.add(buildSensorsPage());
        pages.add(buildMappingPage());
        pages.add(buildJoystickPage());
        pages.add(buildOdometryPage());
        pages.add(buildLogPage());
        pages.add(buildSettingsPage());
        for (View page : pages) {
            root.addView(page, ui.top(4));
        }
        setContentView(scroll);
    }

    private WindowInsets applySystemInsets(View view, WindowInsets insets) {
        if (Build.VERSION.SDK_INT >= 35) {
            android.graphics.Insets bars = insets.getInsets(WindowInsets.Type.systemBars()
                    | WindowInsets.Type.displayCutout() | WindowInsets.Type.ime());
            view.setPadding(bars.left, bars.top, bars.right, bars.bottom);
        }
        return insets;
    }

    private View tabBar() {
        HorizontalScrollView scroller = new HorizontalScrollView(this);
        scroller.setHorizontalScrollBarEnabled(false);
        LinearLayout bar = ui.row();
        for (int i = 0; i < TAB_NAMES.length; i++) {
            LinearLayout tab = ui.column();
            TextView name = ui.label(TAB_NAMES[i], 15, Ui.MUTED);
            name.setPadding(0, ui.dp(10), 0, ui.dp(9));
            View underline = new View(this);
            underline.setBackgroundColor(Ui.TEXT);
            tab.addView(name, new LinearLayout.LayoutParams(LinearLayout.LayoutParams.WRAP_CONTENT,
                    LinearLayout.LayoutParams.WRAP_CONTENT));
            tab.addView(underline, new LinearLayout.LayoutParams(LinearLayout.LayoutParams.MATCH_PARENT, ui.dp(2)));
            final int index = i;
            tab.setOnClickListener(v -> selectTab(index));
            LinearLayout.LayoutParams lp = new LinearLayout.LayoutParams(LinearLayout.LayoutParams.WRAP_CONTENT,
                    LinearLayout.LayoutParams.WRAP_CONTENT);
            lp.setMargins(0, 0, ui.dp(22), 0);
            bar.addView(tab, lp);
            tabViews.add(tab);
            tabLabels.add(name);
            tabUnderlines.add(underline);
        }
        scroller.addView(bar);
        return scroller;
    }

    private boolean tabAvailable(int tab) {
        if (tab == TAB_MAPPING) {
            return profile.routeMap || profile.speedMap;
        }
        if (tab == TAB_ODOMETRY) {
            return profile.odometry;
        }
        return tab >= 0 && tab < pages.size();
    }

    private void selectTab(int tab) {
        if (!tabAvailable(tab)) {
            tab = TAB_DRIVE;
        }
        currentTab = tab;
        for (int i = 0; i < pages.size(); i++) {
            boolean selected = i == tab;
            pages.get(i).setVisibility(selected ? View.VISIBLE : View.GONE);
            tabLabels.get(i).setTextColor(selected ? Ui.TEXT : Ui.MUTED);
            tabUnderlines.get(i).setVisibility(selected ? View.VISIBLE : View.INVISIBLE);
        }
        prefs().edit().putInt("tab", tab).apply();
        updateTelemetry();
    }

    // ------------------------------------------------------------------ parameters

    private Param param(String key, String label, String def, int group) {
        return param(key, label, def, def, group);
    }

    private Param param(String key, String label, String def, String reginDef, int group) {
        Param p = new Param(key, label, def, reginDef, group);
        params.add(p);
        return p;
    }

    private View pair(Param a, Param b) {
        View left = ui.labeled(a.label, a.input);
        a.view = left;
        View right;
        if (b != null) {
            right = ui.labeled(b.label, b.input);
            b.view = right;
        } else {
            right = new View(this);
        }
        return ui.rowOf(left, right);
    }

    private List<String> commandsFor(int... groups) {
        ArrayList<String> commands = new ArrayList<>();
        for (Param p : params) {
            for (int g : groups) {
                if (p.group == g && !p.value().isEmpty()) {
                    commands.add(p.key + "=" + p.value());
                }
            }
        }
        return commands;
    }

    /** Drive values the connected robot understands. */
    private List<String> driveCommands() {
        ArrayList<Integer> groups = new ArrayList<>();
        groups.add(G_COMMON);
        if (profile.turbine) {
            groups.add(G_TURBINE);
        }
        if (profile.cornerOptions) {
            groups.add(G_CORNER);
        }
        if (profile.speedMap) {
            groups.add(G_ODOM);
        }
        int[] array = new int[groups.size()];
        for (int i = 0; i < array.length; i++) {
            array[i] = groups.get(i);
        }
        return commandsFor(array);
    }

    private String prefKey(Param p) {
        return "p." + profile.key + "." + p.key;
    }

    private void loadParams() {
        SharedPreferences store = prefs();
        for (Param p : params) {
            String def = "regin".equals(profile.key) ? p.reginDef : p.def;
            p.input.setText(store.getString(prefKey(p), def));
        }
        btNameInput.setText(store.getString("bt.name." + profile.key, profile.title.replace(' ', '_')));
        updateThreshold();
    }

    private void saveParams() {
        SharedPreferences.Editor editor = prefs().edit();
        for (Param p : params) {
            editor.putString(prefKey(p), p.input.getText().toString().trim());
        }
        editor.putString("bt.name." + profile.key, btNameInput.getText().toString().trim());
        editor.apply();
    }

    // ------------------------------------------------------------------ drive page

    private View buildDrivePage() {
        LinearLayout page = ui.column();

        LinearLayout turbine = ui.column();
        turbine.addView(ui.section("Turbine"), ui.matchWrap());
        turbine.addView(pair(param("Turbine_Speed", "Speed, 0-1000", "100", G_TURBINE),
                param("Turbine_Prep_Time", "Spin-up, ms", "1000", G_TURBINE)), ui.matchWrap());
        turbineSection = turbine;
        page.addView(turbine, ui.matchWrap());

        page.addView(ui.section("PID"), ui.matchWrap());
        page.addView(pair(param("Kp", "Kp", "0.015", G_COMMON), param("Kd", "Kd", "0.55", G_COMMON)), ui.matchWrap());
        Param threshold = param("Treshold", "Threshold", "3300", "3700", G_COMMON);
        page.addView(pair(threshold, null), ui.matchWrap());
        threshold.input.addTextChangedListener(new Watcher(this::updateThreshold));

        page.addView(ui.section("Speed"), ui.matchWrap());
        page.addView(pair(param("Base_speed", "Base", "125", G_COMMON), param("Max_speed", "Max", "200", G_COMMON)),
                ui.matchWrap());
        page.addView(pair(param("Sharp_bend_speed_left", "Sharp bend left", "120", G_COMMON),
                param("Sharp_bend_speed_right", "Sharp bend right", "-75", G_COMMON)), ui.matchWrap());
        page.addView(pair(param("Bend_speed_left", "Bend left", "120", G_COMMON),
                param("Bend_speed_right", "Bend right", "-75", G_COMMON)), ui.matchWrap());

        LinearLayout corner = ui.column();
        corner.addView(ui.section("Corners"), ui.matchWrap());
        corner.addView(pair(param("ErrSlow", "Error slowdown, 0-1", "0", G_CORNER),
                param("DWin", "D window, cycles", "1", G_CORNER)), ui.matchWrap());
        corner.addView(pair(param("CornerLatch", "Corner latch, 0 / 1", "0", G_CORNER),
                param("CornerIn", "Latch at error", "5000", G_CORNER)), ui.matchWrap());
        corner.addView(pair(param("CornerOut", "Release below error", "1500", G_CORNER), null), ui.matchWrap());
        corner.addView(ui.hint("Error slowdown cuts the base speed by slowdown x (error / 5500)². "
                + "Corner latch pivots with the sharp bend speeds from Latch at error until the line is back "
                + "below Release. D window takes D over that many 0.5 ms cycles."), ui.matchWrap());
        cornerSection = corner;
        page.addView(corner, ui.matchWrap());

        Button send = ui.button("Send values", Ui.STYLE_QUIET);
        page.addView(send, ui.top(12));
        send.setOnClickListener(v -> {
            if (requireConnected()) {
                for (String command : driveCommands()) {
                    link.send(command);
                }
            }
        });

        page.addView(ui.section("Presets"), ui.matchWrap());
        presetSpinner = new Spinner(this);
        page.addView(presetSpinner, new LinearLayout.LayoutParams(LinearLayout.LayoutParams.MATCH_PARENT, ui.dp(48)));
        presetSpinner.setOnTouchListener((v, event) -> {
            if (event.getActionMasked() == MotionEvent.ACTION_UP) {
                presetTouched = true;
            }
            return false;
        });
        presetSpinner.setOnItemSelectedListener(new AdapterView.OnItemSelectedListener() {
            @Override
            public void onItemSelected(AdapterView<?> parent, View view, int position, long id) {
                if (presetTouched && position >= 0 && position < presetNames.size()) {
                    loadPreset(presetNames.get(position));
                }
                presetTouched = false;
            }

            @Override
            public void onNothingSelected(AdapterView<?> parent) {
            }
        });
        LinearLayout presetRow = ui.row();
        presetNameInput = ui.input("Name");
        Button save = ui.button("Save", Ui.STYLE_QUIET);
        Button delete = ui.button("Delete", Ui.STYLE_QUIET);
        presetRow.addView(presetNameInput, new LinearLayout.LayoutParams(0, ui.dp(48), 1f));
        presetRow.addView(save, fixed(88));
        presetRow.addView(delete, fixed(88));
        page.addView(presetRow, ui.top(8));
        save.setOnClickListener(v -> saveCurrentPreset());
        delete.setOnClickListener(v -> deleteSelectedPreset());

        page.addView(ui.section("Tire cleaning"), ui.matchWrap());
        cleanSpeed = param("CleanSpeed", "Speed", "170", G_LOCAL);
        Button clean = ui.button("Hold to clean", Ui.STYLE_QUIET);
        LinearLayout cleanRow = ui.row();
        cleanRow.setGravity(Gravity.BOTTOM);
        cleanRow.addView(ui.labeled("Speed", cleanSpeed.input), new LinearLayout.LayoutParams(0,
                LinearLayout.LayoutParams.WRAP_CONTENT, 1f));
        LinearLayout.LayoutParams cleanParams = new LinearLayout.LayoutParams(0, ui.dp(48), 1f);
        cleanParams.setMargins(ui.dp(8), 0, 0, 0);
        cleanRow.addView(clean, cleanParams);
        page.addView(cleanRow, ui.matchWrap());
        clean.setOnTouchListener((view, event) -> {
            int action = event.getActionMasked();
            if (action == MotionEvent.ACTION_DOWN) {
                view.setPressed(true);
                if (requireConnected()) {
                    link.send("CleanSpeed=" + cleanSpeed.value());
                    link.send("Clean=1");
                }
            } else if (action == MotionEvent.ACTION_UP || action == MotionEvent.ACTION_CANCEL) {
                view.setPressed(false);
                link.send("Clean=0");
            }
            return true;
        });
        return page;
    }

    private LinearLayout.LayoutParams fixed(int widthDp) {
        LinearLayout.LayoutParams lp = new LinearLayout.LayoutParams(ui.dp(widthDp), ui.dp(48));
        lp.setMargins(ui.dp(8), 0, 0, 0);
        return lp;
    }

    private void updateThreshold() {
        for (Param p : params) {
            if ("Treshold".equals(p.key) && sensorBars != null) {
                sensorBars.setThreshold(p.floatValue(3300));
            }
        }
    }

    // ------------------------------------------------------------------ sensors page

    private View buildSensorsPage() {
        LinearLayout page = ui.column();
        sensorStreamButton = ui.button("Pause", Ui.STYLE_QUIET);
        page.addView(sensorStreamButton, ui.top(12));
        sensorStreamButton.setOnClickListener(v -> {
            streamPaused = !streamPaused;
            sensorStreamButton.setText(streamPaused ? "Resume" : "Pause");
            updateTelemetry();
        });

        sensorBars = new SensorBarsView(this);
        page.addView(sensorBars, ui.sized(280, 16));

        sensorErrorText = ui.readoutValue("-");
        sensorPositionText = ui.readoutValue("-");
        LinearLayout readouts = ui.rowOf(ui.readout("Error", sensorErrorText),
                ui.readout("Position / centre", sensorPositionText));
        page.addView(readouts, ui.top(12));

        sensorSummaryText = ui.mono("No data", 14, Ui.MUTED);
        page.addView(sensorSummaryText, ui.top(10));
        page.addView(ui.hint("Bars from the robot's left to right, dashed line is the threshold, the tick under "
                + "the bars is the centre. Error = centre - position, plus means the line is right of the centre."),
                ui.matchWrap());
        return page;
    }

    // ------------------------------------------------------------------ mapping page

    private View buildMappingPage() {
        LinearLayout page = ui.column();

        trackView = new TrackView(this);
        trackView.setSpeedMap(speedMap);
        page.addView(trackView, ui.sized(320, 12));

        mapStatusText = ui.label("", 13, Ui.MUTED);
        page.addView(mapStatusText, ui.top(8));
        transferText = ui.label("", 13, Ui.MUTED);
        page.addView(transferText, ui.top(2));

        liveSpeedText = ui.readoutValue("-");
        liveTargetText = ui.readoutValue("-");
        liveLapText = ui.readoutValue("-");
        liveSyncText = ui.readoutValue("-");
        LinearLayout readouts = ui.rowOf(ui.readout("Speed", liveSpeedText), ui.readout("Target", liveTargetText),
                ui.readout("Lap", liveLapText), ui.readout("Sync, mm", liveSyncText));
        liveReadouts = readouts;
        page.addView(readouts, ui.top(10));

        Button startMapping = ui.button("Start mapping", Ui.STYLE_QUIET);
        speedRunButton = ui.button("Speed run", Ui.STYLE_PRIMARY);
        Button stop = ui.button("Stop", Ui.STYLE_DANGER);
        page.addView(ui.rowOf(startMapping, speedRunButton, stop), ui.top(12));
        startMapping.setOnClickListener(v -> startMapping());
        speedRunButton.setOnClickListener(v -> startRun(START_SPEED, true));
        stop.setOnClickListener(v -> stopRobot());

        gyroText = ui.label("", 13, Ui.TEXT);
        Button applyGyro = ui.button("Apply", Ui.STYLE_QUIET);
        LinearLayout gyro = ui.row();
        gyro.addView(gyroText, new LinearLayout.LayoutParams(0, LinearLayout.LayoutParams.WRAP_CONTENT, 1f));
        gyro.addView(applyGyro, fixed(96));
        gyroRow = gyro;
        gyroRow.setVisibility(View.GONE);
        page.addView(gyro, ui.top(8));
        applyGyro.setOnClickListener(v -> applyGyroSuggestion());

        // speed map
        LinearLayout speed = ui.column();
        speed.addView(ui.section("Speed map"), ui.matchWrap());
        Button download = ui.button("Get speed map", Ui.STYLE_QUIET);
        Button sendProfile = ui.button("Send profile", Ui.STYLE_QUIET);
        speed.addView(ui.rowOf(download, sendProfile), ui.top(4));
        download.setOnClickListener(v -> requestSpeedMap());
        sendProfile.setOnClickListener(v -> {
            if (requireConnected()) {
                for (String c : commandsFor(G_SPEED)) {
                    link.send(c);
                }
                link.send("SpeedMapBuild=1");
            }
        });

        profileView = new SpeedProfileView(this);
        profileView.setModel(speedMap);
        speed.addView(profileView, ui.sized(180, 12));

        Watcher profileWatcher = new Watcher(() -> {
            main.removeCallbacks(rebuildProfile);
            main.postDelayed(rebuildProfile, 250);
        });
        Param[] sp = {
                param("SmVmax", "Max speed, m/s", "2.5", G_SPEED),
                param("SmVmin", "Min speed, m/s", "0.8", G_SPEED),
                param("SmAcc", "Acceleration, m/s²", "6", G_SPEED),
                param("SmBrake", "Braking, m/s²", "8", G_SPEED),
                param("SmLat", "Corner grip, m/s²", "7", G_SPEED),
                param("SmLag", "Response lag, ms", "60", G_SPEED),
                param("SmWin", "Curvature window, cm", "10", G_SPEED),
                param("SmLaps", "Laps, 0 = no stop", "0", G_SPEED),
                param("SmKp", "Speed Kp, PWM per m/s", "120", G_SPEED),
                param("SmFF", "Feed-forward, 0 = learned", "0", G_SPEED),
        };
        for (int i = 0; i < sp.length; i += 2) {
            speed.addView(pair(sp[i], sp[i + 1]), ui.matchWrap());
            sp[i].input.addTextChangedListener(profileWatcher);
            sp[i + 1].input.addTextChangedListener(profileWatcher);
        }
        speed.addView(ui.hint("The robot follows the line with the normal PID, only the base speed comes from "
                + "the map. Corner speed = √(grip / curvature). Braking sets how early it slows down before "
                + "a corner, acceleration how fast it speeds up after one. Driven distance re-syncs on every "
                + "corner exit."), ui.matchWrap());

        speed.addView(ui.section("Odometry"), ui.matchWrap());
        gyroScale = param("GyroScale", "Gyro scale", "1.0", G_ODOM);
        speed.addView(pair(gyroScale, param("GyroWeight", "Gyro weight, 0-1", "1.0", G_ODOM)), ui.matchWrap());
        speed.addView(ui.hint("A closed mapping lap suggests a gyro scale, apply it and map again."),
                ui.matchWrap());
        speedMapSection = speed;
        page.addView(speed, ui.matchWrap());

        // legacy odometry route
        LinearLayout route = ui.column();
        route.addView(ui.section("Odometry route"), ui.matchWrap());
        routeInfoText = ui.label("", 13, Ui.MUTED);
        route.addView(routeInfoText, ui.top(2));
        Button getRecorded = ui.button("Get GRUZIK.txt", Ui.STYLE_QUIET);
        Button getRobotMap = ui.button("Get map.txt", Ui.STYLE_QUIET);
        route.addView(ui.rowOf(getRecorded, getRobotMap), ui.top(8));
        getRecorded.setOnClickListener(v -> {
            if (requireConnected()) {
                link.send("MapDump=recorded");
            }
        });
        getRobotMap.setOnClickListener(v -> {
            if (requireConnected()) {
                link.send("MapDump=optimized");
            }
        });
        optStep = param("opt.step", "Point step", "80", G_LOCAL);
        optSmooth = param("opt.smooth", "Smoothing", "5", G_LOCAL);
        optBase = param("opt.base", "Base speed", "40", G_LOCAL);
        optGain = param("opt.gain", "Gain", "60", G_LOCAL);
        optMin = param("opt.min", "Min speed", "35", G_LOCAL);
        optMax = param("opt.max", "Max speed", "95", G_LOCAL);
        route.addView(pair(optStep, optSmooth), ui.matchWrap());
        route.addView(pair(optBase, optGain), ui.matchWrap());
        route.addView(pair(optMin, optMax), ui.matchWrap());
        route.addView(pair(param("MapP", "MapP", "90", G_ROUTE), param("MapI", "MapI", "0", G_ROUTE)), ui.matchWrap());
        route.addView(pair(param("MapD", "MapD", "10", G_ROUTE), param("MapSpeed", "Default speed", "80", G_ROUTE)),
                ui.matchWrap());
        Button optimize = ui.button("Optimize", Ui.STYLE_QUIET);
        Button upload = ui.button("Upload", Ui.STYLE_QUIET);
        Button runRoute = ui.button("Run route", Ui.STYLE_QUIET);
        route.addView(ui.rowOf(optimize, upload, runRoute), ui.top(12));
        optimize.setOnClickListener(v -> optimizeRoute(true));
        upload.setOnClickListener(v -> {
            if (optimizedRoute.isEmpty() && !optimizeRoute(true)) {
                return;
            }
            uploadRoute();
        });
        runRoute.setOnClickListener(v -> runRoute());
        Button clearView = ui.button("Clear view", Ui.STYLE_QUIET);
        route.addView(clearView, ui.top(8));
        clearView.setOnClickListener(v -> {
            recordedRoute.clear();
            optimizedRoute.clear();
            robotRoute.clear();
            trackView.clearTrail();
            updateRouteUi();
        });
        routeSection = route;
        page.addView(route, ui.matchWrap());
        return page;
    }

    // ------------------------------------------------------------------ joystick, odometry, log, settings

    private View buildJoystickPage() {
        LinearLayout page = ui.column();
        joystickSpeed = param("joystick.speed", "Max PWM", "85", G_LOCAL);
        Button stop = ui.button("Stop", Ui.STYLE_DANGER);
        LinearLayout row = ui.row();
        row.setGravity(Gravity.BOTTOM);
        row.addView(ui.labeled("Max PWM", joystickSpeed.input), new LinearLayout.LayoutParams(0,
                LinearLayout.LayoutParams.WRAP_CONTENT, 1f));
        LinearLayout.LayoutParams stopParams = new LinearLayout.LayoutParams(0, ui.dp(48), 1f);
        stopParams.setMargins(ui.dp(8), 0, 0, 0);
        row.addView(stop, stopParams);
        page.addView(row, ui.top(12));

        JoystickView joystick = new JoystickView(this);
        page.addView(joystick, ui.sized(300, 12));
        joystickStatus = ui.mono("L 0   R 0", 14, Ui.MUTED);
        joystickStatus.setGravity(Gravity.CENTER);
        page.addView(joystickStatus, ui.top(8));
        joystick.setListener((forward, turn, active) -> sendManual(forward, turn, !active));
        stop.setOnClickListener(v -> sendManual(0, 0, true));
        return page;
    }

    private View buildOdometryPage() {
        LinearLayout page = ui.column();
        odomX = ui.readoutValue("-");
        odomY = ui.readoutValue("-");
        odomYaw = ui.readoutValue("-");
        odomDistance = ui.readoutValue("-");
        odomWheels = ui.readoutValue("-");
        odomGyro = ui.readoutValue("-");
        odomFusion = ui.mono("-", 14, Ui.TEXT);
        page.addView(ui.rowOf(ui.readout("X, m", odomX), ui.readout("Y, m", odomY)), ui.top(12));
        page.addView(ui.rowOf(ui.readout("Yaw, deg", odomYaw), ui.readout("Distance, m", odomDistance)), ui.top(12));
        page.addView(ui.rowOf(ui.readout("Wheels L / R, m/s", odomWheels), ui.readout("Gyro Z, dps", odomGyro)),
                ui.top(12));
        page.addView(ui.readout("Yaw from encoders / gyro, deg", odomFusion), ui.top(12));
        page.addView(ui.hint("Pose is integrated while the robot drives. Encoder yaw drifting away from gyro yaw "
                + "shows wheel slip."), ui.matchWrap());
        return page;
    }

    private View buildLogPage() {
        LinearLayout page = ui.column();
        Button clear = ui.button("Clear", Ui.STYLE_QUIET);
        page.addView(clear, ui.top(12));
        logText = ui.mono("", 12, Ui.TEXT);
        logText.setTextIsSelectable(true);
        page.addView(logText, ui.top(10));
        clear.setOnClickListener(v -> {
            logBuffer.setLength(0);
            logText.setText("");
        });
        return page;
    }

    private View buildSettingsPage() {
        LinearLayout page = ui.column();
        page.addView(ui.section("Robot"), ui.matchWrap());
        robotTypeSpinner = new Spinner(this);
        robotTypeSpinner.setAdapter(new ArrayAdapter<>(this, android.R.layout.simple_spinner_dropdown_item,
                RobotProfile.OVERRIDE_NAMES));
        robotTypeSpinner.setSelection(prefs().getInt("robot.override", RobotProfile.AUTO));
        page.addView(robotTypeSpinner, new LinearLayout.LayoutParams(LinearLayout.LayoutParams.MATCH_PARENT,
                ui.dp(48)));
        robotTypeSpinner.setOnItemSelectedListener(new AdapterView.OnItemSelectedListener() {
            @Override
            public void onItemSelected(AdapterView<?> parent, View view, int position, long id) {
                if (position != prefs().getInt("robot.override", RobotProfile.AUTO)) {
                    prefs().edit().putInt("robot.override", position).apply();
                    RobotProfile forced = RobotProfile.forOverride(position);
                    if (forced != null) {
                        switchProfile(forced);
                    } else if (link.isConnected()) {
                        link.send("Info=1");
                    }
                }
            }

            @Override
            public void onNothingSelected(AdapterView<?> parent) {
            }
        });
        page.addView(ui.hint("Auto asks the firmware (Info) and falls back to the Bluetooth name."),
                ui.matchWrap());

        page.addView(ui.section("Bluetooth name"), ui.matchWrap());
        LinearLayout nameRow = ui.row();
        btNameInput = ui.input("Name");
        btNameInput.setInputType(InputType.TYPE_CLASS_TEXT | InputType.TYPE_TEXT_FLAG_CAP_CHARACTERS);
        Button rename = ui.button("Rename", Ui.STYLE_QUIET);
        nameRow.addView(btNameInput, new LinearLayout.LayoutParams(0, ui.dp(48), 1f));
        nameRow.addView(rename, fixed(104));
        page.addView(nameRow, ui.matchWrap());
        page.addView(ui.hint("The module has to be in AT mode. Pair it again after renaming."), ui.matchWrap());
        btNameStatus = ui.label("", 13, Ui.TEXT);
        btNameStatus.setVisibility(View.GONE);
        page.addView(btNameStatus, ui.top(6));
        rename.setOnClickListener(v -> confirmRename());
        return page;
    }

    // ================================================================== profile

    private RobotProfile initialProfile() {
        RobotProfile forced = RobotProfile.forOverride(prefs().getInt("robot.override", RobotProfile.AUTO));
        if (forced != null) {
            return forced;
        }
        return "regin".equals(prefs().getString("last.profile", "gruzik"))
                ? RobotProfile.regin()
                : RobotProfile.gruzik(2, "odom|sd|map|speedmap|live|sensors|corner");
    }

    private void switchProfile(RobotProfile next) {
        boolean sameKey = next.key.equals(profile.key);
        if (!sameKey) {
            saveParams();
        }
        profile = next;
        prefs().edit().putString("last.profile", profile.key).apply();
        if (!sameKey) {
            loadParams();
            loadPresetNames();
        }
        applyProfile();
    }

    private void applyProfile() {
        titleText.setText(link.isConnected() ? profile.title : profile.title + "  ·  not connected");
        turbineSection.setVisibility(profile.turbine ? View.VISIBLE : View.GONE);
        cornerSection.setVisibility(profile.cornerOptions ? View.VISIBLE : View.GONE);
        speedMapSection.setVisibility(profile.speedMap ? View.VISIBLE : View.GONE);
        speedRunButton.setVisibility(profile.speedMap ? View.VISIBLE : View.GONE);
        liveReadouts.setVisibility(profile.speedMap ? View.VISIBLE : View.GONE);
        routeSection.setVisibility(profile.routeMap ? View.VISIBLE : View.GONE);
        sensorBars.setCount(profile.sensorCount);
        sensorPositionText.setText("- / " + profile.centre());
        for (int i = 0; i < tabViews.size(); i++) {
            tabViews.get(i).setVisibility(tabAvailable(i) ? View.VISIBLE : View.GONE);
        }
        if (!tabAvailable(currentTab)) {
            selectTab(TAB_DRIVE);
        }
        updateMapStatus();
        updateRouteUi();
    }

    // ================================================================== commands

    private boolean requireConnected() {
        if (!link.isConnected()) {
            showWarning("Not connected");
            return false;
        }
        return true;
    }

    private void startMapping() {
        trackView.clearTrail();
        profileView.clearTrace();
        gyroRow.setVisibility(View.GONE);
        autoDownloadSpeedMap = profile.speedMap;
        transferText.setText("Mapping");
        startRun(START_MAPPING, true);
    }

    /** Values, start command and the live stream, queued in that order. */
    private void startRun(String startCommand, boolean live) {
        if (!requireConnected()) {
            return;
        }
        if (START_SPEED.equals(startCommand) && !profile.speedMap) {
            showWarning("This firmware has no speed map");
            return;
        }
        saveParams();
        hideWarning();
        List<String> commands = new ArrayList<>(driveCommands());
        if (START_SPEED.equals(startCommand)) {
            commands.addAll(commandsFor(G_SPEED));
            profileView.clearTrace();
        } else if (START_ROUTE.equals(startCommand)) {
            commands.addAll(commandsFor(G_ROUTE));
        }
        if (!START_MAPPING.equals(startCommand)) {
            trackView.clearTrail();
        }

        link.send("Telemetry=off");
        for (String c : commands) {
            link.send(c);
        }
        link.pause(100);
        link.send(startCommand + "=1");
        boolean wantLive = live && profile.liveFrames && currentTab == TAB_MAPPING;
        if (wantLive) {
            link.pause(250);
            link.send("Telemetry=live");
            requestedTelemetry = "live";
        }

        final long requested = SystemClock.uptimeMillis();
        String fallback = START_NORMAL.equals(startCommand) ? "P"
                : START_MAPPING.equals(startCommand) ? "M"
                : START_ROUTE.equals(startCommand) ? "U" : null;
        if (fallback != null) {
            long delay = commands.size() * 30L + 1200L;
            main.postDelayed(() -> {
                if (lastStartAckMs < requested && link.isConnected()) {
                    // firmware without StartNormal / StartMapping / StartPlayback
                    link.send("Mode=" + fallback);
                    link.pause(120);
                    link.send("Mode=Y");
                }
            }, delay);
        }
    }

    private void stopRobot() {
        if (!requireConnected()) {
            return;
        }
        link.send("Mode=N");
        link.send("Manual=0,0");
        link.send("Clean=0");
        robotRunning = false;
        main.postDelayed(this::updateTelemetry, 400);
    }

    private void sendManual(float forward, float turn, boolean force) {
        int max = Math.max(0, Math.min(250, Math.round(joystickSpeed.floatValue(85))));
        int left = Math.max(-250, Math.min(250, Math.round((forward + turn) * max)));
        int right = Math.max(-250, Math.min(250, Math.round((forward - turn) * max)));
        long now = SystemClock.uptimeMillis();
        if (!force && (now - lastManualMs) < 70 && Math.abs(left - lastManualLeft) < 4
                && Math.abs(right - lastManualRight) < 4) {
            return;
        }
        lastManualMs = now;
        lastManualLeft = left;
        lastManualRight = right;
        joystickStatus.setText(String.format(Locale.US, "L %d   R %d", left, right));
        if (link.isConnected()) {
            link.sendQuiet(String.format(Locale.US, "Manual=%d,%d", left, right));
        }
    }

    /** One stream at a time, chosen by the open tab. 9600 baud is too slow for more. */
    private void updateTelemetry() {
        if (!link.isConnected()) {
            requestedTelemetry = "off";
            return;
        }
        String wanted = "off";
        if (currentTab == TAB_SENSORS && !streamPaused) {
            wanted = profile.sensorFrames ? "sensors" : "debug";
        } else if (currentTab == TAB_MAPPING && robotRunning && profile.liveFrames) {
            wanted = "live";
        } else if (currentTab == TAB_ODOMETRY && profile.odometry) {
            wanted = "odom";
        }
        if (!wanted.equals(requestedTelemetry)) {
            requestedTelemetry = wanted;
            link.send("Telemetry=" + wanted);
        }
        if (currentTab != TAB_SENSORS) {
            sensorSummaryText.setText("No data");
        }
    }

    private void requestSpeedMap() {
        if (!requireConnected()) {
            return;
        }
        requestedTelemetry = "off";
        transferText.setText("Downloading speed map");
        link.send("SpeedMapDump=1");
    }

    private void applyGyroSuggestion() {
        if (gyroSuggestion <= 0) {
            return;
        }
        gyroScale.input.setText(String.format(Locale.US, "%.5f", gyroSuggestion));
        saveParams();
        if (link.isConnected()) {
            link.send("GyroScale=" + gyroScale.value());
        }
        gyroRow.setVisibility(View.GONE);
    }

    private void confirmRename() {
        String name = btNameInput.getText().toString().trim();
        if (!validName(name)) {
            showBtStatus("Use 1-20 characters: A-Z 0-9 space _ - .");
            return;
        }
        if (!requireConnected()) {
            return;
        }
        new AlertDialog.Builder(this)
                .setTitle("Rename module")
                .setMessage("The Bluetooth link can drop after the rename.")
                .setPositiveButton("Rename", (d, w) -> {
                    saveParams();
                    link.send("BtNameNow=" + name);
                })
                .setNegativeButton("Cancel", null)
                .show();
    }

    private static boolean validName(String name) {
        if (name == null || name.isEmpty() || name.length() > BT_NAME_MAX_LEN) {
            return false;
        }
        for (int i = 0; i < name.length(); i++) {
            char c = name.charAt(i);
            if (!((c >= 'A' && c <= 'Z') || (c >= 'a' && c <= 'z') || (c >= '0' && c <= '9') || c == ' '
                    || c == '_' || c == '-' || c == '.')) {
                return false;
            }
        }
        return true;
    }

    // ------------------------------------------------------------------ legacy route

    private boolean optimizeRoute(boolean report) {
        if (recordedRoute.size() < 2) {
            if (report) {
                showWarning("Download GRUZIK.txt first");
            }
            return false;
        }
        RouteOptimizer.Params p = new RouteOptimizer.Params();
        p.step = Math.round(optStep.floatValue(80));
        p.smoothing = Math.round(optSmooth.floatValue(5));
        p.baseSpeed = optBase.floatValue(40);
        p.gain = optGain.floatValue(60);
        p.minSpeed = optMin.floatValue(35);
        p.maxSpeed = optMax.floatValue(95);
        optimizedRoute.clear();
        optimizedRoute.addAll(RouteOptimizer.optimize(recordedRoute, p));
        updateRouteUi();
        return !optimizedRoute.isEmpty();
    }

    private void uploadRoute() {
        if (!requireConnected() || optimizedRoute.isEmpty()) {
            return;
        }
        int count = optimizedRoute.size();
        transferText.setText("Uploading map.txt, " + count + " points");
        link.send("MapUploadBegin=" + count);
        link.pause(160);
        for (RoutePoint p : optimizedRoute) {
            link.send(String.format(Locale.US, "MapPoint=%.3f,%.3f,%.3f", p.x, p.y, p.speed), 35, false);
        }
        link.send("MapUploadEnd=1");
    }

    private void runRoute() {
        if (!requireConnected()) {
            return;
        }
        if (optimizedRoute.isEmpty() && !optimizeRoute(true)) {
            return;
        }
        uploadRoute();
        link.pause(300);
        startRun(START_ROUTE, true);
    }

    private void updateRouteUi() {
        if (trackView == null) {
            return;
        }
        trackView.setRoutes(recordedRoute, optimizedRoute, robotRoute);
        routeInfoText.setText(String.format(Locale.US, "GRUZIK.txt %d pts, %.2f m   optimized %d pts   map.txt %d pts",
                recordedRoute.size(), RouteOptimizer.length(recordedRoute), optimizedRoute.size(), robotRoute.size()));
    }

    private static RoutePoint parseRoutePoint(String value, boolean recorded) {
        String[] tokens = value.trim().split("[,\\s]+");
        ArrayList<Float> numbers = new ArrayList<>();
        for (String token : tokens) {
            try {
                numbers.add(Float.parseFloat(token));
            } catch (NumberFormatException ignored) {
            }
        }
        if (numbers.size() < 2) {
            return null;
        }
        float speed = 0;
        if (recorded && numbers.size() >= 5) {
            speed = numbers.get(4);
        } else if (numbers.size() >= 3) {
            speed = numbers.get(2);
        }
        return new RoutePoint(numbers.get(0), numbers.get(1), speed);
    }

    // ------------------------------------------------------------------ speed profile

    private SpeedMapModel.Params speedParams() {
        SpeedMapModel.Params p = new SpeedMapModel.Params();
        for (Param q : params) {
            if (q.group != G_SPEED) {
                continue;
            }
            switch (q.key) {
                case "SmVmax": p.vMax = q.floatValue(p.vMax); break;
                case "SmVmin": p.vMin = q.floatValue(p.vMin); break;
                case "SmAcc": p.accel = q.floatValue(p.accel); break;
                case "SmBrake": p.brake = q.floatValue(p.brake); break;
                case "SmLat": p.lat = q.floatValue(p.lat); break;
                case "SmLag": p.lagS = q.floatValue(60) / 1000f; break;
                case "SmWin": p.windowM = q.floatValue(10) / 100f; break;
                case "SmKp": p.kp = q.floatValue(p.kp); break;
                case "SmFF": p.ff = q.floatValue(p.ff); break;
                case "SmLaps": p.laps = Math.round(q.floatValue(0)); break;
                default: break;
            }
        }
        return p;
    }

    private void rebuildSpeedProfile() {
        if (!speedMap.isEmpty()) {
            speedMap.build(speedParams());
            trackView.setSpeedMap(speedMap);
            profileView.setModel(speedMap);
        }
        updateMapStatus();
    }

    private void updateMapStatus() {
        if (mapStatusText == null) {
            return;
        }
        if (!profile.speedMap) {
            mapStatusText.setText(profile.routeMap ? "Odometry route only, flash the new firmware for the speed map" : "");
            return;
        }
        if (speedMap.isEmpty()) {
            mapStatusText.setText("No speed map. Start mapping, or get the one saved on the robot.");
            return;
        }
        String text = String.format(Locale.US, "%.2f m, %s, lap about %.2f s, %.1f - %.1f m/s",
                speedMap.length, speedMap.closed ? "closed" : "open", speedMap.lapTime, speedMap.vLow, speedMap.vPeak);
        if (speedMap.closed && speedMap.closeErrorM > 0) {
            text += String.format(Locale.US, ", lap gap %.0f mm", speedMap.closeErrorM * 1000);
        }
        if (!robotReadyText.isEmpty()) {
            text += "\n" + robotReadyText;
        }
        mapStatusText.setText(text);
    }

    // ================================================================== robot replies

    @Override
    public void onLine(String line) {
        if (line.startsWith("S,") || line.startsWith("DBG,")) {
            // only the newest sensor frame matters
            pendingSensorFrame = line;
            if (!sensorFramePosted) {
                sensorFramePosted = true;
                main.post(() -> {
                    sensorFramePosted = false;
                    String frame = pendingSensorFrame;
                    if (frame != null) {
                        handleSensorFrame(frame);
                    }
                });
            }
            return;
        }
        main.post(() -> handleLine(line));
    }

    @Override
    public void onSent(String command) {
        appendLog("> " + command);
    }

    @Override
    public void onLinkState(int state, String message) {
        main.post(() -> {
            if (state == BtLink.CONNECTED) {
                connectButton.setText("Disconnect");
                deviceSpinner.setEnabled(false);
                hideWarning();
                infoReceived = false;
                requestedTelemetry = "unknown";
                RobotProfile forced = RobotProfile.forOverride(prefs().getInt("robot.override", RobotProfile.AUTO));
                if (forced != null) {
                    switchProfile(forced);
                }
                link.send("Telemetry=off");
                requestedTelemetry = "off";
                link.send("Info=1");
                main.postDelayed(() -> {
                    if (!infoReceived && link.isConnected()
                            && RobotProfile.forOverride(prefs().getInt("robot.override", 0)) == null) {
                        switchProfile(RobotProfile.fromName(link.deviceName()));
                        updateTelemetry();
                    }
                }, 1200);
                applyProfile();
                updateTelemetry();
            } else if (state == BtLink.CONNECTING) {
                connectButton.setText("Connecting");
                deviceSpinner.setEnabled(false);
            } else {
                connectButton.setText("Connect");
                deviceSpinner.setEnabled(true);
                robotRunning = false;
                requestedTelemetry = "off";
                applyProfile();
                if (message != null) {
                    showWarning(message);
                }
            }
        });
    }

    private void handleLine(String line) {
        if (line.startsWith("L,")) {
            handleLiveFrame(line);
            return;
        }
        if (line.startsWith("ODOM,")) {
            handleOdom(line);
            return;
        }
        if (line.startsWith("SMH,")) {
            speedMap.addLine(line.split(","));
            int expected = speedMap.expectedSamples();
            if (expected > 0) {
                transferText.setText(String.format(Locale.US, "Downloading speed map, %d%%",
                        Math.min(100, speedMap.receivedSamples() * 100 / expected)));
            }
            return;
        }
        if (line.startsWith("MAP,") && incomingRouteKind != null) {
            RoutePoint p = parseRoutePoint(line.substring(4), "recorded".equals(incomingRouteKind));
            if (p != null) {
                incomingRoute.add(p);
                if (incomingRoute.size() % 50 == 0) {
                    transferText.setText("Receiving " + incomingRouteKind + ", " + incomingRoute.size() + " points");
                }
            }
            return;
        }

        appendLog("< " + line);

        if (line.startsWith("INFO,")) {
            handleInfo(line);
        } else if (line.startsWith("STARTING,") || "Start".equals(line)) {
            lastStartAckMs = SystemClock.uptimeMillis();
            robotRunning = true;
            hideWarning();
            if (line.startsWith("STARTING,")) {
                transferText.setText(line.substring(9));
            }
            updateTelemetry();
        } else if ("Stop".equals(line)) {
            robotRunning = false;
            trackView.hideMarker();
            updateTelemetry();
        } else if (line.startsWith("SM_BEGIN,")) {
            speedMap.begin(line.split(","));
        } else if (line.startsWith("SM_END")) {
            if (speedMap.finish()) {
                rebuildSpeedProfile();
                transferText.setText(String.format(Locale.US, "Speed map, %d points", speedMap.n));
            } else {
                transferText.setText("Speed map transfer incomplete, try again");
            }
            updateTelemetry();
        } else if (line.startsWith("SPEEDMAP_READY,")) {
            handleSpeedMapReady(line);
        } else if (line.startsWith("SPEEDMAP_SAVED") || line.startsWith("SPEEDMAP_ERROR,save")) {
            if (autoDownloadSpeedMap) {
                autoDownloadSpeedMap = false;
                requestSpeedMap();
            }
        } else if (line.startsWith("SPEEDRUN_DONE")) {
            transferText.setText("Speed run finished");
        } else if (line.startsWith("MAP_AUTO_CLOSED")) {
            robotRunning = false;
            handleAutoClosed(line);
            updateTelemetry();
        } else if (line.startsWith("GYRO_SCALE,")) {
            handleGyroScale(line);
        } else if (line.startsWith("MAP_BEGIN,")) {
            incomingRouteKind = line.substring(10).trim();
            incomingRoute.clear();
        } else if (line.startsWith("MAP_END") && incomingRouteKind != null) {
            if ("recorded".equals(incomingRouteKind)) {
                recordedRoute.clear();
                recordedRoute.addAll(incomingRoute);
                optimizeRoute(false);
            } else {
                robotRoute.clear();
                robotRoute.addAll(incomingRoute);
            }
            transferText.setText("Received " + incomingRouteKind + ", " + incomingRoute.size() + " points");
            incomingRoute.clear();
            incomingRouteKind = null;
            updateRouteUi();
        } else if (line.startsWith("UPLOAD_")) {
            transferText.setText(line);
        } else if (line.startsWith("TELEMETRY,")) {
            String mode = line.substring(10);
            if (mode.startsWith("off,")) {
                showWarning("Sensor stream works only while the robot is stopped");
            }
        } else if (line.startsWith("BT_NAME_OK,")) {
            showBtStatus("Renamed. Remove the pairing in Android and pair again.");
        } else if (line.startsWith("BT_NAME_ERROR,at_no_response")) {
            showBtStatus("No AT response. Put the module in AT mode and try again.");
        } else if (line.startsWith("BT_NAME_")) {
            showBtStatus(line);
        }

        if (line.startsWith("!") || line.contains("ERROR") || line.contains("stop_robot_first")
                || line.startsWith("Stop robot") || line.startsWith("map.txt is empty")) {
            if (!line.startsWith("TELEMETRY,")) {
                showWarning(line.replace("!", "").trim());
            }
            if (line.startsWith("MAP_ERROR,open_read,map.txt,4") || line.startsWith("MAP_ERROR,open_playback,map.txt,4")) {
                robotRoute.clear();
                updateRouteUi();
            }
        }

        int battery = line.indexOf("Battery");
        if (battery >= 0) {
            String volts = firstNumber(line.substring(battery));
            if (volts != null) {
                voltageText.setText(volts + " V");
            }
        }
    }

    private void handleInfo(String line) {
        // INFO,<name>,<fw>,<sensors>,<caps>,<battery_mV>,<speed map samples>
        infoReceived = true;
        String[] t = line.split(",");
        String name = t.length > 1 ? t[1].trim() : "";
        int fw = parseInt(t, 2, 1);
        int sensors = parseInt(t, 3, 12);
        String caps = t.length > 4 ? t[4] : "";
        int mv = parseInt(t, 5, 0);
        if (mv > 0) {
            voltageText.setText(String.format(Locale.US, "%.2f V", mv / 1000f));
        }
        if (RobotProfile.forOverride(prefs().getInt("robot.override", 0)) != null) {
            return;
        }
        String upper = name.toUpperCase(Locale.US);
        RobotProfile next;
        if (upper.contains("GRUZIK")) {
            next = RobotProfile.gruzik(fw, caps);
        } else if (upper.contains("REGIN")) {
            next = RobotProfile.regin();
        } else {
            next = RobotProfile.generic(name, sensors);
        }
        switchProfile(next);
        updateTelemetry();
        if (next.speedMap && speedMap.isEmpty() && parseInt(t, 6, 0) > 0) {
            mapStatusText.setText("Speed map saved on the robot, use Get speed map.");
        }
    }

    private void handleSpeedMapReady(String line) {
        // SPEEDMAP_READY,count,len_mm,lap_ms,peak_mms,ff_x10,landmarks,closed
        String[] t = line.split(",");
        robotReadyText = String.format(Locale.US, "Robot: %.2f m, lap about %.2f s, peak %.2f m/s, FF %.1f, %d corners",
                parseInt(t, 2, 0) / 1000f, parseInt(t, 3, 0) / 1000f, parseInt(t, 4, 0) / 1000f,
                parseInt(t, 5, 0) / 10f, parseInt(t, 6, 0));
        updateMapStatus();
    }

    private void handleAutoClosed(String line) {
        // MAP_AUTO_CLOSED,err_mm,len_mm,turn_cdeg,dropped
        String[] t = line.split(",");
        if (t.length >= 4) {
            transferText.setText(String.format(Locale.US, "Lap closed, %.2f m, %d mm from the start, turn %.1f deg",
                    parseInt(t, 2, 0) / 1000f, parseInt(t, 1, 0), parseInt(t, 3, 0) / 100f));
        } else {
            transferText.setText("Lap closed");
        }
        trackView.hideMarker();
    }

    private void handleGyroScale(String line) {
        // GYRO_SCALE,suggested,turns,error_cdeg
        String[] t = line.split(",");
        try {
            gyroSuggestion = Float.parseFloat(t[1]);
        } catch (RuntimeException ex) {
            return;
        }
        float error = parseInt(t, 3, 0) / 100f;
        gyroText.setText(String.format(Locale.US, "Gyro scale %.5f, the lap closed %+.1f deg off", gyroSuggestion,
                error));
        gyroRow.setVisibility(Math.abs(gyroSuggestion - gyroScale.floatValue(1)) > 0.0002f ? View.VISIBLE : View.GONE);
    }

    private void handleLiveFrame(String line) {
        // L,s_mm,x_mm,y_mm,yaw_cdeg,v_mms,vt_mms,state,lap,sync_mm
        String[] t = line.split(",");
        if (t.length < 10) {
            return;
        }
        float s = parseInt(t, 1, 0) / 1000f;
        float x = parseInt(t, 2, 0) / 1000f;
        float y = parseInt(t, 3, 0) / 1000f;
        float yaw = (float) Math.toRadians(parseInt(t, 4, 0) / 100.0);
        float v = parseInt(t, 5, 0) / 1000f;
        float vt = parseInt(t, 6, 0) / 1000f;
        robotState = parseInt(t, 7, 0);
        int lap = parseInt(t, 8, 0);
        int sync = parseInt(t, 9, 0);
        robotRunning = robotState != 0;

        liveSpeedText.setText(String.format(Locale.US, "%.2f", v));
        if (robotState == 4) {
            liveTargetText.setText(String.format(Locale.US, "%.2f", vt));
            liveLapText.setText(String.valueOf(lap + 1));
            liveSyncText.setText(String.valueOf(sync));
            if (!speedMap.isEmpty() && speedMap.x.length == speedMap.n) {
                int i = speedMap.indexAt(s);
                int j = Math.min(speedMap.n - 1, i + 1);
                float heading = (float) Math.atan2(speedMap.y[j] - speedMap.y[i], speedMap.x[j] - speedMap.x[i]);
                trackView.setMarker(speedMap.x[i], speedMap.y[i], heading);
            }
            profileView.addTrace(s, v);
        } else if (robotState != 0) {
            liveTargetText.setText("-");
            liveLapText.setText("-");
            liveSyncText.setText("-");
            trackView.addTrailPoint(x, y);
            trackView.setMarker(x, y, yaw);
        }
    }

    private void handleOdom(String line) {
        String[] t = line.split(",");
        if (t.length < 10) {
            return;
        }
        odomX.setText(t[1]);
        odomY.setText(t[2]);
        odomYaw.setText(t[3]);
        odomDistance.setText(t[4]);
        odomWheels.setText(t[5] + " / " + t[6]);
        odomGyro.setText(t[7]);
        odomFusion.setText(t[8] + " / " + t[9]);
    }

    private void handleSensorFrame(String line) {
        String[] t = line.split(",");
        int position;
        int active;
        int lastEnd;
        int[] values;
        if (line.startsWith("S,")) {
            // S,pos,actives,last_end,<3 hex per sensor, left to right>,battery_mV
            if (t.length < 5) {
                return;
            }
            position = parseInt(t, 1, 0);
            active = parseInt(t, 2, 0);
            lastEnd = parseInt(t, 3, 0);
            String hex = t[4].trim();
            int count = hex.length() / 3;
            values = new int[count];
            for (int i = 0; i < count; i++) {
                try {
                    values[i] = Integer.parseInt(hex.substring(i * 3, i * 3 + 3), 16);
                } catch (NumberFormatException ex) {
                    values[i] = 0;
                }
            }
            int mv = parseInt(t, 5, 0);
            if (mv > 0) {
                voltageText.setText(String.format(Locale.US, "%.2f V", mv / 1000f));
            }
        } else {
            // DBG,pos,actives,last_end,<9 encoder and IMU fields>,S1..Sn
            if (t.length < 14) {
                return;
            }
            position = parseInt(t, 1, 0);
            active = parseInt(t, 2, 0);
            lastEnd = parseInt(t, 3, 0);
            int count = t.length - 13;
            values = new int[count];
            boolean reverse = profile.dbgRightToLeft || count == 16;
            for (int i = 0; i < count; i++) {
                int v = parseInt(t, 13 + i, 0);
                values[reverse ? count - 1 - i : i] = v;
            }
        }

        if (values.length != sensorBars.count() && values.length > 0) {
            sensorBars.setCount(values.length);
        }
        int count = sensorBars.count();
        int centre = (count + 1) * 500;
        sensorBars.setData(values, (count + 1) * 1000 - position, active);

        long now = SystemClock.uptimeMillis();
        frameCount++;
        if (now - frameWindowStart >= 1000) {
            frameRate = frameCount * 1000f / Math.max(1, now - frameWindowStart);
            frameCount = 0;
            frameWindowStart = now;
        }
        // Last_end: 1 = the line was last seen on the right edge
        String side = lastEnd == 1 ? "right" : "left";
        if (active > 0) {
            int error = centre - position;
            sensorErrorText.setText(error > 0 ? "+" + error : String.valueOf(error));
            sensorPositionText.setText(position + " / " + centre);
            sensorSummaryText.setText(String.format(Locale.US, "Active %-3d Last end %-6s %4.1f Hz", active, side,
                    frameRate));
        } else {
            sensorErrorText.setText("-");
            sensorPositionText.setText("- / " + centre);
            sensorSummaryText.setText(String.format(Locale.US, "Line lost  Last end %-6s %4.1f Hz", side, frameRate));
        }
    }

    // ================================================================== Bluetooth

    private void onConnectClicked() {
        if (link.state() == BtLink.CONNECTED) {
            link.send("Telemetry=off");
            main.postDelayed(link::close, 150);
        } else if (link.state() == BtLink.DISCONNECTED) {
            connectSelected();
        }
    }

    private void connectSelected() {
        if (!missingPermissions().isEmpty()) {
            requestBluetoothPermissionIfNeeded();
            new AlertDialog.Builder(this)
                    .setTitle("Bluetooth permission")
                    .setMessage("Allow Nearby devices for this app in Android settings.")
                    .setPositiveButton("Settings", (d, w) -> {
                        Intent intent = new Intent(Settings.ACTION_APPLICATION_DETAILS_SETTINGS);
                        intent.setData(Uri.fromParts("package", getPackageName(), null));
                        startActivity(intent);
                    })
                    .setNegativeButton("Cancel", null)
                    .show();
            return;
        }
        int index = deviceSpinner.getSelectedItemPosition();
        if (index < 0 || index >= paired.size()) {
            showWarning("Pair the robot's Bluetooth module in Android settings first");
            return;
        }
        BluetoothDevice device = paired.get(index);
        prefs().edit().putString("last.device", device.getAddress()).apply();
        hideWarning();
        link.connect(adapter, device);
    }

    private void requestBluetoothPermissionIfNeeded() {
        ArrayList<String> missing = missingPermissions();
        if (!missing.isEmpty()) {
            requestPermissions(missing.toArray(new String[0]), REQUEST_BT);
        }
    }

    @Override
    public void onRequestPermissionsResult(int requestCode, String[] permissions, int[] grantResults) {
        super.onRequestPermissionsResult(requestCode, permissions, grantResults);
        if (requestCode == REQUEST_BT) {
            loadPairedDevices();
        }
    }

    private ArrayList<String> missingPermissions() {
        ArrayList<String> list = new ArrayList<>();
        if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.S) {
            if (checkSelfPermission(Manifest.permission.BLUETOOTH_CONNECT) != PackageManager.PERMISSION_GRANTED) {
                list.add(Manifest.permission.BLUETOOTH_CONNECT);
            }
            if (checkSelfPermission(Manifest.permission.BLUETOOTH_SCAN) != PackageManager.PERMISSION_GRANTED) {
                list.add(Manifest.permission.BLUETOOTH_SCAN);
            }
        } else if (checkSelfPermission(Manifest.permission.ACCESS_FINE_LOCATION) != PackageManager.PERMISSION_GRANTED) {
            list.add(Manifest.permission.ACCESS_FINE_LOCATION);
        }
        return list;
    }

    private void loadPairedDevices() {
        paired.clear();
        ArrayList<String> names = new ArrayList<>();
        if (adapter == null) {
            names.add("Bluetooth not available");
        } else if (!missingPermissions().isEmpty()) {
            names.add("Bluetooth permission needed");
        } else if (!adapter.isEnabled()) {
            names.add("Bluetooth is off");
        } else {
            try {
                Set<BluetoothDevice> bonded = adapter.getBondedDevices();
                for (BluetoothDevice device : bonded) {
                    paired.add(device);
                    String name = device.getName() == null ? "Unknown" : device.getName();
                    names.add(name + "  " + device.getAddress());
                }
            } catch (SecurityException ex) {
                paired.clear();
                names.clear();
                names.add("Bluetooth permission needed");
            }
            if (names.isEmpty()) {
                names.add("No paired devices");
            }
        }
        deviceSpinner.setAdapter(new ArrayAdapter<>(this, android.R.layout.simple_spinner_dropdown_item, names));
        String last = prefs().getString("last.device", "");
        for (int i = 0; i < paired.size(); i++) {
            if (paired.get(i).getAddress().equals(last)) {
                deviceSpinner.setSelection(i);
                break;
            }
        }
    }

    // ================================================================== presets

    private String presetListKey() {
        return "presets." + profile.key;
    }

    private String presetKey(String name, String field) {
        return "preset." + profile.key + "." + name + "." + field;
    }

    private boolean inPreset(Param p) {
        return p.group == G_COMMON || p.group == G_TURBINE || p.group == G_CORNER;
    }

    private void loadPresetNames() {
        presetNames.clear();
        String joined = prefs().getString(presetListKey(), "");
        if (joined != null) {
            for (String part : joined.split("\\|", -1)) {
                if (!part.trim().isEmpty()) {
                    presetNames.add(part.trim());
                }
            }
        }
        updatePresetSpinner();
    }

    private void saveCurrentPreset() {
        String name = presetNameInput.getText().toString().trim().replace("|", "");
        if (name.isEmpty()) {
            showWarning("Preset name is empty");
            return;
        }
        SharedPreferences.Editor editor = prefs().edit();
        for (Param p : params) {
            if (inPreset(p)) {
                editor.putString(presetKey(name, p.key), p.input.getText().toString().trim());
            }
        }
        if (!presetNames.contains(name)) {
            presetNames.add(name);
        }
        editor.putString(presetListKey(), TextUtils.join("|", presetNames));
        editor.apply();
        updatePresetSpinner();
        presetSpinner.setSelection(presetNames.indexOf(name));
    }

    private void loadPreset(String name) {
        presetNameInput.setText(name);
        for (Param p : params) {
            if (inPreset(p)) {
                p.input.setText(prefs().getString(presetKey(name, p.key), p.input.getText().toString()));
            }
        }
    }

    private void deleteSelectedPreset() {
        int index = presetSpinner.getSelectedItemPosition();
        if (index < 0 || index >= presetNames.size()) {
            return;
        }
        String name = presetNames.get(index);
        new AlertDialog.Builder(this)
                .setTitle("Delete preset")
                .setMessage(name)
                .setPositiveButton("Delete", (d, w) -> {
                    SharedPreferences.Editor editor = prefs().edit();
                    for (Param p : params) {
                        editor.remove(presetKey(name, p.key));
                    }
                    presetNames.remove(name);
                    editor.putString(presetListKey(), TextUtils.join("|", presetNames));
                    editor.apply();
                    updatePresetSpinner();
                })
                .setNegativeButton("Cancel", null)
                .show();
    }

    private void updatePresetSpinner() {
        ArrayList<String> names = new ArrayList<>(presetNames);
        if (names.isEmpty()) {
            names.add("No presets");
        }
        presetSpinner.setAdapter(new ArrayAdapter<>(this, android.R.layout.simple_spinner_dropdown_item, names));
    }

    // ================================================================== messages and helpers

    private SharedPreferences prefs() {
        return getSharedPreferences(PREFS, Context.MODE_PRIVATE);
    }

    private void showWarning(String message) {
        warningText.setText(message);
        warningText.setVisibility(View.VISIBLE);
    }

    private void hideWarning() {
        warningText.setVisibility(View.GONE);
    }

    private void showBtStatus(String message) {
        btNameStatus.setText(message);
        btNameStatus.setVisibility(View.VISIBLE);
    }

    private void appendLog(String value) {
        String text = value.endsWith("\n") ? value : value + "\n";
        synchronized (logBuffer) {
            logBuffer.append(text);
            if (logBuffer.length() > 9000) {
                logBuffer.delete(0, logBuffer.length() - 8000);
            }
            if (logPosted) {
                return;
            }
            logPosted = true;
        }
        main.postDelayed(() -> {
            synchronized (logBuffer) {
                logPosted = false;
                logText.setText(logBuffer.toString());
            }
        }, 120);
    }

    private static String firstNumber(String text) {
        StringBuilder number = new StringBuilder();
        for (int i = 0; i < text.length(); i++) {
            char c = text.charAt(i);
            if ((c >= '0' && c <= '9') || c == '.') {
                number.append(c);
            } else if (number.length() > 0) {
                break;
            }
        }
        return number.length() > 0 ? number.toString() : null;
    }

    private static int parseInt(String[] t, int index, int fallback) {
        if (index < 0 || index >= t.length) {
            return fallback;
        }
        try {
            return Integer.parseInt(t[index].trim());
        } catch (NumberFormatException ex) {
            try {
                return Math.round(Float.parseFloat(t[index].trim()));
            } catch (NumberFormatException ignored) {
                return fallback;
            }
        }
    }

    private static final class Watcher implements TextWatcher {
        private final Runnable onChange;

        Watcher(Runnable onChange) {
            this.onChange = onChange;
        }

        @Override
        public void beforeTextChanged(CharSequence s, int start, int count, int after) {
        }

        @Override
        public void onTextChanged(CharSequence s, int start, int before, int count) {
        }

        @Override
        public void afterTextChanged(Editable s) {
            onChange.run();
        }
    }
}
