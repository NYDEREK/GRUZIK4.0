package com.snyde.robotapp;

import android.bluetooth.BluetoothAdapter;
import android.bluetooth.BluetoothDevice;
import android.bluetooth.BluetoothSocket;

import java.io.IOException;
import java.io.InputStream;
import java.io.OutputStream;
import java.nio.charset.StandardCharsets;
import java.util.UUID;
import java.util.concurrent.LinkedBlockingQueue;

/**
 * Bluetooth SPP link to the HC-04/HC-05 module.
 * One reader thread splits incoming text into lines, one writer thread sends queued commands
 * with a short gap, so the robot's 64 byte receive buffer never overflows.
 */
final class BtLink {
    interface Listener {
        /** Called on the reader thread. */
        void onLine(String line);

        /** Called on any thread. */
        void onLinkState(int state, String message);

        /** Called on the writer thread for every logged command. */
        void onSent(String command);
    }

    static final int DISCONNECTED = 0;
    static final int CONNECTING = 1;
    static final int CONNECTED = 2;

    private static final UUID SPP_UUID = UUID.fromString("00001101-0000-1000-8000-00805F9B34FB");
    private static final int DEFAULT_GAP_MS = 22;

    private static final class Item {
        final String text;
        final int gapMs;
        final boolean log;

        Item(String text, int gapMs, boolean log) {
            this.text = text;
            this.gapMs = gapMs;
            this.log = log;
        }
    }

    private static final Item STOP = new Item("", 0, false);

    private final Listener listener;
    private final LinkedBlockingQueue<Item> queue = new LinkedBlockingQueue<>();
    private volatile BluetoothSocket socket;
    private volatile OutputStream output;
    private volatile int state = DISCONNECTED;
    private volatile String deviceName = "";
    private Thread reader;
    private Thread writer;

    BtLink(Listener listener) {
        this.listener = listener;
    }

    int state() {
        return state;
    }

    boolean isConnected() {
        return state == CONNECTED;
    }

    String deviceName() {
        return deviceName;
    }

    void connect(BluetoothAdapter adapter, BluetoothDevice device) {
        if (state != DISCONNECTED) {
            return;
        }
        state = CONNECTING;
        listener.onLinkState(CONNECTING, null);
        new Thread(() -> {
            try {
                String name = device.getName();
                deviceName = name == null ? device.getAddress() : name;
                BluetoothSocket newSocket = device.createRfcommSocketToServiceRecord(SPP_UUID);
                adapter.cancelDiscovery();
                newSocket.connect();
                socket = newSocket;
                output = newSocket.getOutputStream();
                queue.clear();
                startReader(newSocket.getInputStream());
                startWriter();
                state = CONNECTED;
                listener.onLinkState(CONNECTED, null);
            } catch (IOException | SecurityException ex) {
                shutdown("Connection failed: " + ex.getMessage());
            }
        }, "bt-connect").start();
    }

    /** Queue a command, \n is added when missing. */
    void send(String command) {
        send(command, DEFAULT_GAP_MS, true);
    }

    void sendQuiet(String command) {
        send(command, 0, false);
    }

    void send(String command, int gapMs, boolean log) {
        if (state != CONNECTED) {
            return;
        }
        String text = command.endsWith("\n") ? command : command + "\n";
        queue.offer(new Item(text, gapMs, log));
    }

    /** Pause in the command stream, e.g. after a start command. */
    void pause(int ms) {
        if (state == CONNECTED) {
            queue.offer(new Item(null, ms, false));
        }
    }

    void close() {
        shutdown(null);
    }

    private void startReader(InputStream input) {
        reader = new Thread(() -> {
            byte[] buffer = new byte[512];
            StringBuilder line = new StringBuilder();
            while (true) {
                int len;
                try {
                    len = input.read(buffer);
                } catch (IOException ex) {
                    shutdown(state == CONNECTED ? "Connection lost" : null);
                    return;
                }
                if (len < 0) {
                    shutdown("Connection lost");
                    return;
                }
                for (int i = 0; i < len; i++) {
                    char c = (char) (buffer[i] & 0xFF);
                    if (c == '\n') {
                        String text = line.toString().trim();
                        line.setLength(0);
                        if (!text.isEmpty()) {
                            listener.onLine(text);
                        }
                    } else if (c != '\r' && line.length() < 600) {
                        line.append(c);
                    }
                }
            }
        }, "bt-read");
        reader.start();
    }

    private void startWriter() {
        writer = new Thread(() -> {
            while (true) {
                Item item;
                try {
                    item = queue.take();
                } catch (InterruptedException ex) {
                    return;
                }
                if (item == STOP) {
                    return;
                }
                OutputStream stream = output;
                if (stream == null) {
                    return;
                }
                if (item.text != null) {
                    try {
                        stream.write(item.text.getBytes(StandardCharsets.US_ASCII));
                        stream.flush();
                    } catch (IOException ex) {
                        shutdown("Send failed: " + ex.getMessage());
                        return;
                    }
                    if (item.log) {
                        listener.onSent(item.text);
                    }
                }
                if (item.gapMs > 0) {
                    try {
                        Thread.sleep(item.gapMs);
                    } catch (InterruptedException ex) {
                        return;
                    }
                }
            }
        }, "bt-write");
        writer.start();
    }

    private synchronized void shutdown(String message) {
        if (state == DISCONNECTED && socket == null) {
            return;
        }
        BluetoothSocket current = socket;
        socket = null;
        output = null;
        state = DISCONNECTED;
        queue.clear();
        queue.offer(STOP);
        if (current != null) {
            try {
                current.close();
            } catch (IOException ignored) {
            }
        }
        listener.onLinkState(DISCONNECTED, message);
    }

    /** Last commands before closing the app, written directly. */
    void sendNow(String command) {
        OutputStream stream = output;
        if (stream == null) {
            return;
        }
        try {
            stream.write((command.endsWith("\n") ? command : command + "\n").getBytes(StandardCharsets.US_ASCII));
            stream.flush();
        } catch (IOException ignored) {
        }
    }
}
