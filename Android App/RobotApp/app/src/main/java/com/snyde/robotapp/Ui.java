package com.snyde.robotapp;

import android.content.Context;
import android.graphics.Color;
import android.graphics.Typeface;
import android.graphics.drawable.GradientDrawable;
import android.graphics.drawable.StateListDrawable;
import android.text.InputType;
import android.view.Gravity;
import android.view.View;
import android.widget.Button;
import android.widget.EditText;
import android.widget.LinearLayout;
import android.widget.TextView;

/** Palette and view factory, same look as the REGIN V3 app. */
final class Ui {
    static final int BG = Color.rgb(14, 14, 15);
    static final int FIELD = Color.rgb(30, 30, 33);
    static final int SURFACE = Color.rgb(24, 24, 26);
    static final int PRESSED = Color.rgb(44, 44, 48);
    static final int LINE = Color.rgb(42, 42, 46);
    static final int GRID = Color.rgb(34, 34, 37);
    static final int TEXT = Color.rgb(236, 236, 238);
    static final int MUTED = Color.rgb(140, 140, 148);
    static final int DIM = Color.rgb(88, 88, 94);
    static final int DANGER = Color.rgb(229, 72, 77);
    static final int WARNING = Color.rgb(245, 165, 36);

    static final int STYLE_QUIET = 0;
    static final int STYLE_PRIMARY = 1;
    static final int STYLE_DANGER = 2;

    private final Context context;
    private final float density;

    Ui(Context context) {
        this.context = context;
        this.density = context.getResources().getDisplayMetrics().density;
    }

    int dp(float value) {
        return Math.round(value * density);
    }

    LinearLayout column() {
        LinearLayout layout = new LinearLayout(context);
        layout.setOrientation(LinearLayout.VERTICAL);
        return layout;
    }

    LinearLayout row() {
        LinearLayout row = new LinearLayout(context);
        row.setOrientation(LinearLayout.HORIZONTAL);
        row.setGravity(Gravity.CENTER_VERTICAL);
        return row;
    }

    TextView label(String text, int sp, int color) {
        TextView label = new TextView(context);
        label.setText(text);
        label.setTextSize(sp);
        label.setTextColor(color);
        return label;
    }

    TextView section(String text) {
        TextView label = label(text, 14, TEXT);
        label.setTypeface(Typeface.create("sans-serif-medium", Typeface.NORMAL));
        label.setPadding(0, dp(20), 0, dp(2));
        return label;
    }

    TextView hint(String text) {
        TextView label = label(text, 13, MUTED);
        label.setPadding(0, dp(6), 0, 0);
        return label;
    }

    TextView mono(String text, int sp, int color) {
        TextView label = label(text, sp, color);
        label.setTypeface(Typeface.MONOSPACE);
        return label;
    }

    TextView readoutValue(String text) {
        TextView value = label(text, 22, TEXT);
        value.setTypeface(Typeface.create("sans-serif-medium", Typeface.NORMAL));
        value.setFontFeatureSettings("tnum");
        return value;
    }

    LinearLayout readout(String title, TextView value) {
        LinearLayout box = column();
        box.addView(label(title, 12, MUTED), matchWrap());
        box.addView(value, matchWrap());
        return box;
    }

    View hairline() {
        View line = new View(context);
        line.setBackgroundColor(LINE);
        line.setLayoutParams(new LinearLayout.LayoutParams(LinearLayout.LayoutParams.MATCH_PARENT, dp(1)));
        return line;
    }

    EditText input(String hint) {
        EditText input = new EditText(context);
        input.setHint(hint);
        input.setTextSize(15);
        input.setTextColor(TEXT);
        input.setHintTextColor(MUTED);
        input.setSingleLine(true);
        input.setPadding(dp(12), 0, dp(12), 0);
        input.setBackground(rounded(FIELD, FIELD));
        return input;
    }

    EditText numberInput(String value) {
        EditText input = input(value);
        input.setText(value);
        input.setInputType(InputType.TYPE_CLASS_NUMBER | InputType.TYPE_NUMBER_FLAG_DECIMAL
                | InputType.TYPE_NUMBER_FLAG_SIGNED);
        return input;
    }

    LinearLayout labeled(String title, View input) {
        LinearLayout box = column();
        TextView label = label(title, 12, MUTED);
        label.setPadding(0, dp(8), 0, dp(4));
        box.addView(label, matchWrap());
        box.addView(input, new LinearLayout.LayoutParams(LinearLayout.LayoutParams.MATCH_PARENT, dp(48)));
        return box;
    }

    Button button(String text, int style) {
        Button button = new Button(context);
        button.setText(text);
        button.setTextSize(15);
        button.setAllCaps(false);
        button.setStateListAnimator(null);
        button.setTypeface(Typeface.create("sans-serif-medium", Typeface.NORMAL));
        button.setMinHeight(dp(48));
        button.setMinimumHeight(dp(48));
        button.setPadding(dp(12), 0, dp(12), 0);
        if (style == STYLE_PRIMARY) {
            button.setTextColor(BG);
            button.setBackground(states(TEXT, Color.rgb(196, 196, 200), TEXT));
        } else if (style == STYLE_DANGER) {
            button.setTextColor(Color.WHITE);
            button.setBackground(states(DANGER, Color.rgb(180, 52, 56), DANGER));
        } else {
            button.setTextColor(TEXT);
            button.setBackground(states(BG, PRESSED, LINE));
        }
        return button;
    }

    StateListDrawable states(int normal, int pressed, int stroke) {
        StateListDrawable states = new StateListDrawable();
        states.addState(new int[]{android.R.attr.state_pressed}, rounded(pressed, stroke));
        states.addState(new int[]{}, rounded(normal, stroke));
        return states;
    }

    GradientDrawable rounded(int color, int stroke) {
        GradientDrawable drawable = new GradientDrawable();
        drawable.setColor(color);
        drawable.setCornerRadius(dp(6));
        drawable.setStroke(dp(1), stroke);
        return drawable;
    }

    LinearLayout.LayoutParams matchWrap() {
        return new LinearLayout.LayoutParams(LinearLayout.LayoutParams.MATCH_PARENT,
                LinearLayout.LayoutParams.WRAP_CONTENT);
    }

    LinearLayout.LayoutParams top(int marginDp) {
        LinearLayout.LayoutParams params = matchWrap();
        params.setMargins(0, dp(marginDp), 0, 0);
        return params;
    }

    LinearLayout.LayoutParams sized(int heightDp, int marginDp) {
        LinearLayout.LayoutParams params = new LinearLayout.LayoutParams(LinearLayout.LayoutParams.MATCH_PARENT,
                dp(heightDp));
        params.setMargins(0, dp(marginDp), 0, 0);
        return params;
    }

    /** Equal width cell in a row, 4 dp gap towards the neighbours. */
    LinearLayout.LayoutParams cell(int index, int count) {
        LinearLayout.LayoutParams params = new LinearLayout.LayoutParams(0, LinearLayout.LayoutParams.WRAP_CONTENT,
                1f);
        int left = index == 0 ? 0 : 4;
        int right = index == count - 1 ? 0 : 4;
        params.setMargins(dp(left), 0, dp(right), 0);
        return params;
    }

    /** Row of equal buttons or fields. */
    LinearLayout rowOf(View... views) {
        LinearLayout row = row();
        for (int i = 0; i < views.length; i++) {
            row.addView(views[i], cell(i, views.length));
        }
        return row;
    }
}
