# Gruzik Robot Android App

Bluetooth Classic controller for the STM32 robot UART parser in `SimpleParser.c`.

## Ready APK

Gotowy debug APK do pobrania jest tutaj:

`apk/GRUZIK4.0-debug.apk`

To jest build debug podpisany standardowym kluczem debug Androida, wystarczajacy do instalacji/testow na telefonie.

Protocol sent to robot:

```text
Kp=0.015\n
Kd=0.55\n
Base_speed=125\n
Max_speed=200\n
Sharp_bend_speed_right=-75\n
Sharp_bend_speed_left=120\n
Bend_speed_right=-75\n
Bend_speed_left=120\n
Treshold=3300\n
Turbine_Speed=0\n
Turbine_Prep_Time=0\n
Mode=Y\n
Mode=N\n

Mapping protocol:
```
Mode=M\n                 # select mapping mode
Mode=U\n                 # select playback/unmapping mode
MapDump=recorded\n       # robot streams GRUZIK.txt as MAP_BEGIN/MAP/MAP_END
MapDump=optimized\n      # robot streams map.txt as MAP_BEGIN/MAP/MAP_END
MapUploadBegin=<count>\n # app starts writing map.txt on SD
MapPoint=<x>,<y>,<pwm>\n # one optimized waypoint
MapUploadEnd=1\n         # app closes map.txt
```
```

Open this folder in Android Studio, let it sync Gradle, then build/install on the phone.
Pair the HC-04/HC-05 module in Android Bluetooth settings first, then connect from the app.
