# NYDER LF (Android)

Aplikacja Bluetooth do line followerow GRUZIK4.0 i REGIN V3. Wyglad i uklad jak w aplikacji REGIN V3, protokol tekstowy z `SimpleParser.c`.

## Gotowy APK

`apk/GRUZIK4.0-debug.apk` (build debug, podpisany kluczem debug Androida). Zastepuje poprzednia aplikacje GRUZIK4.0 (ten sam `applicationId`).

## Rozpoznanie robota

Po polaczeniu aplikacja wysyla `Info=1`. Firmware GRUZIK4.0 v2 odpowiada `INFO,...` z lista mozliwosci. Starsze firmware i REGIN V3 sa rozpoznawane po nazwie modulu Bluetooth. W `Settings` mozna wymusic typ robota. Nastawy i presety sa trzymane osobno dla kazdego robota.

## Zakladki

- `Drive` - PID, predkosci, opcje zakretow 90 stopni (GRUZIK v2), turbina (REGIN), presety, czyszczenie opon. `Start` i `Stop` sa zawsze na gorze.
- `Sensors` - slupki czujnikow od lewej do prawej, prog, pozycja linii, blad i czestotliwosc odswiezania.
- `Mapping` - trasa na zywo, mapa predkosci, wykres profilu, nastawy, kalibracja zyroskopu, stary tryb trasy po odometrii (`map.txt`).
- `Joystick`, `Odometry`, `Log`, `Settings`.

Telemetria jest jedna naraz i zalezy od otwartej zakladki (9600 bodow to ok. 960 B/s).

## Budowanie

```bash
./gradlew assembleDebug
```

Gradle potrzebuje JDK 17+, np. tego z Android Studio:

```bash
export JAVA_HOME="/Applications/Android Studio.app/Contents/jbr/Contents/Home"
```
