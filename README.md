# GRUZIK4.0

GRUZIK4.0 to szybki robot line follower oparty o STM32. Projekt laczy klasyczna jazde po czujniku linii z odometria z enkoderow, IMU, mapowaniem trasy na karte SD i pozniejszym odtwarzaniem zoptymalizowanej trasy.

Kod STM32 znajduje sie w:

`STM32 Software/Gruzik40LFLCode`

Najwazniejsze pliki:

- `Core/Src/Line_Follower.c` - normalna jazda po linii i PID.
- `Core/Src/odometry.c` - fuzja enkoderow i IMU.
- `Core/Src/map.c` - mapowanie oraz odtwarzanie trasy.
- `Core/Src/SimpleParser.c` - komendy Bluetooth z aplikacji.
- `Core/Inc/robot_config.h` - stale robota, wymiary, limity i parametry algorytmow.

## Czujniki

Robot korzysta z kilku zrodel informacji naraz:

- czujniki linii mowia, gdzie jest linia pod robotem,
- enkodery mowia, ile obrocily sie kola,
- IMU mierzy obrot robota wokol osi Z,
- karta SD zapisuje i odczytuje trase,
- aplikacja przez Bluetooth wysyla nastawy, tryby pracy i mapy.

Kazdy sensor ma inny problem. Czujnik linii jest szybki, ale gdy robot wyjedzie poza linie, nie wie gdzie jest. Enkodery dobrze mierza dystans, ale slizg kol psuje kat. IMU dobrze widzi obrot, ale z czasem dryfuje. Dlatego robot nie opiera sie tylko na jednym z nich.

## Fuzja IMU i enkoderow

Fuzja jest w `odometry.c`.

Enkodery licza droge lewego i prawego kola. Z tego robot wylicza:

- sredni przejechany dystans,
- zmiane kata z roznicy drogi kol.

IMU mierzy predkosc obrotu `gyro_z`, ktora jest zamieniana na zmiane kata w czasie jednej petli sterowania.

Nastepnie robot miesza oba katy:

```c
fused_delta_yaw = gyro_weight * delta_yaw_gyro
                + (1.0f - gyro_weight) * delta_yaw_encoder;
```

Aktualnie `gyro_weight` jest ustawione w `robot_config.h` na `0.96`. To znaczy, ze kat robota jest mocno oparty o IMU, a enkodery tylko lekko go koryguja. Dystans dalej pochodzi z enkoderow, bo IMU nie mierzy przesuniecia robota po trasie.

Po policzeniu nowego kata robot aktualizuje pozycje:

- `x`,
- `y`,
- `yaw`.

Pozycja jest liczona w metrach, a kat w radianach. Dzieki temu mapa nie jest tylko lista bledow z linii, ale realna sciezka w ukladzie wspolrzednych robota.

## Normalna jazda po linii

Normalny tryb jazdy jest w `Line_Follower.c`.

Czujniki linii zwracaja pozycje linii. Srodek czujnika jest traktowany jako wartosc okolo `6500`. Blad jest liczony tak:

```c
error = 6500 - position;
```

Potem dziala klasyczny regulator:

- `P` reaguje na aktualny blad,
- `D` reaguje na szybka zmiane bledu,
- wynik zmienia predkosc lewego i prawego silnika.

Ten tryb jest najlepszy do zwyklej jazdy, bo robot caly czas patrzy na fizyczna linie, a nie na zapisane wspolrzedne.

## Mapowanie trasy

Mapowanie jest w `map.c`.

Po starcie mapowania robot:

1. resetuje odometrie,
2. otwiera plik `GRUZIK.txt` na karcie SD,
3. jedzie normalnie po linii PID-em,
4. co kilka cykli zapisuje punkt trasy.

Zapisywany punkt zawiera m.in.:

- pozycje robota `x/y`,
- pozycje przodu z czujnikami linii,
- predkosc,
- blad `P` i `D` z czujnika linii.

Mapa konczy sie automatycznie, gdy robot po przejechaniu minimalnego dystansu wroci blisko startu. Promien zamkniecia jest ustawiony na `3 cm`:

```c
ROBOT_MAP_CLOSE_RADIUS_M 0.030f
```

Dzieki temu robot nie musi mapowac idealnie do recznego stopu. Gdy znajdzie sie blisko startu, firmware domyka zapis i zatrzymuje mapowanie.

## Optymalizacja mapy w aplikacji

Aplikacja pobiera `GRUZIK.txt`, pokazuje zapisana trase i tworzy z niej `map.txt`.

Optymalizacja robi trzy glowne rzeczy:

1. wybiera co ktory punkt ma trafic do mapy,
2. wygladza trase,
3. nadaje predkosci dla punktow.

Predkosc nie jest juz liczona z odleglosci miedzy punktami, bo przy gestych punktach robot nie dochodzil do zadanej predkosci. Teraz aplikacja patrzy na zakret:

- proste odcinki ida blizej `Max speed`,
- zakrety zwalniaja zaleznie od `Gain`,
- `Min speed` ogranicza minimalna predkosc.

Wazne: zoptymalizowana mapa nie dopisuje na koncu punktu startowego. Po ostatnim punkcie robot nie zawraca sztucznie do poczatku mapy. Firmware przechodzi wtedy na zwykly PID i po chwili zatrzymuje robota.

## Odtwarzanie trasy

Odtwarzanie jest w `DriveOnMap()` w `map.c`.

Robot czyta `map.txt` z karty SD. Kazda linia ma:

```text
x y speed
```

Robot nie goni juz pojedynczego punktu agresywnie. Zamiast tego traktuje trase jak segmenty:

- ma poprzedni punkt,
- ma aktualny punkt docelowy,
- liczy kierunek segmentu,
- liczy blad boczny wzgledem segmentu,
- uzywa lookahead, zeby nie skrecac nerwowo.

Regulator odtwarzania korzysta z `MapP`, `MapI`, `MapD`.

- `MapP` odpowiada za reakcje na blad kierunku/pozycji.
- `MapI` zwykle powinien zostac `0`, bo moze latwo rozbujac robota.
- `MapD` tlumi szybkie zmiany, ale jest filtrowany, zeby nie wzmacniac szumu.

Korekta jest limitowana, zeby przy wiekszej predkosci robot nie robil zbyt ostrych ruchow silnikami. Dodatkowo predkosc jest lekko zmniejszana, gdy blad kierunku jest duzy.

## Rola czujnika linii podczas odtwarzania

Podczas odtwarzania glowna nawigacja idzie z odometrii, czyli z IMU i enkoderow. Czujnik linii jest tylko dodatkowa korekta.

Kod liczy pewnosc linii:

- jesli linia jest widoczna i blisko srodka, korekta z czujnika linii pomaga,
- jesli linia jest zgubiona, korekta znika,
- jesli czujnik widzi za duzo aktywnych pol, zaufanie spada.

Dzieki temu robot nie wariuje od razu, gdy czujnik lekko wyjedzie poza linie, ale dalej moze sie delikatnie kalibrowac do prawdziwej trasy.

## Koniec odtwarzania

Gdy robot przejedzie ostatni punkt z `map.txt`, firmware konczy odtwarzanie mapy i wraca do zwyklego PID-a. Potem po krotkim czasie zatrzymuje robota.

To jest celowe: ostatni fragment robot moze dojechac po fizycznej linii, zamiast wracac po sztucznym odcinku do punktu startowego.

## Tryby z aplikacji

Aplikacja Android ma kilka zakladek:

- `Drive` - normalna jazda i nastawy PID.
- `Mapping` - mapowanie, pobieranie mapy, optymalizacja i upload `map.txt`.
- `Joystick` - reczne sterowanie robotem po Bluetooth.
- `Odometry` - podglad pozycji, kata, predkosci kol i fuzji.
- `Debug` - podglad czujnikow linii, enkoderow i IMU.

Joystick wysyla reczne predkosci lewego i prawego kola. Po puszczeniu palca wysylane jest zero. Firmware ma tez timeout, wiec gdy pakiety Bluetooth przestana przychodzic, robot sam zatrzyma tryb reczny.

## Najwazniejsze komendy Bluetooth

Przyklady:

```text
StartNormal=1
StartMapping=1
StartPlayback=1
Mode=N
Telemetry=odom
Telemetry=debug
MapDump=recorded
MapUploadBegin=<count>
MapPoint=<x>,<y>,<speed>
MapUploadEnd=1
Manual=<left_pwm>,<right_pwm>
CleanSpeed=170
Clean=1
Clean=0
```

## Strojenie

Typowe punkty startowe:

- normalny PID: stroic `Kp`, `Kd`, `Base_speed`, `Max_speed`,
- odtwarzanie mapy: zaczac od `MapP` okolo `65-80`, `MapD` okolo `4-8`, `MapI = 0`,
- optymalizacja: `Max speed` ustawia predkosc na prostych, `Gain` mocniej zwalnia zakrety.

Jezeli robot oscyluje przy odtwarzaniu:

- zmniejszyc `MapP`,
- zmniejszyc `MapD`, jesli robot szarpie,
- zmniejszyc `Max speed`,
- zwiekszyc `Gain`, jesli za szybko wchodzi w zakrety.

Jezeli robot nie trzyma trasy:

- sprawdzic kierunek enkoderow,
- sprawdzic yaw z IMU,
- sprawdzic, czy mapa zostala nagrana po poprawnym okrazeniu,
- pobrac nowa mape i ponownie zrobic `Optimize` oraz `Run map`.

## Budowanie

Projekt STM32 jest przygotowany pod STM32CubeIDE.

Androidowa aplikacja jest teraz w repo:

`Android App/RobotApp`

Firmware do robota musi byc wgrany przez ST-Link / STM32CubeProgrammer. Aplikacje Android mozna zbudowac Gradle i wgrac przez ADB.
