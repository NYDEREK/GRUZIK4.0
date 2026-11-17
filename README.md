# GRUZIK4.0

GRUZIK4.0 to szybki robot line follower oparty o STM32. Projekt laczy klasyczna jazde po czujniku linii z odometria z enkoderow, IMU, mapowaniem trasy na karte SD i pozniejszym odtwarzaniem zoptymalizowanej trasy.

Kod STM32 znajduje sie w:

`STM32 Software/Gruzik40LFLCode`

Najwazniejsze pliki:

- `Core/Src/Line_Follower.c` - normalna jazda po linii i PID.
- `Core/Src/odometry.c` - fuzja enkoderow i IMU.
- `Core/Src/map.c` - mapowanie oraz odtwarzanie trasy po odometrii.
- `Core/Src/speed_map.c` - mapa predkosci (nagrywanie, profil, jazda, synchronizacja dystansu).
- `Core/Src/telemetry.c` - nieblokujace wysylanie po UART i ramki telemetrii dla aplikacji.
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

Aktualnie `gyro_weight` jest ustawione na `1.0`, czyli kat pochodzi tylko z IMU. Przy `0.96` do kata przeciekalo 4% bledu enkoderow, a enkodery w zakretach klamia przez poslizg kol. Wage mozna zmienic z aplikacji (`GyroWeight=`). Dystans dalej pochodzi z enkoderow, bo IMU nie mierzy przesuniecia robota po trasie.

Zyroskop jest calkowany z prawdziwym czasem miedzy tickami (licznik cykli DWT), a nie z zalozonym `1 ms`, i mnozony przez `GyroScale` (kalibracja skali, opis nizej).

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

### Przerwy w linii

Normalny PID ma dodatkowy tryb przejazdu przez krotka przerwe w tasmie. Robot uzbraja ten tryb tylko wtedy, gdy ostatni odczyt byl bezpieczny:

- linia byla widoczna,
- blad byl blisko srodka listwy,
- skrajne sensory nie byly aktywne.

Jesli zaraz po takim odczycie wszystkie sensory przestana widziec linie, robot nie robi od razu sharp turn. Przez maksymalnie `4000 ms` jedzie ostatnimi poprawnymi komendami silnikow. Gdy linia wroci, bridge natychmiast sie wylacza i PID dziala normalnie. Jesli linia nie wroci przez ten czas, robot wraca do standardowego odzyskiwania linii na podstawie `Last_end`.

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

## Drift odometrii - co bylo nie tak i co jest poprawione

1. **Karta SD w przerwaniu 1 kHz.** `MapUpdate()` zapisywal `GRUZIK.txt` (`f_puts`) wewnatrz przerwania TIM20, a `DriveOnMap()` czytal `map.txt` bajt po bajcie tez w przerwaniu. Zapis sektora na karte trwa kilka ms, czasem kilkadziesiat-kilkaset ms. Wszystkie przerwania maja priorytet 0, wiec w tym czasie stal tez PID (TIM5), a calka zyroskopu gubila caly ten obrot - kat skakal. Teraz przerwanie tylko wrzuca rekord do kolejki w RAM, a zapis robi petla glowna. Odtwarzanie wczytuje cale `map.txt` do RAM przed startem.
2. **Staly `dt`.** Kat byl liczony jako `gyro * 0.001 s`, nawet gdy tick przyszedl pozniej albo przepadl. Teraz `dt` jest mierzony licznikiem cykli procesora.
3. **Skala zyroskopu.** Typowy blad skali ICM-42688 to 0.5-1%. Na trasie z 10 pelnymi obrotami to 18-36 stopni bledu kata. Po domknietym okrazeniu mapowania robot zna faktyczny obrot (wielokrotnosc 360 stopni) i odsyla `GYRO_SCALE,<sugestia>`. Aplikacja pokazuje przycisk `Apply`, a wartosc idzie do robota przy kazdym starcie (`GyroScale=`).
4. **Bias zyroskopu.** Bias uczyl sie tylko przy wlaczonej telemetrii. Teraz uczy sie zawsze, gdy robot stoi.
5. **Domykanie okrazenia.** Przy dryfie robot rzadko wraca w promien 3 cm od startu, wiec mapowanie sie nie konczylo. Teraz, jesli po min. 1.5 m jest w promieniu 25 cm od startu i z katem zblizonym do startowego, okrazenie zamyka sie w punkcie najblizszym startu.
6. **Telemetria blokowala petle glowna.** `HAL_UART_Transmit` przy 9600 bodow (ok. 960 B/s) wysylal ODOM+DBG (ok. 200 B) co 100 ms, wiec petla stala. Teraz strumienie ida przez przerwanie UART, a ramka jest pomijana, gdy poprzednia jeszcze leci.

## Mapa predkosci

Robot zawsze jedzie po fizycznej linii zwyklym PID-em, wiec nie scina zakretow. Z mapy bierze tylko predkosc bazowa, zalezna od miejsca na trasie (przejechanego dystansu).

**Mapowanie** (`StartMapping=1`): co 1 cm przejechanego dystansu zapisywany jest kat robota (RAM, do 50 m trasy). Rownolegle dalej powstaje `GRUZIK.txt` dla starego trybu. Po domknieciu okrazenia albo `Stop` mapa jest zapisywana na karte jako `speedmap.bin` i wczytywana po wlaczeniu robota. Bez karty SD mapa predkosci dziala do wylaczenia zasilania.

**Profil** (budowany w robocie przed startem i w aplikacji jako podglad, ta sama matematyka):

- krzywizna `k(s)` = zmiana kata na oknie `Curvature window` / dlugosc okna,
- predkosc w zakrecie `v = sqrt(Corner grip / |k|)`, ograniczona do `Min speed`..`Max speed`,
- hamowanie PRZED zakretem (przebieg od konca): `v(s) <= sqrt(v(s+ds)^2 + 2 * Braking * ds)`,
- przyspieszanie na prostych (przebieg od poczatku): `v(s) <= sqrt(v(s-ds)^2 + 2 * Acceleration * ds)`.

`Braking` i `Acceleration` reguluja sie osobno. Mniejsze `Braking` = robot zaczyna hamowac wczesniej.

**Jazda** (`StartSpeedRun=1`): PID steruje jak zwykle, a predkosc bazowa (PWM) daje petla predkosci `base = FF * v_cel + Kp * (v_cel - v_zmierzona)`. `FF` (PWM na m/s) robot uczy sie sam na prostych podczas mapowania, mozna tez wpisac recznie. Cel jest brany z przodu o `predkosc * Response lag` (najmniejsza predkosc na tym odcinku), zeby silniki zdazyly zahamowac. Ujemna baza = aktywne hamowanie.

**Synchronizacja dystansu.** Poslizg kol sprawia, ze przejechany dystans z enkoderow ucieka od mapy (3% poslizgu = 30 cm na 10 m). Ten sam detektor wyjscia z zakretu dziala przy mapowaniu i w jezdzie. Kazde wyjscie z zakretu w jezdzie dociaga pozycje do pasujacego wyjscia na mapie (ten sam kierunek, podobny kat, w oknie odleglosci), a z kolejnych korekt robot uczy sie sredniego poslizgu. W symulacji z 3% poslizgiem blad pozycji na mapie spadl z ok. 44 cm do 4.5 cm na koniec 2 okrazenia.

`Laps` > 0 konczy jazde po tylu okrazeniach (dalej zwykly PID i stop po 1.75 s). Na mapie otwartej (zatrzymanej recznie) jazda konczy sie na koncu mapy.

## Zakrety 90 stopni na zwyklym PID

Zwykly PID przestrzeliwuje ostry zakret, bo linia ucieka za krawedz listwy, zanim korekta zdazy obrocic robota, a potem `sharp_turn` zawraca dopiero po zgubieniu linii. W firmware sa trzy opcje (domyslnie wylaczone, zakladka Drive, sekcja Corners):

- `ErrSlow` (0..1) - zwalnia baze o `ErrSlow * (blad / 5500)^2`. Na prostej prawie nic nie zmienia, przy linii na krawedzi daje robotowi czas na skret. Start: `0.4`.
- `CornerLatch=1` z `CornerIn` / `CornerOut` - gdy blad dojdzie do `CornerIn` (domyslnie 5000, czyli linia na skrajnych czujnikach), robot od razu obraca sie predkosciami `Sharp bend` i trzyma to, az linia wroci ponizej `CornerOut` (domyslnie 1500). Nie czeka na zgubienie linii i nie puszcza, gdy linia tylko musnie skrajny czujnik - to daje "przyklejenie" w zakrecie 90 stopni.
- `DWin` (1..9) - czlon D liczony z bledu sprzed `DWin` cykli. Pozycja z progowanych czujnikow skacze co 500-1000, wiec D z jednego cyklu (0.5 ms) to glownie szpilki. `DWin=4` daje gladsze, przewidywalne hamowanie skretu. Przy zmianie `DWin` trzeba ponownie dobrac `Kd`.

Dalsze kroki, ktore warto rozwazyc: pozycja linii liczona analogowo (srednia wazona wartosciami ADC po kalibracji min/max kazdego czujnika zamiast progu) - blad staje sie ciagly i D przestaje szumiec; oraz mapa predkosci, ktora hamuje przed zakretem zanim linia w ogole dojdzie do krawedzi.

## Tryby z aplikacji

Aplikacja Android (`Android App/RobotApp`, nazwa `NYDER LF`) dziala z GRUZIK4.0 i REGIN V3. Po polaczeniu pyta robota `Info=1`, starsze firmware rozpoznaje po nazwie Bluetooth. Zakladki:

- `Drive` - PID, predkosci, opcje zakretow (GRUZIK), turbina (REGIN), presety, czyszczenie opon.
- `Sensors` - slupki wszystkich czujnikow (12 albo 16) z wartosciami ADC, progiem i pozycja linii. GRUZIK wysyla zwarta ramke `S` ok. 20 razy na sekunde (wczesniej ok. 4).
- `Mapping` - trasa na zywo podczas mapowania, pozycja robota na mapie podczas jazdy, mapa predkosci (kolor = predkosc), wykres profilu z rzeczywista predkoscia, nastawy mapy, kalibracja zyroskopu i stary tryb trasy po odometrii.
- `Joystick` - reczne sterowanie robotem po Bluetooth.
- `Odometry` - pozycja, kat, predkosci kol, zyroskop, kat z enkoderow vs z zyroskopu.
- `Log` i `Settings` (typ robota, zmiana nazwy modulu).

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
Info=1                     -> INFO,GRUZIK4.0,<fw>,<czujniki>,<mozliwosci>,<bateria_mV>,<probki mapy>
Telemetry=sensors          -> S,<pos>,<aktywne>,<last_end>,<12 x 3 hex, od lewej>,<bateria_mV>
Telemetry=live             -> L,<s_mm>,<x_mm>,<y_mm>,<yaw_cdeg>,<v_mm_s>,<v_cel_mm_s>,<stan>,<okrazenie>,<sync_mm>
StartSpeedRun=1
SpeedMapBuild=1            -> SPEEDMAP_READY,<probki>,<dlugosc_mm>,<okrazenie_ms>,<v_max_mm_s>,<FF_x10>,<zakrety>,<zamknieta>
SpeedMapDump=1             -> SM_BEGIN / SMH,<indeks>,<kat_mrad>,<przyrosty...> / SM_END
SpeedMapClear=1
SmVmax= SmVmin= SmAcc= SmBrake= SmLat= SmLag=<ms> SmWin=<cm> SmKp= SmFF= SmLaps=
GyroScale= GyroWeight=
ErrSlow= CornerLatch= CornerIn= CornerOut= DWin=
Manual=<left_pwm>,<right_pwm>
CleanSpeed=170
Clean=1
Clean=0
```

## Strojenie

Typowe punkty startowe:

- normalny PID: stroic `Kp`, `Kd`, `Base_speed`, `Max_speed`,
- odtwarzanie mapy: zaczac od `MapP` okolo `65-80`, `MapD` okolo `4-8`, `MapI = 0`,
- mapa predkosci: najpierw `Max speed` nisko (np. 1.5 m/s), `Corner grip` 5-7, `Braking` 6-8. Na wykresie w aplikacji pomaranczowa linia (zmierzona predkosc) powinna isc za biala (cel). Jesli robot dojezdza do zakretu za szybko - zmniejszyc `Braking` albo zwiekszyc `Response lag`. Jesli wylatuje w zakrecie - zmniejszyc `Corner grip`. Kolumna `Sync, mm` pokazuje ostatnia korekte dystansu, stale duze wartosci (>100 mm) oznaczaja duzy poslizg albo zle `GyroScale`,
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

Nowe pliki `speed_map.c` i `telemetry.c` STM32CubeIDE dolacza sam (managed build). Firmware do robota musi byc wgrany przez ST-Link / STM32CubeProgrammer.

Aplikacja: `cd "Android App/RobotApp" && ./gradlew assembleDebug` (JDK 17+, np. `export JAVA_HOME="/Applications/Android Studio.app/Contents/jbr/Contents/Home"`). Gotowy APK: `Android App/RobotApp/apk/GRUZIK4.0-debug.apk`.
