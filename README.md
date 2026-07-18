====================================================================
 ESP32 WATER QUALITY MONITORING SYSTEM — README
 Sketch: ESP32project_with_blynk_remodification1_0.ino  (v5.0)
====================================================================

WHAT THIS PROJECT DOES
--------------------------------------------------------------------
This sketch turns an ESP32 into a water quality monitor. It reads
four things from the water:

  - Temperature   (DS18B20 sensor)
  - TDS           (Total Dissolved Solids, in ppm)
  - pH
  - Turbidity     (cloudiness, in NTU)

It compares each reading against WHO / EPA / EU / ISO 7027 safe
ranges, decides an overall status (GOOD / WARNING / CRITICAL),
lights the matching LED, sounds a buzzer if something is off, and
sends all the data to a Blynk dashboard on your phone every 3
seconds. Everything is also printed to the Serial Monitor.


WHAT YOU NEED
--------------------------------------------------------------------
Hardware:
  - ESP32 dev board
  - DS18B20 temperature sensor + 4.7k ohm pull-up resistor
  - TDS sensor
  - pH sensor
  - Turbidity sensor
  - Buzzer
  - 3x LEDs (green, yellow, red) + 220 ohm resistors each
  - (Optional) 16x2 I2C LCD display

Software (Arduino IDE Library Manager):
  - OneWire              (by Paul Stoffregen)
  - DallasTemperature     (by Miles Burton)
  - LiquidCrystal_I2C     (by Frank de Brabander)  -- only if using LCD
  - Blynk                 (by Volodymyr Shymanskyy) -- install "Blynk"
    v1.3.x, this is the ESP32-compatible Blynk IoT library


WIRING / PIN MAP
--------------------------------------------------------------------
  DS18B20 data        -> GPIO 4   (+ 4.7k pull-up to 3.3V)
  TDS sensor output    -> GPIO 34
  pH sensor output     -> GPIO 35
  Turbidity output     -> GPIO 32
  Buzzer (+)           -> GPIO 25
  Green LED            -> GPIO 26
  Yellow LED           -> GPIO 27
  Red LED              -> GPIO 14
  LCD (if enabled)     -> SDA=GPIO21, SCL=GPIO22


HOW TO SET IT UP
--------------------------------------------------------------------
1. Install the libraries listed above in the Arduino IDE.

2. Open the sketch and edit these three lines near the top with
   YOUR OWN Blynk template info (from the Blynk IoT app/console):

       #define BLYNK_TEMPLATE_ID   "..."
       #define BLYNK_TEMPLATE_NAME "..."
       #define BLYNK_AUTH_TOKEN    "..."

   IMPORTANT: these three lines must stay ABOVE the
   #include <BlynkSimpleEsp32.h> line, and in this order, or the
   Blynk connection will fail silently.

3. Enter your WiFi network name and password:

       char ssid[] = "YourWiFiName";
       char pass[] = "YourWiFiPassword";

4. In the Blynk console, create a template for an ESP32 device and
   add 5 datastreams:
       V0 = Temperature (double)
       V1 = TDS (double)
       V2 = pH (double)
       V3 = Turbidity (double)
       V4 = Water status, 0/1/2 (integer)
   Then add gauge/value widgets for V0-V3 and a label/LED for V4
   on your dashboard.

5. Wire the hardware according to the pin map above.

6. Select your ESP32 board and COM port in the Arduino IDE, then
   upload the sketch.


IMPORTANT: SERIAL MONITOR BAUD RATE
--------------------------------------------------------------------
The sketch opens the serial connection with:

       Serial.begin(115200);

This means the Serial Monitor in the Arduino IDE MUST be set to
match that same speed, or the text will show up as garbled
characters (or nothing at all).

To set it correctly:
  1. Open Tools > Serial Monitor (or the Serial Monitor window).
  2. In the baud-rate dropdown in the bottom-right corner, select
     115200 baud.

If you ever change the Serial.begin(...) value in the code (for
example to 9600), you must change the Serial Monitor dropdown to
that exact same number, or the two will be out of sync and the
output will look like nonsense text.

Note: baud rates are standard fixed values (9600, 19200, 38400,
57600, 115200, etc.) - there is no "115000" standard baud rate.
If your intention was to use 115200 (the value already used in
this sketch), you don't need to change anything in the code -
just make sure the Serial Monitor dropdown is also set to 115200.


CALIBRATION (do this before trusting the readings)
--------------------------------------------------------------------
pH:
  1. Set DEBUG_PH to true near the top of the sketch.
  2. Dip the probe in pH 7 buffer solution, note the mV value
     printed to Serial, put it into PH_MV_AT_7.
  3. Dip the probe in pH 4 buffer solution, note the mV value,
     put it into PH_MV_AT_4.
  4. Set DEBUG_PH back to false.

TDS:
  Adjust TDS_FACTOR using a solution of known ppm until the
  reading matches.

Turbidity (most important to calibrate):
  1. Set DEBUG_TURBIDITY to true.
  2. Dip the sensor in clean water and note the mV value shown.
  3. Put that value into TURB_CLEAR_MV.
  4. Set DEBUG_TURBIDITY back to false.
  5. If your particular sensor reads a HIGHER voltage in dirty
     water (instead of lower), set TURB_INVERTED to true.


OPTIONAL SETTINGS
--------------------------------------------------------------------
  USE_LCD   -> set to true if you have the 16x2 I2C LCD wired up.
  USE_ADC_CAL -> leave as true; uses the ESP32's factory ADC
                 calibration for more accurate voltage readings.


HOW IT BEHAVES WHEN RUNNING
--------------------------------------------------------------------
  - On power-up, all 3 LEDs flash briefly as a wiring test.
  - It connects to WiFi, then to the Blynk cloud (this can take
    a few seconds - if it hangs, double check ssid/pass and that
    your BLYNK_AUTH_TOKEN is correct).
  - Every 3 seconds it reads all sensors, updates the LED that
    matches the current status, sounds the buzzer if there's a
    WARNING or CRITICAL result, sends the values to your Blynk
    dashboard, and prints a full report box to the Serial Monitor.
  - GOOD = green LED, no buzzer.
  - WARNING = yellow LED, 2 short buzzer beeps.
  - CRITICAL = red LED, 5 quick buzzer beeps.


TROUBLESHOOTING
--------------------------------------------------------------------
  - Garbled text in Serial Monitor -> Serial Monitor baud rate
    doesn't match Serial.begin(115200) in the code. Set it to
    115200.
  - "DS18B20 not found" warning -> check GPIO4 wiring and that
    the 4.7k pull-up resistor goes to 3.3V, not 5V.
  - Blynk never connects -> check WiFi name/password, and confirm
    the BLYNK_TEMPLATE_ID / NAME / AUTH_TOKEN lines are correct
    and placed before the #include line.
  - Turbidity always reads 0 -> recalibrate TURB_CLEAR_MV in
    clean water (see Calibration section above).
====================================================================
