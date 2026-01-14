#include <Arduino.h>
#include <Wire.h>

#include <helpers.h>
#include <secret.h>

#include "daynight.h"

#include "network_logic.h" // Volgorde is hier erg belangrijk. niet aanpassen!
#include <math.h> // Nodig voor log() berekening

// De U8G2 die je hier aanpast ook in [helpers.h] [network_logic.h], [daynight.cpp] aanpassen!
U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, U8X8_PIN_NONE);

bool eersteStart = true; // Zorgt ervoor dat info éénmalig getoond wordt

String sunriseStr = "--:--";
String sunsetStr = "--:--";
String currentTimeStr = "--:--:--";
String currentDateStr = "--. --:---:----";
String TempC = "0.0";
float tempC = 0.0;
int fanDuty = 0;
int rawValue = 0;
int rpm = 0;
int rpm2 = 0;
int rpm3 = 0;

// De "belofte" aan de compiler dat deze functie verderop staat:
void drawDisplay(struct tm* timeInfo, time_t now);

unsigned long lastBrightnessCheck = 0;
const unsigned long brightnessInterval = 60000; // 1 minuut

// Pin definities
const int ntcPin = 1; // GPIO1 (A1) voor analoge meting
const int fanPwmPin = 10; // GPIO10 voor PWM output naar ventilator
const int fanTachoPin = 2; // GPIO2 voor Tachometer (met 10k pull-up!)
const int fanTachoPin2 = 3; // GPIO3 voor Tachometer (met 10k pull-up!)
const int fanTachoPin3 = 4; // GPIO4 voor Tachometer (met 10k pull-up!)

// --- NTC & DIVIDER PARAMETERS ---
const float seriesResistor = 10000; // Vaste weerstand van 10k
const float nominalResistance = 10000; // NTC weerstand bij 25 graden
const float nominalTemperature = 25; // Nominale temp in Celsius
const float bCoefficient = 3950; // Beta-waarde van de meeste 10k NTC's
const float adcMax = 4095.0; // 12-bit ADC resolutie van de C3

// PWM instellingen voor 4-pin ventilator
const int pwmFreq = 25000; // 25 kHz (industriestandaard)
const int pwmChannel = 0; // Gebruik PWM kanaal 0
const int pwmRes = 10; // 10-bit resolutie (0-1023)

// Tachometer variabelen
volatile int pulseCount = 0;
volatile int pulseCount2 = 0;
volatile int pulseCount3 = 0;
unsigned long lastMillis = 0;
int currentRPM = 0;

// Interrupt Service Routine (moet in RAM voor snelheid)
void IRAM_ATTR countPulses() { pulseCount++; }
void IRAM_ATTR countPulses2() { pulseCount2++; }
void IRAM_ATTR countPulses3() { pulseCount3++; }

// Functie om ADC waarde naar Celsius om te rekenen
float calculateCelsius(int rawADC)
{
    if (rawADC <= 0)
        return 0;
    // Bereken weerstand van de NTC
    float resistance = seriesResistor / (adcMax / (float)rawADC - 1.0);

    // Steinhart-Hart formule (B-parameter vergelijking)
    float steinhart;
    steinhart = resistance / nominalResistance; // (R/Ro)
    steinhart = log(steinhart); // ln(R/Ro)
    steinhart /= bCoefficient; // 1/B * ln(R/Ro)
    steinhart += 1.0 / (nominalTemperature + 273.15); // + (1/To)
    steinhart = 1.0 / steinhart; // Inverteer naar Kelvin
    steinhart -= 273.15; // Kelvin naar Celsius
    return steinhart;
}

/*
 * Setup
 */
void setup()
{
    // Start met een veilige lage snelheid (ca. 10%)
    ledcWrite(pwmChannel, 100);

    Serial.begin(115200);

    // Wacht maximaal 5 seconden tot de seriële monitor is verbonden
    // Dit voorkomt dat je data mist bij het opstarten
    unsigned long startWait = millis();
    while (!Serial && (millis() - startWait < 5000)) {
        delay(10);
    }

    Serial.println("USB verbinding tot stand gebracht!");

    // Configureer PWM kanaal
    ledcSetup(pwmChannel, pwmFreq, pwmRes);
    ledcAttachPin(fanPwmPin, pwmChannel);

    //  Sinds versie 3.x van de ESP32 Arduino Core is de manier waarop PWM (ledc) werkt veranderd:
    //  Oude methode: ledcSetup() en ledcAttachPin().
    //  Nieuwe methode (2025/2026): Gebruik direct ledcAttach(pin, freq, resolution) en daarna ledcWrite(pin, duty).

    // Tachometer setup
    pinMode(fanTachoPin, INPUT_PULLUP); // Extra veiligheid naast externe pull-up
    attachInterrupt(digitalPinToInterrupt(fanTachoPin), countPulses, FALLING);

    pinMode(fanTachoPin2, INPUT_PULLUP); // Extra veiligheid naast externe pull-up
    attachInterrupt(digitalPinToInterrupt(fanTachoPin2), countPulses2, FALLING);

    pinMode(fanTachoPin3, INPUT_PULLUP); // Extra veiligheid naast externe pull-up
    attachInterrupt(digitalPinToInterrupt(fanTachoPin3), countPulses3, FALLING);

    // 1. Hardware basis
    Serial.begin(115200);
    Wire.begin(I2C_SDA, I2C_SCL);
    u8g2.begin();
    u8g2.setContrast(10);

    // 2. Netwerk (nu lekker kort!)
    setupWiFi(SECRET_SSID, SECRET_PASS);

    // fetchWeather(); // Haal direct het eerste weerbericht op

    if (WiFi.status() == WL_CONNECTED) {
        toonNetwerkInfo(); // Deze functie bevat de 'rtc_info->reason' check
    }

    setupOTA(DEVICE_MDNS_NAME);

    // 3. Tijd en Regeling
    configTzTime(SECRET_TZ_INFO, SECRET_NTP_SERVER);

    // Initialiseer eerste waarden
    manageBrightness();

    // 1. Lees NTC en stuur fan aan
    rawValue = analogRead(ntcPin);
    tempC = calculateCelsius(rawValue);

    // Succes schermpje (optioneel)
    const char* Msg = "Systeem Online";
    u8g2.clearBuffer();
    u8g2.drawRFrame(0, 0, LCDWidth, LCDHeight, 5);
    u8g2.drawStr(ALIGN_CENTER(Msg), ALIGN_V_CENTER, Msg);
    // u8g2.print(WiFi.localIP().toString());
    u8g2.sendBuffer();
    delay(2000);
}

/*
 *loop
 */
void loop()
{
    // 1. Altijd als eerste: Onderhoud voor OTA en Netwerk
    ArduinoOTA.handle();

    unsigned long currentMillis = millis();

    // 2. Weer-update timer (elke 15 minuten = 900.000 ms)
    static unsigned long lastNTC_Update = 0;
    const unsigned long NTC_Interval = 900000;

    if (currentMillis - lastNTC_Update >= NTC_Interval || lastNTC_Update == 0) {
        lastNTC_Update = currentMillis;
        // Lees NTC en stuur fan aan
        rawValue = analogRead(ntcPin);
        tempC = calculateCelsius(rawValue);
    }

    // 3. Display en Tijd update timer (elke seconde = 1000 ms)
    static unsigned long lastDisplayUpdate = 0;
    if (currentMillis - lastDisplayUpdate >= 1000) {
        lastDisplayUpdate = currentMillis;

        time_t now = time(nullptr);
        struct tm* timeInfo = localtime(&now);

        // Alleen actie ondernemen als we een geldige tijd hebben (na 1 jan 1970)
        if (now > 100000) {
            // Update de tijds-strings (Zo. 12 Jan, etc.)
            updateDateTimeStrings(timeInfo);

            // Controleer elke minuut de helderheid/zonnestand
            static unsigned long lastBrightnessCheck = 0;
            if (currentMillis - lastBrightnessCheck >= 60000) {
                lastBrightnessCheck = currentMillis;
                manageBrightness();
            }

            // Teken alles op het scherm (Klok, Datum, Iconen en Weer)
            drawDisplay(timeInfo, now);
        }
    }
    // timeClient.update(); // Update time from NTP (if needed)
    // unsigned long currentTime = timeClient.getEpochTime(); // Get current Unix time

    // // Display current time (every second)

    // Aansturing (ADC): Warmer = lagere ADC = hogere PWM
    fanDuty = map(rawValue, 1800, 800, 0, 1023); // Pas deze waarden aan op jouw NTC en ventilator 2200 = koud, 1200 = warm
    fanDuty = constrain(fanDuty, 0, 1023);
    ledcWrite(pwmChannel, fanDuty);

    // RPM berekening
    if (millis() - lastMillis >= 1000) {
        noInterrupts();
        rpm = (pulseCount / 2.0) * 60.0;
        rpm2 = (pulseCount2 / 2.0) * 60.0;
        rpm3 = (pulseCount3 / 2.0) * 60.0;
        pulseCount = 0;
        pulseCount2 = 0;
        pulseCount3 = 0;
        interrupts();
        lastMillis = millis();
    }
}

void drawDisplay(struct tm* timeInfo, time_t now)
{
    u8g2.clearBuffer();
    u8g2.enableUTF8Print();

    // --- 1. Bovenste balk: Iconen & Datum ---
    bool timeValid = (now > 1735689600);
    long ntpIcon = timeValid ? 57367 : 57368;
    long rssiIcon = map(WiFi.RSSI(), -90, -30, 57949, 57953);

    u8g2.setFont(u8g2_font_waffle_t_all); // Zorg dat dit font actief is
    u8g2.drawGlyph(0, 10, ntpIcon); // Y iets verlaagd naar 10 voor betere weergave
    u8g2.drawGlyph(14, 10, rssiIcon); // X iets meer ruimte gegeven

    // Header
    u8g2.setFont(u8g2_font_6x12_tr);
    u8g2.drawStr(32, 10, "RADIATOR MONITOR");
    u8g2.drawLine(0, 13, 128, 13);

    // Grote Temperatuur Weergave
    u8g2.setFont(u8g2_font_helvB18_tf);
    u8g2.setCursor(0, 40);
    u8g2.print(tempC, 1); // Toon 1 decimaal
    u8g2.print("°C"); // Dankzij enableUTF8Print()

    // RPM Rechtsboven
    u8g2.setFont(u8g2_font_6x10_tr);
    u8g2.setCursor(80, 28);
    u8g2.print(rpm);
    u8g2.setCursor(80, 40);
    u8g2.print(rpm2);
    u8g2.setCursor(80, 52);
    u8g2.print(rpm3);

    // PWM Balk onderin
    int pwmPercent = map(fanDuty, 0, 1023, 0, 100);
    u8g2.drawFrame(0, 56, 128, 8);
    u8g2.drawBox(2, 58, map(pwmPercent, 0, 100, 0, 124), 4);
    u8g2.setFont(u8g2_font_4x6_tr);
    u8g2.drawStr(0, 53, "FAN POWER:");
    u8g2.setCursor(110, 53);
    u8g2.print(pwmPercent);
    u8g2.print("%");

    u8g2.sendBuffer();
    delay(200);

    // 3. Output naar Seriële Monitor
    Serial.print("NTC: ");
    Serial.print(rawValue);
    Serial.print(" | PWM: ");
    Serial.print((fanDuty / 1023.0) * 100);
    Serial.print("% | Fan Snelheid: ");
    Serial.print(rpm);
    Serial.println(" RPM");

    delay(2000); // Update elke 2 seconden
    u8g2.sendBuffer();
}
