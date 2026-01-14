/*
 * (c)2026 R van Dorland
 */

#ifndef NETWORK_LOGIC_H
#define NETWORK_LOGIC_H

#include "helpers.h" // Nodig voor u8g2 en uitlijning
#include <Arduino.h>
#include <ArduinoOTA.h>
#include <wifi.h>
#include <ESPmDNS.h>
#include <esp_system.h>

// 2. Het u8g2 object bekend maken bij alle bestanden
// Let op: type moet exact matchen met de constructor in main.cpp
// Het u8g2 object wordt in main.cpp gedefinieerd, we vertellen de compiler dat het bestaat
extern U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2;

extern String sunriseStr;
extern String sunsetStr;
extern String currentTimeStr;
extern String currentDateStr;



/**
 * Toont alleen netwerk informatie bij een "koude" start (stekker erin)
 * Bij een software herstart (na OTA) wordt dit overgeslagen.
 */
// De 'toonNetwerkInfo' functie in network_logic.h
void toonNetwerkInfo()
{
    esp_reset_reason_t reset_reason = esp_reset_reason();

    // We gebruiken de namen die de compiler zojuist zelf voorstelde:
    if (reset_reason == ESP_RST_POWERON || reset_reason == ESP_RST_SW) {
        // ... je code voor het informatiescherm ...
        u8g2.clearBuffer();
        u8g2.drawRFrame(0, 0, LCDWidth, LCDHeight, 5);
        u8g2.setFont(u8g2_font_helvR08_tf);
        u8g2.drawStr(ALIGN_CENTER("SYSTEEM START"), 15, "SYSTEEM START");

        u8g2.setFont(u8g2_font_helvR08_tf);
        u8g2.setCursor(12, 35);
        u8g2.print("IP:   " + WiFi.localIP().toString());
        u8g2.setCursor(12, 48);
        u8g2.print("mDNS: " + String(DEVICE_MDNS_NAME)/* + ".local"*/);

        u8g2.sendBuffer();
        delay(4000);
    }
}

/**
 * WiFi SETUP
 */
void setupWiFi(const char* ssid, const char* password)
{
    WiFi.begin(ssid, password);

    unsigned long startAttemptTime = millis();
    u8g2.setFont(u8g2_font_helvR08_tf);

    while (WiFi.status() != WL_CONNECTED && millis() - startAttemptTime < 15000) {
        const char* Msg = "WiFi Verbinden...";
        u8g2.clearBuffer();
        u8g2.drawRFrame(0, 0, LCDWidth, LCDHeight, 5);
        u8g2.drawStr(ALIGN_CENTER(Msg), ALIGN_V_CENTER, Msg);
        u8g2.sendBuffer();
        delay(500);
    }
}

/**
 * OTA SETUP
 */
void setupOTA(const char* hostname)
{
    ArduinoOTA.setHostname(hostname);

    ArduinoOTA.onStart([]() {
        const char* Msg = "OTA Update Start...";
        u8g2.clearBuffer();
        u8g2.setFont(u8g2_font_helvR08_tf);
        u8g2.drawRFrame(0, 0, LCDWidth, LCDHeight, 5);
        u8g2.drawStr(ALIGN_CENTER(Msg), ALIGN_V_CENTER, Msg);
        u8g2.sendBuffer();
    });

    ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
        const char* Msg = "Bezig met UPDATEN";
        u8g2.clearBuffer();
        u8g2.drawRFrame(0, 0, LCDWidth, LCDHeight, 5);
        u8g2.drawStr(ALIGN_CENTER(Msg), ALIGN_V_CENTER - 10, Msg);
        // Voortgangsbalkje
        unsigned int width = (progress / (total / 100));
        u8g2.drawBox(14, ALIGN_V_CENTER + 5, width, 5);
        u8g2.sendBuffer();
    });

    ArduinoOTA.begin();
    MDNS.begin(hostname);
    MDNS.addService("arduino", "tcp", 8266);
}

#endif
