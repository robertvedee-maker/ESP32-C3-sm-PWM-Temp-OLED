/*
 * (c)2026 R van Dorland
 */

#ifndef DAY_NIGHT_H
#define DAY_NIGHT_H

#include "secret.h"
#include <Arduino.h>
#include <SolarCalculator.h>
#include <U8g2lib.h> // Nodig voor het u8g2 object
#include <time.h>

// Functie declaratie
void manageBrightness();
void updateDateTimeStrings(struct tm* timeInfo);

// Extern variabelen (zodat ze ook elders gebruikt kunnen worden indien nodig)
[[maybe_unused]] extern const char* ntpServer;
[[maybe_unused]] extern const long gmtOffset_sec;
[[maybe_unused]] extern const int daylightOffset_sec;

#endif
