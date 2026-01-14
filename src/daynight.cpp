/*
 * (c)2026 R van Dorland
 */

#include "daynight.h"
#include "helpers.h"

// Het u8g2 object wordt in main.cpp gedefinieerd, we vertellen de compiler dat het bestaat
extern U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2; // Pas het type aan naar jouw specifieke display type!

// const char* TZ_INFO = SECRET_TZ_INFO;
// const char* ntpServer = SECRET_NTP_SERVER;
double sunrise_local = 0;
double sunset_local = 0;


double latitude = SECRET_LAT;
double longitude = SECRET_LON;

void manageBrightness()
{
    struct tm timeinfo;
    if (!getLocalTime(&timeinfo))
        return;

    time_t now = time(nullptr);
    double transit, sunrise, sunset;

    // 1. Bereken de tijden (deze komen altijd terug in UTC)
    calcSunriseSunset(now, latitude, longitude, transit, sunrise, sunset);

    // 2. Bepaal de lokale offset (Winter = 1.0, Zomer = 2.0)
    double utcOffset = (timeinfo.tm_isdst > 0) ? 2.0 : 1.0;

    // 3. Tel de offset handmatig op bij de resultaten
    double sunrise_local = sunrise + utcOffset;
    double sunset_local = sunset + utcOffset;

    // 4. Formatteer de LOKALE tijden voor het display
    sunriseStr = formatTime(sunrise_local);
    sunsetStr = formatTime(sunset_local);

    // 5. Gebruik de lokale tijden voor de contrast-regeling
    double currentHour = timeinfo.tm_hour + (timeinfo.tm_min / 60.0);

    // Pas het contrast aan op basis van de zon
    if (currentHour > sunrise && currentHour < sunset) {
        u8g2.setContrast(100); // Overdag fel
    } else {
        u8g2.setContrast(5); // 's Nachts gedimd
    }
}
