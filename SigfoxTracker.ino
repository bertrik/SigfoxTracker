#include <Arduino.h>
#include <math.h>

#include "SoftwareSerial.h"
#include "TinyGPSPlus.h"
#include "IO_WSSFM10.h"

#define GPS_BAUD        9600

#define PIN_LED         D0
#define PIN_GPS_RX      D1
#define PIN_SIGFOX_RX   D2
#define PIN_SIGFOX_TX   D3

#define print Serial.printf

static SoftwareSerial gps_serial(PIN_GPS_RX);
static TinyGPSPlus gps;
static IO_WSSFM10 sigfox(PIN_SIGFOX_RX, PIN_SIGFOX_TX, false);

void setup(void)
{
    Serial.begin(115200);
    print("\nSIGFOXTRACKER\n");

    pinMode(PIN_LED, OUTPUT);
    digitalWrite(PIN_LED, 1);

    gps_serial.begin(GPS_BAUD);

    sigfox.begin();
    print("SigFox status = %s\n", sigfox.test().c_str());
    print("SigFox ID     = %s\n", sigfox.getID().c_str());
    print("SigFox PAC    = %s\n", sigfox.getPAC().c_str());
}

static size_t encode(uint8_t * buf, size_t size, float lat, float lon, float alt, int numsats)
{
    int32_t lat_i = round(lat * 1000000);
    int32_t lon_i = round(lon * 1000000);
    uint16_t alt_i = round(alt * 10);

    size_t idx = 0;
    buf[idx++] = (lat_i >> 24) & 0xFF;
    buf[idx++] = (lat_i >> 16) & 0xFF;
    buf[idx++] = (lat_i >> 8) & 0xFF;
    buf[idx++] = (lat_i >> 0) & 0xFF;
    buf[idx++] = (lon_i >> 24) & 0xFF;
    buf[idx++] = (lon_i >> 16) & 0xFF;
    buf[idx++] = (lon_i >> 8) & 0xFF;
    buf[idx++] = (lon_i >> 0) & 0xFF;
    buf[idx++] = (alt_i >> 8) & 0xFF;
    buf[idx++] = (alt_i >> 0) & 0xFF;
    buf[idx++] = numsats;
    return idx;
}

// state variables
static uint8_t buf[12];
static float gps_lat, gps_lon, gps_alt;
static int gps_sats = 0;
static bool gps_have_new = false;
static int period_last = -1;

void loop(void)
{
    // poll command line input
    // ...

    // poll GPS
    while (gps_serial.available()) {
        char c = gps_serial.read();
        if (gps.encode(c)) {
            if (gps.location.isUpdated() && gps.altitude.isUpdated() && gps.satellites.isUpdated()) {
                gps_lat = gps.location.lat();
                gps_lon = gps.location.lng();
                gps_alt = gps.altitude.meters();
                gps_sats = gps.satellites.value();
                gps_have_new = true;
                print("lat=%.6f,lon=%.6f,alt=%.1f,num=%d\n", gps_lat, gps_lon, gps_alt, gps_sats);
                break;
            }
        }
    }

    // handle SigFox
    int period = millis() / 600000L;
    if (period != period_last) {
        // send if we have a GPS fix
        if (gps_have_new) {
            period_last = period;
            gps_have_new = false;

            int len = encode(buf, sizeof(buf), gps_lat, gps_lon, gps_alt, gps_sats);
            for (int i = 0; i < len; i++) {
                print(" %02X", buf[i]);
            }
            print("\n");
        }
    }
}
