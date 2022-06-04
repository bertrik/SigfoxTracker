#include <Arduino.h>
#include <math.h>

#include "SoftwareSerial.h"
#include "TinyGPS.h"
#include "IO_WSSFM10.h"

#define GPS_BAUD        9600

#define PIN_LED         D0
#define PIN_GPS_RX      D1
#define PIN_SIGFOX_RX   D2
#define PIN_SIGFOX_TX   D3

#define print Serial.printf

static SoftwareSerial gps_serial(PIN_GPS_RX);
static TinyGPS gps;
static IO_WSSFM10 sigfox(PIN_SIGFOX_RX, PIN_SIGFOX_TX, true);

void setup(void)
{
    Serial.begin(115200);
    print("\nSIGFOXTRACKER\n");

    pinMode(PIN_LED, OUTPUT);
    digitalWrite(PIN_LED, 1);

    gps_serial.begin(GPS_BAUD);

//    sigfox.begin();
//    print("SigFox ID  = %s", sigfox.getID().c_str());
//    print("SigFox PAC = %s", sigfox.getPAC().c_str());
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
static int gps_numsats = 0;
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
            float lat, lon;
            unsigned long age;
            gps.f_get_position(&lat, &lon, &age);
            if (age < 2000) {
                // fix is recent
                gps_lat = lat;
                gps_lon = lon;
                gps_alt = gps.f_altitude();
                gps_numsats = gps.satellites();
                gps_have_new = true;
            }
            print("lat=%.6f,lon=%.6f,alt=%.1f,num=%d,age=%lu\n", gps_lat, gps_lon, gps_alt,
                  gps_numsats, age);
            break;
        }
    }

    // handle SigFox
    int period = millis() / 600000L;
    if (period != period_last) {
        // send if we have a GPS fix
        if (gps_have_new) {
            period_last = period;
            gps_have_new = false;

            int len = encode(buf, sizeof(buf), gps_lat, gps_lon, gps_alt, gps_numsats);
            for (int i = 0; i < len; i++) {
                print(" %02X", buf[i]);
            }
        }
    }
}
