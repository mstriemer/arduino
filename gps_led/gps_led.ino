// Test code for Adafruit GPS modules using MTK3329/MTK3339 driver
//
// This code shows how to listen to the GPS module in an interrupt
// which allows the program to have more 'freedom' - just parse
// when a new NMEA sentence is available! Then access data when
// desired.
//
// Tested and works great with the Adafruit Ultimate GPS module
// using MTK33x9 chipset
//    ------> http://www.adafruit.com/products/746
// Pick one up today at the Adafruit electronics shop
// and help support open source hardware & software! -ada

#include <Adafruit_GPS.h>
#if ARDUINO >= 100
#include <SoftwareSerial.h>
#else
// Older Arduino IDE requires NewSoftSerial, download from:
// http://arduiniana.org/libraries/newsoftserial/
#include <NewSoftSerial.h>
// DO NOT install NewSoftSerial if using Arduino 1.0 or later!
#endif

// Connect the GPS Power pin to 5V
// Connect the GPS Ground pin to ground
// If using software serial (sketch example default):
//   Connect the GPS TX (transmit) pin to Digital 3
//   Connect the GPS RX (receive) pin to Digital 2
// If using hardware serial (e.g. Arduino Mega):
//   Connect the GPS TX (transmit) pin to Arduino RX1, RX2 or RX3
//   Connect the GPS RX (receive) pin to matching TX1, TX2 or TX3

// If using software serial, keep these lines enabled
// (you can change the pin numbers to match your wiring):
#if ARDUINO >= 100
SoftwareSerial mySerial(3, 2);
#else
NewSoftSerial mySerial(3, 2);
#endif
Adafruit_GPS GPS(&mySerial);
// If using hardware serial (e.g. Arduino Mega), comment
// out the above six lines and enable this line instead:
//Adafruit_GPS GPS(&Serial1);


// Set GPSECHO to 'false' to turn off echoing the GPS data to the Serial console
// Set to 'true' if you want to debug and listen to the raw GPS sentences
#define GPSECHO  false

// this keeps track of whether we're using the interrupt
// off by default!
boolean usingInterrupt = false;
void useInterrupt(boolean); // Func prototype keeps Arduino 0023 happy

float t_lat_min, t_lat_max, t_lon_min, t_lon_max;

#define OUTPUT_PAUSE_MILLIS 2000

// Set the target
#define SHS_LAT_MIN 4951.3751
#define SHS_LAT_MAX 4951.4153
#define SHS_LON_MIN 9713.7445
#define SHS_LON_MAX 9713.8043

#define VAS_LAT_MIN 4948.298
#define VAS_LAT_MAX 4948.31
#define VAS_LON_MIN 9712.53
#define VAS_LON_MAX 9712.56

bool cheat = false;
int iters = 0;
uint32_t lastMillis = millis();

void setupSerialMonitor() {
    Serial.begin(115200);
    Serial.println("Adafruit GPS library basic test!");
}

void setupGPS() {
    // 9600 NMEA is the default baud rate for Adafruit MTK GPS's- some use 4800
    GPS.begin(9600);

    // uncomment this line to turn on RMC (recommended minimum) and GGA (fix data) including altitude
    GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);

    // Set the update rate
    GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);   // 1 Hz update rate
    // For the parsing code to work nicely and have time to sort thru the data, and
    // print it out we don't suggest using anything higher than 1 Hz

    // the nice thing about this code is you can have a timer0 interrupt go off
    // every 1 millisecond, and read data from the GPS for you. that makes the
    // loop code a heck of a lot easier!
    useInterrupt(true);
}

void setup() {
    setupSerialMonitor();
    setupGPS();

    t_lat_min = SHS_LAT_MIN;
    t_lat_max = SHS_LAT_MAX;
    t_lon_min = SHS_LON_MIN;
    t_lon_max = SHS_LON_MAX;

    pinMode(13, OUTPUT);

    delay(1000);
}


// Interrupt is called once a millisecond, looks for any new GPS data, and stores it
SIGNAL(TIMER0_COMPA_vect) {
    char c = GPS.read();
    // if you want to debug, this is a good time to do it!
    if (GPSECHO)
        if (c) UDR0 = c;
    // writing direct to UDR0 is much much faster than Serial.print
    // but only one character can be written at a time.
}

void useInterrupt(boolean v) {
    if (v) {
        // Timer0 is already used for millis() - we'll just interrupt somewhere
        // in the middle and call the "Compare A" function above
        OCR0A = 0xAF;
        TIMSK0 |= _BV(OCIE0A);
        usingInterrupt = true;
    } else {
        // do not call the interrupt function COMPA anymore
        TIMSK0 &= ~_BV(OCIE0A);
        usingInterrupt = false;
    }
}

bool isWithinBounds(float latitude, float longitude) {
    return t_lat_min <= latitude &&
        t_lat_max >= latitude &&
        t_lon_min <= longitude &&
        t_lon_max >= longitude;
}

void newGPSMessage() {
    char lat, lon;
    float latitude, longitude;

    latitude = GPS.latitude;
    lat = GPS.lat;
    longitude = GPS.longitude;
    lon = GPS.lon;

    if (iters++ % 5 == 0 && cheat) {
        latitude = (t_lat_max - t_lat_min) + t_lat_min;
        longitude = (t_lon_max - t_lon_min) + t_lon_min;
    }

    if (isWithinBounds(latitude, longitude)) {
        digitalWrite(13, HIGH);
    } else {
        digitalWrite(13, LOW);
    }
}

bool isLoggingTime() {
    // if millis() or lastMillis wraps around, we'll just reset it
    if (lastMillis > millis()) lastMillis = millis();

    if (millis() - lastMillis > OUTPUT_PAUSE_MILLIS) {
        lastMillis = millis(); // reset the lastMillis
        return true;
    } else {
        return false;
    }
}

void printGPSData() {
    if (isLoggingTime()) {
        Serial.print("\nTime: ");
        Serial.print(GPS.hour, DEC); Serial.print(':');
        Serial.print(GPS.minute, DEC); Serial.print(':');
        Serial.print(GPS.seconds, DEC); Serial.print('.');
        Serial.println(GPS.milliseconds);
        Serial.print("Date: ");
        Serial.print(GPS.day, DEC); Serial.print('/');
        Serial.print(GPS.month, DEC); Serial.print("/20");
        Serial.println(GPS.year, DEC);
        Serial.print("Fix: "); Serial.print((int)GPS.fix);
        Serial.print(" quality: "); Serial.println((int)GPS.fixquality);
        if (GPS.fix) {
            Serial.print("Location: ");
            Serial.print(GPS.latitude, 4); Serial.print(GPS.lat);
            Serial.print(", ");
            Serial.print(GPS.longitude, 4); Serial.println(GPS.lon);

            Serial.print("Speed (knots): "); Serial.println(GPS.speed);
            Serial.print("Angle: "); Serial.println(GPS.angle);
            Serial.print("Altitude: "); Serial.println(GPS.altitude);
            Serial.print("Satellites: "); Serial.println((int)GPS.satellites);
        }
    }
}

bool validGPSMessage() {
    return GPS.newNMEAreceived() && GPS.parse(GPS.lastNMEA());
}

void handleNewGPSMessage() {
    if (validGPSMessage())
        newGPSMessage();
}

void loop() {
    handleNewGPSMessage();
    printGPSData();
}
