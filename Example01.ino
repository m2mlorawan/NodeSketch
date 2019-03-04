/*THIS SKETCH IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, 
INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. 
m2mlorawan@gmail.com
*/

#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>
#include <CayenneLPP.h>
CayenneLPP lpp(51); // create a buffer of 51 bytes to store the payload
//
//#define CFG_us915
//#define CFG_as923

#define ABP
//#define OTA

// show debug statements; comment next line to disable debug statements
#define DEBUG

#define UNO11

// Product model
#define PCB01   //Model A328  | Arduino Pro Mini M2M Kit
//#define PCB02   //Model B1284

//Sleep Time in /1000 Sec. might depend on clock speed.
float slptime = 5000;
//float slptime = 10000;
//float slptime = 300000;

//UnCommnet Sensor 
//Select Only one sensor for 328P
//#define SENSOR_VOLT  
//#define SENSOR_AM2321
//#define SENSOR_HTU21D
#define SENSOR_GPSM6N

//OTAA 
// The 2 below should be in little endian format (lsb)

static const uint8_t PROGMEM APPEUI[8]= { 0xFF, 0x65, 0x00, 0xF0, 0x7E, 0xD5, 0xB3, 0xFF };
static const uint8_t PROGMEM DEVEUI[8]= { 0xFF, 0x8F, 0xDA, 0xBB, 0xDD, 0x70, 0x3F, 0xFF };
// This should be in big endian format (msb)
static const uint8_t PROGMEM APPKEY[16] = { 0xFF, 0x5A, 0x42, 0xB8, 0x71, 0x50, 0x2E, 0xBA, 0x68, 0x38, 0xC5, 0x28, 0xDE, 0x07, 0xC2, 0xFF };

//ABP
#if defined(UNO11)
static const uint8_t PROGMEM NWKSKEY[16] = { 0xFF, 0x87, 0x2B, 0x96, 0x71, 0xE3, 0xEA, 0xFB, 0xFC, 0x18, 0x72, 0x79, 0xF1, 0xF9, 0x9A, 0xFF };
static const uint8_t PROGMEM APPSKEY[16] = { 0xFF, 0x1C, 0x17, 0xAD, 0xE6, 0xED, 0x42, 0x92, 0x1D, 0xEE, 0xBE, 0x29, 0xA8, 0x6C, 0xB5, 0xFF };
static const uint32_t DEVADDR = 0xAAAAAAAA;
#endif

#if defined(UNO12)
static const uint8_t PROGMEM NWKSKEY[16] = { 0xFF, 0x6C, 0x16, 0x3E, 0x72, 0xE7, 0x67, 0x94, 0xA6, 0xB5, 0xCA, 0xB4, 0x75, 0xB2, 0xE8, 0xFF };
static const uint8_t PROGMEM APPSKEY[16] = { 0xFF, 0x2D, 0x9C, 0x36, 0xAA, 0xDE, 0x30, 0x4F, 0xF2, 0x99, 0x20, 0x68, 0x1C, 0x57, 0xAB, 0xFF };
static const uint32_t DEVADDR = 0xAAAAAAAA;
#endif


#if defined(SENSOR_VOLT)
double Vcc = 5.0; // not necessarily true
int value = analogRead(0);
//double volt = (value / 1023.0) * Vcc; // only correct if Vcc = 5.0 volts
float volt = (value / 1023.0) * Vcc; // only correct if Vcc = 5.0 volts
#endif


#if defined(SENSOR_AM2321)
#include <Wire.h>
#include <AM2321.h>
#endif

#if defined(SENSOR_HTU21D)
#include "SparkFunHTU21D.h"
#endif


#if defined(SENSOR_GPSM6N)
#include <TinyGPS.h>
#include <SoftwareSerial.h>
float lat = 13.8000, lon = 100.1000; // create dummy variable for latitude and longitude object
#if defined(PCB01)
SoftwareSerial gpsSerial(2, 3); //rx,tx
#endif
#if defined(PCB02)
SoftwareSerial gpsSerial(10, 11); //rx,tx
#endif
TinyGPS gps;
#endif


#ifdef OTA
void os_getArtEui(u1_t* buf)
{
    memcpy_P(buf, APPEUI, 8);
}
void os_getDevEui(u1_t* buf) { memcpy_P(buf, DEVEUI, 8); }
void os_getDevKey(u1_t* buf) { memcpy_P(buf, APPKEY, 16); }
#else
void os_getArtEui(u1_t* buf)
{
}
void os_getDevEui(u1_t* buf) {}
void os_getDevKey(u1_t* buf) {}
#endif

//
static osjob_t sendjob;

// Schedule TX every this many seconds (might become longer due to duty cycle limitations).
const unsigned TX_INTERVAL = 5;


#if defined(PCB01)
// PCB Version 1,2 ATMEGA328
const lmic_pinmap lmic_pins = {
    .nss = 10,
    .rxtx = LMIC_UNUSED_PIN,
    .rst = LMIC_UNUSED_PIN,
    .dio = { 4, 5, 7 },
};
#endif

#if defined(PCB02)
//ATMEGA1284
#define __AVR_ATmega1280__
const lmic_pinmap lmic_pins = {
    .nss = 14,
    .rxtx = LMIC_UNUSED_PIN,
    .rst = LMIC_UNUSED_PIN,
    .dio = { 12, 13, 15 },
};
#endif


/* **************************************************************
 * low power
 * *************************************************************/
// low power library does not work for Sodaq processor (ARM)
#ifndef _Sodaq_RN2483_h
  #include "LowPower.h"
#endif


/* **************************************************************
* sleep
* *************************************************************/
void do_sleep(float sleepTime) {

  #ifdef DEBUG
    Serial.print(F("Sleep for "));
//  Serial.print(sleepTime/1*12, 3);
    Serial.print(sleepTime/1000, 3);
    Serial.println(F(" seconds"));
  #endif

  Serial.flush();

  #ifndef _Sodaq_RN2483_h
    // sleep logic using LowPower library
    int delays[] = {8000, 4000, 2000, 1000, 500, 250, 120, 60, 30, 15};
    period_t sleep[] = {SLEEP_8S, SLEEP_4S, SLEEP_2S, SLEEP_1S, SLEEP_500MS,  SLEEP_250MS, SLEEP_120MS, SLEEP_60MS, SLEEP_30MS, SLEEP_15MS};

    // correction for overhead in this routine
    sleepTime = sleepTime * 0.93;

    float x;
    unsigned int i;
    for (i=0; i<=9; i++) {
      for (x=sleepTime; x>=delays[i]; x-=delays[i]) {
        LowPower.powerDown(sleep[i], ADC_OFF, BOD_OFF);
        sleepTime -= delays[i];
      }
    }
  #else
    // no LowPower implemented yet, so just a delay
    delay(sleepTime);
  #endif
}

long readVcc()
{
// Read 1.1V reference against AVcc
// set the reference to Vcc and the measurement to the internal 1.1V reference
#if defined(__AVR_ATmega32U4__) || defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
    ADMUX = _BV(REFS0) | _BV(MUX4) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
#elif defined(__AVR_ATtiny24__) || defined(__AVR_ATtiny44__) || defined(__AVR_ATtiny84__)
    ADMUX = _BV(MUX5) | _BV(MUX0);
#elif defined(__AVR_ATtiny25__) || defined(__AVR_ATtiny45__) || defined(__AVR_ATtiny85__)
    ADMUX = _BV(MUX3) | _BV(MUX2);
#else
    ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
#endif

    delay(2); // Wait for Vref to settle
    ADCSRA |= _BV(ADSC); // Start conversion
    while (bit_is_set(ADCSRA, ADSC))
        ; // measuring

    uint8_t low = ADCL; // must read ADCL first - it then locks ADCH
    uint8_t high = ADCH; // unlocks both

    long result = (high << 8) | low;

    result = 1125300L / result; // Calculate Vcc (in mV); 1125300 = 1.1*1023*1000
    return result; // Vcc in millivolts
}


void do_send(osjob_t* j)
{
    // Check if there is not a current TX/RX job running
    if (LMIC.opmode & OP_TXRXPEND) {
        Serial.println(F("OP_TXRXPEND, not sending"));
    }
    else {
// Prepare upstream data transmission at the next possible time.

#if defined(SENSOR_VOLT)
        //******* Voltage Sensor ***************/
        //Serial.print(averageVccInt);
        volt = readVcc();
        #ifdef DEBUG
        Serial.print("V: ");
        Serial.print(volt / 1000);
        Serial.print(" V \n");
        #endif
        /**************************************/
#endif

#if defined(SENSOR_AM2321)
AM2321 ac;
        ac.read();
        float temp = ac.temperature / 10.0;
        float humd = ac.humidity / 10.0;
      #ifdef DEBUG
        Serial.print("temp: ");
        Serial.println(temp);
        Serial.print("Humid: ");
        Serial.println(humd);
      #endif

#endif

#if defined(SENSOR_HTU21D)
        HTU21D myHumidity;
        myHumidity.begin();
        float humd = myHumidity.readHumidity();
        float temp = myHumidity.readTemperature();
  #ifdef DEBUG
        Serial.print("Time:");
        Serial.println(millis());
        Serial.print("Temp:");
        Serial.print(temp, 1);
        Serial.println(" C");
        Serial.print("Humid:");
        Serial.print(humd, 1);
        Serial.print(" %");
        Serial.println();
  #endif
#endif


#if defined(SENSOR_GPSM6N)
           while (gpsSerial.available()) { // check for gps data
               #ifdef DEBUG
               //Serial.println("GPS HW Available");
               #endif
           if (gps.encode(gpsSerial.read())) // encode gps data
            {

                Serial.print("GPS data detected");
                gps.f_get_position(&lat, &lon); // get latitude and longitude
                #ifdef DEBUG
                //display position
                Serial.print("Position: ");
                Serial.print("lat: ");
                Serial.print(lat);
                Serial.print(" "); // print latitude
                Serial.print("lon: ");
                Serial.println(lon); // print longitude
                #endif
                }
       }
#endif

        lpp.reset(); // clear the buffer

#if defined(SENSOR_VOLT)
        lpp.addAnalogInput(1, volt / 1000);
#endif

#if defined(SENSOR_AM2321)
        lpp.addTemperature(2, temp); // on channel 2, add temperature
        lpp.addRelativeHumidity(3, humd); // channel 3, pressure
#endif

#if defined(SENSOR_HTU21D)
        lpp.addTemperature(2, temp); // on channel 2, add temperature
        lpp.addRelativeHumidity(3, humd); // channel 3, pressure
#endif

#if defined(SENSOR_GPSM6N)
        //GPS
        Serial.print("****GPS***** ");
        lpp.addGPS(5, lat, lon, 2);
#endif

        LMIC_setTxData2(1, lpp.getBuffer(), lpp.getSize(), 0);
        Serial.println(F("Packet queued"));
    }
    // Next TX is scheduled after TX_COMPLETE event.

}

void onEvent(ev_t ev)
{
    Serial.print(os_getTime());
    Serial.print(": ");
    switch (ev) {
    case EV_SCAN_TIMEOUT:
        Serial.println(F("EV_SCAN_TIMEOUT"));
        break;
    case EV_BEACON_FOUND:
        Serial.println(F("EV_BEACON_FOUND"));
        break;
    case EV_BEACON_MISSED:
        Serial.println(F("EV_BEACON_MISSED"));
        break;
    case EV_BEACON_TRACKED:
        Serial.println(F("EV_BEACON_TRACKED"));
        break;
    case EV_JOINING:
        Serial.println(F("EV_JOINING"));
        break;
    case EV_JOINED:
        Serial.println(F("EV_JOINED"));
        // Disable link check validation (automatically enabled
        // during join, but not supported by TTN at this time).
        LMIC_setLinkCheckMode(0);
        break;
    case EV_RFU1:
        Serial.println(F("EV_RFU1"));
        break;
    case EV_JOIN_FAILED:
        Serial.println(F("EV_JOIN_FAILED"));
        break;
    case EV_REJOIN_FAILED:
        Serial.println(F("EV_REJOIN_FAILED"));
        break;
        break;

    case EV_TXCOMPLETE:
        Serial.println(F("EV_TXCOMPLETE (includes waiting for RX windows)"));
        if (LMIC.txrxFlags & TXRX_ACK)
            Serial.println(F("Received ack"));
        if (LMIC.dataLen) {
            // data received in rx slot after tx
            Serial.print(F("Received "));
            Serial.print(LMIC.dataLen);
            Serial.print(F(" bytes for downlink: 0x"));
            for (int i = 0; i < LMIC.dataLen; i++) {
                if (LMIC.frame[LMIC.dataBeg + i] < 0x10) {
                    Serial.print(F("0"));
                }
                Serial.print(LMIC.frame[LMIC.dataBeg + i], HEX);
            }
            Serial.println();
        }

        do_sleep(slptime);
       // Schedule next transmission
        os_setTimedCallback(&sendjob, os_getTime() + sec2osticks(TX_INTERVAL), do_send);
        break;
    case EV_LOST_TSYNC:
        Serial.println(F("EV_LOST_TSYNC"));
        break;
    case EV_RESET:
        Serial.println(F("EV_RESET"));
        break;
    case EV_RXCOMPLETE:
        // data received in ping slot
        Serial.println(F("EV_RXCOMPLETE"));
        break;
    case EV_LINK_DEAD:
        Serial.println(F("EV_LINK_DEAD"));
        break;
    case EV_LINK_ALIVE:
        Serial.println(F("EV_LINK_ALIVE"));
        break;
    default:
        Serial.println(F("Unknown event"));
        break;
    }
  
}

void setup()
{
    //  Serial.begin(57600);
    Serial.begin(9600); // connect serial

#if defined(SENSOR_GPSM6N)
    //Serial.println("The GPS Received Signal:");
    //GPS 1
    gpsSerial.begin(9600); // connect gps sensor
#endif

    // LMIC init
    os_init();

    // Reset the MAC state. Session and pending data transfers will be discarded.
    LMIC_reset();


#ifdef ABP
// Set static session parameters. Instead of dynamically establishing a session
// by joining the network, precomputed session parameters are be provided.
#ifdef PROGMEM
    // On AVR, these values are stored in flash and only copied to RAM
    // once. Copy them to a temporary buffer here, LMIC_setSession will
    // copy them into a buffer of its own again.
    uint8_t appskey[sizeof(APPSKEY)];
    uint8_t nwkskey[sizeof(NWKSKEY)];

    memcpy_P(appskey, APPSKEY, sizeof(APPSKEY));
    memcpy_P(nwkskey, NWKSKEY, sizeof(NWKSKEY));
    LMIC_setSession(0x1, DEVADDR, nwkskey, appskey);
#else
    // If not running an AVR with PROGMEM, just use the arrays directly
    LMIC_setSession(0x1, DEVADDR, NWKSKEY, APPSKEY);
#endif

#endif

    // Enable link check validation
    LMIC_setLinkCheckMode(0);
    // Enable data rate adaptation
    //LMIC_setAdrMode(1);
    LMIC_setAdrMode(0);
    // Allow small clock errors
    LMIC_setClockError(MAX_CLOCK_ERROR * 10 / 100);

    // TTN uses SF9 for its RX2 window.
    LMIC.dn2Dr = DR_SF10;

    // Set data rate and transmit power for uplink (note: txpow seems to be ignored by the library)
    //  LMIC_setDrTxpow(DR_SF9, 14);
    LMIC_setDrTxpow(DR_SF7, 14);

    // Start job
    do_send(&sendjob);
}

void loop()
{
    os_runloop_once();
}
