/*******************************************************************************
 * Copyright (c) 2015 Thomas Telkamp and Matthijs Kooijman
 * Do not forget to define the radio type correctly in config.h.
 *******************************************************************************/
#include <SHT1x.h>
#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>

#define RFM95_RST 13
#define RFM95_CS  18
#define RFM95_DIO0 12
#define RFM95_DIO1 11
#define RFM95_DIO2 5
#define DATA_PIN 8      //Sensor data pin
#define CLOCK_PIN 9     //Sensor clock pin

//Sensor class creation
SHT1x sht1x(DATA_PIN, CLOCK_PIN);

/*************************************
 * TODO: Change the following keys
 * NwkSKey: network session key, AppSKey: application session key, and DevAddr: end-device address
 *************************************/
static const PROGMEM u1_t NWKSKEY[16] = { 0x8F, 0xBE, 0x96, 0x7E, 0x89, 0x93, 0xE8, 0xB5, 0x63, 0xF4, 0xED, 0x3F, 0x07, 0x9A, 0x87, 0x57 };
static const u1_t PROGMEM APPSKEY[16] = { 0xA6, 0xB5, 0x92, 0x14, 0x45, 0x9D, 0xDB, 0x80, 0xF4, 0x74, 0x27, 0xEF, 0x9F, 0x7E, 0xAC, 0xD8 };
static const u4_t DEVADDR = 0x26011B57 ; // <-- Change this address for every node!

// These callbacks are only used in over-the-air activation, so they are
// left empty here (we cannot leave them out completely unless
// DISABLE_JOIN is set in config.h, otherwise the linker will complain).
void os_getArtEui (u1_t* buf) { }
void os_getDevEui (u1_t* buf) { }
void os_getDevKey (u1_t* buf) { }

static osjob_t sendjob;

// Schedule TX every this many seconds (might become longer due to duty
// cycle limitations).
const unsigned TX_INTERVAL = 10;

// Pin mapping
const lmic_pinmap lmic_pins = {
    RFM95_CS,   //nss
    LMIC_UNUSED_PIN,    //rxtx
    RFM95_RST,  //rst
    {RFM95_DIO0, RFM95_DIO1, LMIC_UNUSED_PIN},  //DIO0,DIO1,DIO2
};

void onEvent (ev_t ev) {
    Serial.print(os_getTime());
    Serial.print(": ");
    switch(ev) {;
        case EV_TXCOMPLETE:
            Serial.println(F("EV_TXCOMPLETE (includes waiting for RX windows)"));
            // Schedule next transmission
//            os_setTimedCallback(&sendjob, os_getTime()+sec2osticks(TX_INTERVAL), do_send);
            break;
         default:
            Serial.println(F("Unknown event"));
            break;
    }
}

void do_send(osjob_t* j){
    // Payload to send (uplink)
    static uint8_t message[16];

    float celsius = sht1x.readTemperatureC();
    float humi = sht1x.readHumidity();
    
    int16_t temperature = (int16_t)(celsius * 100);
    int16_t humidity = (int16_t)(humi * 100);
    
    message[0] = (temperature >> 8) & 0xFF;
    message[1] = temperature & 0xFF;
    message[2] = (humidity >> 8) & 0xFF;
    message[3] = humidity & 0xFF;

    //LMP3
    sleepSeconds(3);

    // Check if there is not a current TX/RX job running
    if (LMIC.opmode & OP_TXRXPEND) {
        Serial.println(F("OP_TXRXPEND, not sending"));
    } else {
        // Prepare upstream data transmission at the next possible time.
        LMIC_setTxData2(1, message, sizeof(message), 0);
        Serial.println(F("Packet sent..."));
    }
// Next TX is scheduled after TX_COMPLETE event.
}

void setup() {
    Serial.begin(9600);
}
//
void radio_setup(){
    Serial.println(F("Starting"));

    // LMIC init
    os_init();
    // Reset the MAC state. Session and pending data transfers will be discarded.
    LMIC_reset();

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
    LMIC_setSession (0x1, DEVADDR, nwkskey, appskey);
    #else
    // If not running an AVR with PROGMEM, just use the arrays directly
    LMIC_setSession (0x1, DEVADDR, NWKSKEY, APPSKEY);
    #endif

    #if defined(CFG_eu868)
    // Set up the channels used by the Things Network, which corresponds
    // to the defaults of most gateways. Without this, only three base
    // channels from the LoRaWAN specification are used, which certainly
    // works, so it is good for debugging, but can overload those
    // frequencies, so be sure to configure the full frequency range of
    // your network here (unless your network autoconfigures them).
    // Setting up channels should happen after LMIC_setSession, as that
    // configures the minimal channel set.
    // NA-US channels 0-71 are configured automatically
    LMIC_setupChannel(0, 868100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
//    LMIC_setupChannel(1, 868300000, DR_RANGE_MAP(DR_SF12, DR_SF7B), BAND_CENTI);      // g-band
//    LMIC_setupChannel(2, 868500000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
//    LMIC_setupChannel(3, 867100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
//    LMIC_setupChannel(4, 867300000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
//    LMIC_setupChannel(5, 867500000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
//    LMIC_setupChannel(6, 867700000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
//    LMIC_setupChannel(7, 867900000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
//    LMIC_setupChannel(8, 868800000, DR_RANGE_MAP(DR_FSK,  DR_FSK),  BAND_MILLI);      // g2-band
    // TTN defines an additional channel at 869.525Mhz using SF9 for class B
    // devices' ping slots. LMIC does not have an easy way to define set this
    // frequency and support for class B is spotty and untested, so this
    // frequency is not configured here.
    #elif defined(CFG_us915)
    // NA-US channels 0-71 are configured automatically
    // but only one group of 8 should (a subband) should be active
    // TTN recommends the second sub band, 1 in a zero based count.
    // https://github.com/TheThingsNetwork/gateway-conf/blob/master/US-global_conf.json
    LMIC_selectSubBand(1);
    #endif

    // Disable link check validation
    LMIC_setLinkCheckMode(0);

    // TTN uses SF9 for its RX2 window.
    LMIC.dn2Dr = DR_SF9;

    // Set data rate and transmit power for uplink (note: txpow seems to be ignored by the library)
    LMIC_setDrTxpow(DR_SF7,14);
  
    // Start job
//    do_send(&sendjob);
}

void loop() {
  //LPM3
  sleepSeconds(3);
  //Sensor read and radio setup
  radio_setup();
  //LPM3
  sleepSeconds(3);
  //Transmission
  do_send(&sendjob);
}
