#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>
#include <SHT1x.h>

#define RFM95_RST 13
#define RFM95_CS  18
#define RFM95_DIO0 12
#define RFM95_DIO1 11
#define RFM95_DIO2 5
#define DATA_PIN 8      //Sensor data pin
#define CLOCK_PIN 9     //Sensor clock pin

//Sensor class creation
SHT1x sht1x(DATA_PIN, CLOCK_PIN);

// LoRaWAN NwkSKey, network session key
static const PROGMEM u1_t NWKSKEY[16] = { 0x8F, 0xBE, 0x96, 0x7E, 0x89, 0x93, 0xE8, 0xB5, 0x63, 0xF4, 0xED, 0x3F, 0x07, 0x9A, 0x87, 0x57 };
static const u1_t PROGMEM APPSKEY[16] = { 0xA6, 0xB5, 0x92, 0x14, 0x45, 0x9D, 0xDB, 0x80, 0xF4, 0x74, 0x27, 0xEF, 0x9F, 0x7E, 0xAC, 0xD8 };
static const u4_t DEVADDR = 0x26011B57 ; // <-- Change this address for every node!

// These callbacks are only used in over-the-air activation, so they are
// left empty here (we cannot leave them out completely unless
// DISABLE_JOIN is set in config.h, otherwise the linker will complain).
void os_getArtEui (u1_t* buf) { }
void os_getDevEui (u1_t* buf) { }
void os_getDevKey (u1_t* buf) { }

const unsigned SENSE_INTERVAL = 1;
const unsigned TX_INTERVAL = 1;
static osjob_t sendjob;

// Pin mapping
const lmic_pinmap lmic_pins = {
    RFM95_CS,   //nss
    LMIC_UNUSED_PIN,    //rxtx
    RFM95_RST,  //rst
    {RFM95_DIO0, RFM95_DIO1, LMIC_UNUSED_PIN},  //DIO0,DIO1,DIO2
};

void do_send(osjob_t* j){
    //Sleep TX_INTERVAL seconds
    sleepSeconds(SENSE_INTERVAL);
    float celsius = sht1x.readTemperatureC();
    float humi = sht1x.readHumidity();

    u2_t temperature = (u2_t)(celsius * 100);
    u2_t humidity = (u2_t)(humi * 100);
    
    // prepare and schedule data for transmission
    LMIC.frame[0] = (temperature >> 8) & 0xFF;
    LMIC.frame[1] = temperature & 0xFF;
    LMIC.frame[2] = (humidity >> 8) & 0xFF;
    LMIC.frame[3] = humidity & 0xFF;

    sleepSeconds(TX_INTERVAL);
    // Check if there is not a current TX/RX job running
    if (LMIC.opmode & OP_TXRXPEND) {
        Serial.println(F("OP_TXRXPEND, not sending"));
    } else {
        // Prepare upstream data transmission at the next possible time.
        LMIC_setTxData2(1, LMIC.frame, 4, 0); // (port 1, data, 4 bytes, confirmed)
        Serial.println(F("Packet queued"));
    }
    // Next TX is scheduled after TX_COMPLETE event.
}

void setup() {
  // put your setup code here, to run once:
    Serial.begin(115200);
    Serial.println(F("Starting"));

    initClock();

    sleepSeconds(SENSE_INTERVAL);

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
      LMIC_setupChannel(0, 868100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);
//      LMIC_setupChannel(1, 868300000, DR_RANGE_MAP(DR_SF12, DR_SF7B), BAND_CENTI);      // g-band
//      LMIC_setupChannel(2, 868500000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
//      LMIC_setupChannel(3, 867100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
//      LMIC_setupChannel(4, 867300000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
//      LMIC_setupChannel(5, 867500000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
//      LMIC_setupChannel(6, 867700000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
//      LMIC_setupChannel(7, 867900000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
//      LMIC_setupChannel(8, 868800000, DR_RANGE_MAP(DR_FSK,  DR_FSK),  BAND_MILLI);      // g2-band
    #elif defined(CFG_us915)
      LMIC_selectSubBand(1);
    #endif

    // Disable link check validation
    LMIC_setLinkCheckMode(0);

    // TTN uses SF9 for its RX2 window.
    LMIC.dn2Dr = DR_SF9;

    // Set data rate and transmit power for uplink (note: txpow seems to be ignored by the library)
    LMIC_setDrTxpow(DR_SF7,0);

    // Start job
    do_send(&sendjob);
}

void loop() {
  // put your main code here, to run repeatedly:
//  initClock();
  os_runloop_once();
}

void initClock(){
    // Clock System Setup
    CSCTL0_H = CSKEY >> 8;                    // Unlock CS registers
    CSCTL1 = DCOFSEL_6;                       // Set DCO to 8MHz
    CSCTL2 = SELA__VLOCLK | SELS__DCOCLK | SELM__DCOCLK;  // Set SMCLK = ACLK = VLO
                                                  // MCLK = DCO
    CSCTL3 = DIVA__1 | DIVS__1 | DIVM__1;     // Set all dividers to 1
    CSCTL0_H = 0;                             // Lock CS registers
}

void onEvent (ev_t ev) {
    Serial.print(os_getTime());
    Serial.print(": ");
    switch(ev) {
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
        case EV_TXCOMPLETE:
            Serial.println(F("EV_TXCOMPLETE (includes waiting for RX windows)"));
            if (LMIC.txrxFlags & TXRX_ACK)
              Serial.println(F("Received ack"));
            if (LMIC.dataLen) {
              Serial.println(F("Received "));
              Serial.println(LMIC.dataLen);
              Serial.println(F(" bytes of payload"));
            }

            //Sleep TX_INTERVAL seconds
            sleepSeconds(TX_INTERVAL);
            //Next transmission immediately
            os_setCallback(&sendjob, do_send);
//            os_setTimedCallback(&sendjob, os_getTime()+sec2osticks(TX_INTERVAL), do_send);
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
