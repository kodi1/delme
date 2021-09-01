/*******************************************************************************
 * Copyright (c) 2015 Thomas Telkamp and Matthijs Kooijman
 * Copyright (c) 2018 Terry Moore, MCCI
 *
 * Permission is hereby granted, free of charge, to anyone
 * obtaining a copy of this document and accompanying files,
 * to do whatever they want with them without any restriction,
 * including, but not limited to, copying, modification and redistribution.
 * NO WARRANTY OF ANY KIND IS PROVIDED.
 *
 * This example sends a valid LoRaWAN packet with payload "Hello,
 * world!", using frequency and encryption settings matching those of
 * the The Things Network. It's pre-configured for the Adafruit
 * Feather M0 LoRa.
 *
 * This uses OTAA (Over-the-air activation), where where a DevEUI and
 * application key is configured, which are used in an over-the-air
 * activation procedure where a DevAddr and session keys are
 * assigned/generated for use with all further communication.
 *
 * Note: LoRaWAN per sub-band duty-cycle limitation is enforced (1% in
 * g1, 0.1% in g2), but not the TTN fair usage policy (which is probably
 * violated by this sketch when left running for longer)!

 * To use this sketch, first register your application and device with
 * the things network, to set or generate an AppEUI, DevEUI and AppKey.
 * Multiple devices can use the same AppEUI, but each device has its own
 * DevEUI and AppKey.
 *
 * Do not forget to define the radio type correctly in
 * arduino-lmic/project_config/lmic_project_config.h or from your BOARDS.txt.
 *
 *******************************************************************************/
#ifndef USE_REJOIN
#error USE_REJOIN not defined
#endif

#include <Arduino.h>
#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>
#include <arduino_lmic_hal_boards.h>

#include <U8x8lib.h>

#include <Wire.h>
#include <Adafruit_BMP280.h>

U8X8_SSD1306_128X64_NONAME_SW_I2C u8x8(/* clock=*/ 15, /* data=*/ 4, /* reset=*/ 16);

//
// For normal use, we require that you edit the sketch to replace FILLMEIN
// with values assigned by the TTN console. However, for regression tests,
// we want to be able to compile these scripts. The regression tests define
// COMPILE_REGRESSION_TEST, and in that case we define FILLMEIN to a non-
// working but innocuous value.
//
#ifdef COMPILE_REGRESSION_TEST
# define FILLMEIN 0
#else
# warning "You must replace the values marked FILLMEIN with real values from the TTN control panel!"
# define FILLMEIN (#dont edit this, edit the lines that use FILLMEIN)
#endif

#if 1
// This EUI must be in little-endian format, so least-significant-byte
// first. When copying an EUI from ttnctl output, this means to reverse
// the bytes. For TTN issued EUIs the last bytes should be 0xD5, 0xB3,
// 0x70.
static const u1_t PROGMEM APPEUI[8]= { 0x94, 0x6E, 0x53, 0x46, 0xA6, 0xF9, 0x81, 0x60 };
void os_getArtEui (u1_t* buf) { memcpy_P(buf, APPEUI, 8);}

// This should also be in little endian format, see above.
static const u1_t PROGMEM DEVEUI[8]= { 0x1F, 0x30, 0x1C, 0x63, 0xDD, 0xF9, 0x81, 0x60 };
void os_getDevEui (u1_t* buf) { memcpy_P(buf, DEVEUI, 8);}

// This key should be in big endian format (or, since it is not really a
// number but a block of memory, endianness does not really apply). In
// practice, a key taken from the TTN console can be copied as-is.
static const u1_t PROGMEM APPKEY[16] = { 0xC4, 0xB0, 0x9B, 0x6D, 0xEC, 0x50, 0x70, 0x34, 0xF7, 0x61, 0x2B, 0x57, 0xFC, 0x23, 0x09, 0x61 };
void os_getDevKey (u1_t* buf) {  memcpy_P(buf, APPKEY, 16);}
#else
// This EUI must be in little-endian format, so least-significant-byte
// first. When copying an EUI from ttnctl output, this means to reverse
// the bytes. For TTN issued EUIs the last bytes should be 0xD5, 0xB3,
// 0x70.
static const u1_t PROGMEM APPEUI[8]= { 0xF8, 0xCF, 0xD6, 0xCF, 0x4E, 0xF9, 0x81, 0x60 };
void os_getArtEui (u1_t* buf) { memcpy_P(buf, APPEUI, 8);}

// This should also be in little endian format, see above.
static const u1_t PROGMEM DEVEUI[8]= { 0x74, 0x3A, 0x40, 0xBE, 0x7A, 0xF9, 0x81, 0x60 };
void os_getDevEui (u1_t* buf) { memcpy_P(buf, DEVEUI, 8);}

// This key should be in big endian format (or, since it is not really a
// number but a block of memory, endianness does not really apply). In
// practice, a key taken from the TTN console can be copied as-is.
static const u1_t PROGMEM APPKEY[16] = { 0x9E, 0xE9, 0x65, 0xA0, 0xC9, 0x68, 0xEF, 0xBD, 0x73, 0x6B, 0xAC, 0x79, 0x89, 0x82, 0x03, 0x6A };
void os_getDevKey (u1_t* buf) {  memcpy_P(buf, APPKEY, 16);}
#endif

static osjob_t sendjob, initjob, sleepjob;

QueueHandle_t display_queue;
TaskHandle_t hndl_display_task;
SemaphoreHandle_t sleep_semaphore;

typedef union {
    struct {
        uint32_t temp;
        uint32_t pressure;
    };
    uint8_t payload[];
} data_t;

typedef enum {
    DISPLAY_CMD_OFF,
    DISPLAY_CMD_ON,
    DISPLAY_CMD_CLEAR,
    DISPLAY_CMD_SHOW,
} display_cmd_t;

typedef struct {
    display_cmd_t   cmd;
    uint32_t        time;
} msg_cmd_t;

typedef struct {
    msg_cmd_t  display;
    uint8_t    payload[];
} msg_data_t;

#define GET_MSG_DATA_SIZE(x) (x + sizeof(msg_cmd_t))

#define DISPLAY_SHOW(show,...)                      \
do {                                                \
    msg_data_t *display_msg;                        \
    size_t size;                                    \
    size = GET_MSG_DATA_SIZE (                      \
            u8x8.getCols() *  u8x8.getCols());      \
    display_msg = (msg_data_t *)malloc(size);       \
    if(NULL == display_msg) {                       \
        Serial.println(                             \
                "Error display msg"                 \
            );                                      \
    } else {                                        \
        snprintf(                                   \
                    (char *)display_msg->payload,   \
                    size,                           \
                    __VA_ARGS__);                   \
        display_msg->display.cmd =                  \
                            DISPLAY_CMD_SHOW;       \
        display_msg->display.time = show;           \
        xQueueSend(                                 \
                    display_queue,                  \
                    &display_msg,                   \
                    portMAX_DELAY);                 \
    }                                               \
} while (0);

#define DISPLAY_OFF()                               \
do {                                                \
    msg_data_t *display_msg;                        \
    size_t size;                                    \
    size = sizeof(display_msg->display);            \
    display_msg = (msg_data_t *)malloc(size);       \
    if(NULL == display_msg) {                       \
        Serial.println(                             \
                "Error display msg"                 \
            );                                      \
    } else {                                        \
        display_msg->display.cmd =                  \
                            DISPLAY_CMD_OFF;        \
        xQueueSend(                                 \
                    display_queue,                  \
                    &display_msg,                   \
                    portMAX_DELAY);                 \
    }                                               \
} while (0);

typedef struct {
    uint32_t netid;
    uint32_t devaddr;
    uint8_t nwkKey[16];
    uint8_t artKey[16];
} sesion_data_t;

typedef struct {
    sesion_data_t sesion;
    uint32_t seqnoUp;
    uint32_t seqnoDn;
    uint8_t display_off;
} lora_data_sleep_t;

RTC_DATA_ATTR lora_data_sleep_t lora_data_sleep;
Adafruit_BMP280 sensor;

void save_sesion_data (lora_data_sleep_t *data)
{
    data->seqnoUp = LMIC.seqnoUp;
    data->seqnoDn = LMIC.seqnoDn;
    if (data->sesion.netid) {
        // already have session data
        return;
    }

    LMIC_getSessionKeys(&data->sesion.netid,
                    &data->sesion.devaddr,
                    data->sesion.nwkKey,
                    data->sesion.artKey);
}

void load_sesion_data (lora_data_sleep_t *data)
{
    if (!data->sesion.netid) {
        // don't have session data
        return;
    }
    LMIC_setSession(data->sesion.netid,
                    data->sesion.devaddr,
                    data->sesion.nwkKey,
                    data->sesion.artKey);
    LMIC.seqnoUp = data->seqnoUp;
    LMIC.seqnoDn = data->seqnoDn;
}

void printHex2(unsigned v)
{
    v &= 0xff;
    if (v < 16)
        Serial.print('0');
    Serial.print(v, HEX);
}

void do_send(osjob_t* j)
{
    float tmp;
    data_t data = {0};

    // tmp = sensor.readPressure();
    // data.pressure = (uint32_t) (tmp * (1 << 16));

    // tmp = sensor.readTemperature();
    // data.temp = (uint32_t) (tmp * (1 << 16));

    // Check if there is not a current TX/RX job running
    if (LMIC.opmode & OP_TXRXPEND) {
        Serial.println(F("Sending in progress"));
    } else {
        // Prepare upstream data transmission at the next possible time.
        LMIC_setTxData2(1, data.payload, sizeof(data)-1, 0);
        Serial.println(F("Data queued"));
    }
    // Next TX is scheduled after TX_COMPLETE event.
}

void do_receive(void *user, uint8_t port, const uint8_t *data, size_t size)
{
    Serial.printf("Port: %d size %d data:\n", port, size);
    DISPLAY_SHOW(
                    10000,
                    "Receive Data:\n"
                    "port: %d size: %d",
                    port,
                    size);
    for (size_t i=0; i < size; ++i) {
        if (i != 0) {
            Serial.print("-");
        }
        printHex2(data[i]);
    }
    Serial.println("");
}

void do_sleep (osjob_t* j)
{
    save_sesion_data(&lora_data_sleep);
    LMIC_shutdown();

    xSemaphoreTake(sleep_semaphore, portMAX_DELAY);

    vTaskDelete(hndl_display_task);
    vSemaphoreDelete(sleep_semaphore);
    vQueueDelete(display_queue);

    esp_sleep_enable_timer_wakeup(60 * 1000000);
    esp_sleep_enable_touchpad_wakeup();
    esp_deep_sleep_start();
}

void event_process (void *user_data, ev_t ev)
{
    Serial.print(os_getTime());
    Serial.print(": ");
    switch(ev) {
        case EV_SCAN_TIMEOUT:
            Serial.println(F("EV_SCAN_TIMEOUT"));
            DISPLAY_SHOW(1500, "EV_SCAN_TIMEOUT");
            break;
        case EV_BEACON_FOUND:
            Serial.println(F("EV_BEACON_FOUND"));
            DISPLAY_SHOW(1500, "EV_BEACON_FOUND");
            break;
        case EV_BEACON_MISSED:
            Serial.println(F("EV_BEACON_MISSED"));
            DISPLAY_SHOW(1500, "EV_BEACON_MISSED");
            break;
        case EV_BEACON_TRACKED:
            Serial.println(F("EV_BEACON_TRACKED"));
            DISPLAY_SHOW(1500, "EV_BEACON_TRACKED");
            break;
        case EV_JOINING:
            Serial.println(F("EV_JOINING"));
            DISPLAY_SHOW(1500, "EV_JOINING");
            break;
        case EV_JOINED:
            Serial.println(F("EV_JOINED"));
            DISPLAY_SHOW(1500, "EV_JOINED");
            {
              sesion_data_t data;
              LMIC_getSessionKeys(&data.netid,
                                    &data.devaddr,
                                    data.nwkKey,
                                    data.artKey);
              Serial.print("netid: ");
              Serial.println(data.netid, DEC);
              Serial.print("devaddr: ");
              Serial.println(data.devaddr, HEX);
              Serial.print("AppSKey: ");
              for (size_t i=0; i<sizeof(data.artKey); ++i) {
                if (i != 0)
                  Serial.print("-");
                printHex2(data.artKey[i]);
              }
              Serial.println("");
              Serial.print("NwkSKey: ");
              for (size_t i=0; i<sizeof(data.nwkKey); ++i) {
                      if (i != 0)
                              Serial.print("-");
                      printHex2(data.nwkKey[i]);
              }
              Serial.println();
            }
            // Disable link check validation (automatically enabled
            // during join, but because slow data rates change max TX
	    // size, we don't use it in this example.
            LMIC_setLinkCheckMode(0);
#if USE_REJOIN == 1
            os_setCallback(&sendjob, &do_send);
#endif
            break;
        /*
        || This event is defined but not used in the code. No
        || point in wasting codespace on it.
        ||
        || case EV_RFU1:
        ||     Serial.println(F("EV_RFU1"));
        ||     break;
        */
        case EV_JOIN_FAILED:
            Serial.println(F("EV_JOIN_FAILED"));
            DISPLAY_SHOW(1500, "EV_JOIN_FAILED");
            DISPLAY_OFF();
            os_setCallback(&sleepjob, &do_sleep);
            break;
        case EV_REJOIN_FAILED:
            Serial.println(F("EV_REJOIN_FAILED"));
            DISPLAY_SHOW(1500, "EV_REJOIN_FAILED");
            DISPLAY_OFF();
            os_setCallback(&sleepjob, &do_sleep);
            DISPLAY_OFF();
            break;

        case EV_TXCOMPLETE:
            Serial.println(F("EV_TXCOMPLETE (includes waiting for RX windows)"));
            DISPLAY_SHOW(3000, "EV_TXCOMPLETE");
            DISPLAY_SHOW(
                            10000,
                            "Tx Done\n"
                            "power: %d\n"
                            "freq:  %.3f\n"
                            "rssi:  %d\n"
                            "snr:   %d\n"
                            "cntUp: %d\n"
                            "cntDn: %d",
                            LMIC.adrTxPow,
                            LMIC.freq/1000000.0,
                            LMIC.snr,
                            LMIC.rssi,
                            LMIC.seqnoUp,
                            LMIC.seqnoDn);
            DISPLAY_OFF();
            os_setCallback(&sleepjob, &do_sleep);
            break;
        case EV_LOST_TSYNC:
            Serial.println(F("EV_LOST_TSYNC"));
            DISPLAY_SHOW(1500, "EV_LOST_TSYNC");
            break;
        case EV_RESET:
            Serial.println(F("EV_RESET"));
            DISPLAY_SHOW(1500, "EV_RESET");
            break;
        case EV_RXCOMPLETE:
            // data received in ping slot
            Serial.println(F("EV_RXCOMPLETE"));
            DISPLAY_SHOW(1500, "EV_RXCOMPLETE");
            break;
        case EV_LINK_DEAD:
            Serial.println(F("EV_LINK_DEAD"));
            DISPLAY_SHOW(1500, "EV_LINK_DEAD");
            break;
        case EV_LINK_ALIVE:
            Serial.println(F("EV_LINK_ALIVE"));
            DISPLAY_SHOW(1500, "EV_LINK_ALIVE");
            break;
        /*
        || This event is defined but not used in the code. No
        || point in wasting codespace on it.
        ||
        || case EV_SCAN_FOUND:
        ||    Serial.println(F("EV_SCAN_FOUND"));
        ||    break;
        */
        case EV_TXSTART:
            Serial.println(F("EV_TXSTART"));
            DISPLAY_SHOW(1500, "EV_TXSTART");
            break;
        case EV_TXCANCELED:
            Serial.println(F("EV_TXCANCELED"));
            DISPLAY_SHOW(1500, "EV_TXCANCELED");
            break;
        case EV_RXSTART:
            /* do not print anything -- it wrecks timing */
            break;
        case EV_JOIN_TXCOMPLETE:
            Serial.println(F("EV_JOIN_TXCOMPLETE: no JoinAccept"));
            DISPLAY_SHOW(1500, "EV_JOIN\nTXCOMPLETE:\nno JoinAccept");
            break;

        default:
            Serial.print(F("Unknown event: "));
            Serial.println((unsigned) ev);
            DISPLAY_SHOW(10000, "Unknown event: %d", ev);
            break;
    }
}

void t1_callback (void)
{
    ;
}

void my_init (osjob_t* j)
{
    switch (esp_sleep_get_wakeup_cause())
    {
    case ESP_SLEEP_WAKEUP_TOUCHPAD:
        Serial.println(F("touch wakeup"));
        break;
    case ESP_SLEEP_WAKEUP_TIMER:
        Serial.println(F("timer wakeup"));
        break;
    default:
        Serial.println(F("other wakeup"));
        memset(&lora_data_sleep, 0x00, sizeof(lora_data_sleep));
        break;
    }

    touchAttachInterrupt(KEY_BUILTIN, &t1_callback, 1);

    // Reset the MAC state. Session and pending data transfers will be discarded.
    LMIC_reset();

    // allow much more clock error than the X/1000 default. See:
    // https://github.com/mcci-catena/arduino-lorawan/issues/74#issuecomment-462171974
    // https://github.com/mcci-catena/arduino-lmic/commit/42da75b56#diff-16d75524a9920f5d043fe731a27cf85aL633
    // the X/1000 means an error rate of 0.1%; the above issue discusses using values up to 10%.
    // so, values from 10 (10% error, the most lax) to 1000 (0.1% error, the most strict) can be used.
    LMIC_setClockError(1 * MAX_CLOCK_ERROR / 40);

    LMIC_setLinkCheckMode(0);
    LMIC_setDrTxpow(DR_SF7,14);

    LMIC_registerEventCb(&event_process, NULL);
    LMIC_registerRxMessageCb(&do_receive, NULL);

    load_sesion_data(&lora_data_sleep);

#if USE_REJOIN == 1
    LMIC_tryRejoin();
#else
    os_setCallback(&sendjob, &do_send);
#endif
}

void dispalay(void *create_data)
{
    u8x8.begin();
    u8x8.setFont(u8x8_font_victoriabold8_r);

    msg_data_t *data;

    while (1) {
        xQueueReceive(create_data, &data, portMAX_DELAY);
        u8x8.clear();

        switch (data->display.cmd) {
        case DISPLAY_CMD_ON:
            u8x8.display();
            break;

        case DISPLAY_CMD_OFF:
            u8x8.noDisplay();
            free(data);
            xSemaphoreGive(sleep_semaphore);
            break;

        case DISPLAY_CMD_SHOW:
            u8x8.printf("%s", data->payload);
            break;

        default:
            u8x8.printf("Wrong display cmd: %d", data->display.cmd);
            data->display.time = 10000;
            break;
        }

        delay(data->display.time);

        free(data);
    }
}

void setup()
{
   // Wire.begin(12, 13);
    // sensor.setSampling(Adafruit_BMP280::MODE_NORMAL,
    //                     Adafruit_BMP280::SAMPLING_X2,
    //                     Adafruit_BMP280::SAMPLING_X16,
    //                     Adafruit_BMP280::FILTER_X16,
    //                     Adafruit_BMP280::STANDBY_MS_500);

    esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_ALL);

    while (! Serial);
    Serial.begin(115200);
    Serial.println(F("Starting"));

    display_queue = xQueueCreate(32, sizeof(msg_data_t*));
    if(NULL == display_queue){
        Serial.println("Error creating the queue");
    }

    sleep_semaphore = xSemaphoreCreateBinary();
    if(NULL  == display_queue){
        Serial.println("Error creating the semaphore");
    }

    xTaskCreate(
            dispalay,
            "Display",
            4096,
            display_queue,
            1,
            &hndl_display_task);

    DISPLAY_SHOW(
            1500,
            "LoRa send\n"
            "0123456789abcdef\n"
        );

    #ifdef VCC_ENABLE
    // For Pinoccio Scout boards
    pinMode(VCC_ENABLE, OUTPUT);
    digitalWrite(VCC_ENABLE, HIGH);
    delay(1000);
    #endif

    #if defined(ARDUINO_DISCO_L072CZ_LRWAN1)
    SPI.setMOSI(RADIO_MOSI_PORT);
    SPI.setMISO(RADIO_MISO_PORT);
    SPI.setSCLK(RADIO_SCLK_PORT);
    SPI.setSSEL(RADIO_NSS_PORT);
    #endif

    // Pin mapping
    //  We use the built-in mapping -- see src/hal/getpinmap_thisboard.cpp
    //
    // If your board isn't supported, declare an lmic_pinmap object as static
    // or global, and set pPimMap to that pointer.
    //
    const lmic_pinmap *pPinMap = Arduino_LMIC::GetPinmap_ThisBoard();

    // don't die mysteriously; die noisily.
    if (pPinMap == nullptr) {
        pinMode(LED_BUILTIN, OUTPUT);
        for (;;) {
            // flash lights, sleep.
            for (int i = 0; i < 5; ++i) {
                digitalWrite(LED_BUILTIN, 1);
                delay(100);
                digitalWrite(LED_BUILTIN, 0);
                delay(900);
            }
            Serial.println(F("board not known to library; add pinmap or update getconfig_thisboard.cpp"));
        }
    }

    os_init_ex(pPinMap);

    os_setCallback(&initjob, &my_init);

#if CFG_LMIC_US_like
    // This makes joins faster in the US because we don't wander all over the
    // spectrum.
    LMIC_selectSubBand(1);
#endif
}

void loop()
{
    os_runloop_once();
}
