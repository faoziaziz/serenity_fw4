#include "Arduino.h"
#include "WiFi.h"
#include "Audio.h"
#include <Wire.h>
#include "MAX30105.h"
#include "heartRate.h"

// Digital I/O used
#define I2S_DOUT 13 // 13
#define I2S_BCLK 14 // 12
#define I2S_LRC 12  // 11
#define I2C_SDA 35
#define I2C_SCL 36
MAX30105 particleSensor;

const byte RATE_SIZE = 4; // Increase this for more averaging. 4 is good.
byte rates[RATE_SIZE];    // Array of heart rates
byte rateSpot = 0;
long lastBeat = 0; // Time at which the last beat occurred

float beatsPerMinute;
int beatAvg;

Audio audio;

String ssid = "gdg-wrk";
String password = "gdg123wrk";

//****************************************************************************************
//                                   A U D I O _ T A S K                                 *
//****************************************************************************************

struct audioMessage
{
    uint8_t cmd;
    const char *txt;
    uint32_t value;
    uint32_t ret;
} audioTxMessage, audioRxMessage;

struct oximeterMessage
{
    uint32_t value;

} oxiMes;

enum : uint8_t
{
    SET_VOLUME,
    GET_VOLUME,
    CONNECTTOHOST,
    CONNECTTOSD
};

QueueHandle_t audioSetQueue = NULL;
QueueHandle_t audioGetQueue = NULL;
QueueHandle_t oximeterQueue = NULL;

void CreateQueues()
{
    audioSetQueue = xQueueCreate(10, sizeof(struct audioMessage));
    audioGetQueue = xQueueCreate(10, sizeof(struct audioMessage));
}

void CreateOxiQueues()
{
    oximeterQueue = xQueueCreate(10, sizeof(struct oximeterMessage));
}

void audioTask(void *parameter)
{
    CreateQueues();
    if (!audioSetQueue || !audioGetQueue)
    {
        log_e("queues are not initialized");
        while (true)
        {
            ;
        } // endless loop
    }

    struct audioMessage audioRxTaskMessage;
    struct audioMessage audioTxTaskMessage;

    audio.setPinout(I2S_BCLK, I2S_LRC, I2S_DOUT);
    audio.setVolume(15); // 0...21

    while (true)
    {
        if (xQueueReceive(audioSetQueue, &audioRxTaskMessage, 1) == pdPASS)
        {
            if (audioRxTaskMessage.cmd == SET_VOLUME)
            {
                audioTxTaskMessage.cmd = SET_VOLUME;
                audio.setVolume(audioRxTaskMessage.value);
                audioTxTaskMessage.ret = 1;
                xQueueSend(audioGetQueue, &audioTxTaskMessage, portMAX_DELAY);
            }
            else if (audioRxTaskMessage.cmd == CONNECTTOHOST)
            {
                audioTxTaskMessage.cmd = CONNECTTOHOST;
                audioTxTaskMessage.ret = audio.connecttohost(audioRxTaskMessage.txt);
                xQueueSend(audioGetQueue, &audioTxTaskMessage, portMAX_DELAY);
            }
            else if (audioRxTaskMessage.cmd == CONNECTTOSD)
            {
                audioTxTaskMessage.cmd = CONNECTTOSD;
                // audioTxTaskMessage.ret = audio.connecttoSD(audioRxTaskMessage.txt);
                xQueueSend(audioGetQueue, &audioTxTaskMessage, portMAX_DELAY);
            }
            else if (audioRxTaskMessage.cmd == GET_VOLUME)
            {
                audioTxTaskMessage.cmd = GET_VOLUME;
                audioTxTaskMessage.ret = audio.getVolume();
                xQueueSend(audioGetQueue, &audioTxTaskMessage, portMAX_DELAY);
            }
            else
            {
                log_i("error");
            }
        }
        audio.loop();
        if (!audio.isRunning())
        {
            sleep(1);
        }
    }
}

void oximeter_task(void *parameter)
{
    /* task untuk oximeter */
    CreateOxiQueues();
    Wire.begin(I2C_SDA, I2C_SCL);
    // Initialize sensor
    if (!particleSensor.begin(Wire, I2C_SPEED_FAST)) // Use default I2C port, 400kHz speed
    {
        Serial.println("MAX30105 was not found. Please check wiring/power. ");
        while (1)
            ;
    }

    particleSensor.setup();                    // Configure sensor with default settings
    particleSensor.setPulseAmplitudeRed(0x0A); // Turn Red LED to low to indicate sensor is running
    particleSensor.setPulseAmplitudeGreen(0);

    if (!oximeterQueue)
    {
        log_e("queues oximeter are not initialized");
        while (true)
        {
            ;
        } // endless loop
    }
    while (true)
    {

        long irValue = particleSensor.getIR();

        if (checkForBeat(irValue) == true)
        {
            // We sensed a beat!
            long delta = millis() - lastBeat;
            lastBeat = millis();

            beatsPerMinute = 60 / (delta / 1000.0);

            if (beatsPerMinute < 255 && beatsPerMinute > 20)
            {
                rates[rateSpot++] = (byte)beatsPerMinute; // Store this reading in the array
                rateSpot %= RATE_SIZE;                    // Wrap variable

                // Take average of readings
                beatAvg = 0;
                for (byte x = 0; x < RATE_SIZE; x++)
                    beatAvg += rates[x];
                beatAvg /= RATE_SIZE;
            }
        }

        Serial.print("IR=");
        Serial.print(irValue);
        Serial.print(", BPM=");
        Serial.print(beatsPerMinute);
        Serial.print(", Avg BPM=");
        Serial.print(beatAvg);

        if (irValue < 50000)
        {
            Serial.print(" No finger?");
            neopixelWrite(RGB_BUILTIN, 0, RGB_BRIGHTNESS, 0);
        }

        neopixelWrite(RGB_BUILTIN, 0, 0, RGB_BRIGHTNESS);
        Serial.println();

    }
}

void audioInit()
{
    xTaskCreatePinnedToCore(
        audioTask,             /* Function to implement the task */
        "audioplay",           /* Name of the task */
        5000,                  /* Stack size in words */
        NULL,                  /* Task input parameter */
        2 | portPRIVILEGE_BIT, /* Priority of the task */
        NULL,                  /* Task handle. */
        1                      /* Core where the task should run */
    );
}

void oximeter_init()
{
    /* frist should create thread */

    xTaskCreatePinnedToCore(
        oximeter_task,
        "oxitask",
        5000,
        NULL,
        2 | portPRIVILEGE_BIT,
        NULL,
        1);
}

audioMessage transmitReceive(audioMessage msg)
{
    xQueueSend(audioSetQueue, &msg, portMAX_DELAY);
    if (xQueueReceive(audioGetQueue, &audioRxMessage, portMAX_DELAY) == pdPASS)
    {
        if (msg.cmd != audioRxMessage.cmd)
        {
            log_e("wrong reply from message queue");
        }
    }
    return audioRxMessage;
}

void audioSetVolume(uint8_t vol)
{
    audioTxMessage.cmd = SET_VOLUME;
    audioTxMessage.value = vol;
    audioMessage RX = transmitReceive(audioTxMessage);
}

uint8_t audioGetVolume()
{
    audioTxMessage.cmd = GET_VOLUME;
    audioMessage RX = transmitReceive(audioTxMessage);
    return RX.ret;
}

bool audioConnecttohost(const char *host)
{
    audioTxMessage.cmd = CONNECTTOHOST;
    audioTxMessage.txt = host;
    audioMessage RX = transmitReceive(audioTxMessage);
    return RX.ret;
}

bool audioConnecttoSD(const char *filename)
{
    audioTxMessage.cmd = CONNECTTOSD;
    audioTxMessage.txt = filename;
    audioMessage RX = transmitReceive(audioTxMessage);
    return RX.ret;
}

//****************************************************************************************
//                                   S E T U P                                           *
//****************************************************************************************

void setup()
{
    Serial.begin(115200);
    WiFi.begin(ssid.c_str(), password.c_str());

    while (WiFi.status() != WL_CONNECTED)
        delay(1500);

    audioInit();
    oximeter_init();

    audioConnecttohost("https://github.com/faoziaziz/as-hot/raw/main/Dewa20-200120-20Pangeran20Cinta.mp3");
    audioSetVolume(21);
    log_i("current volume is: %d", audioGetVolume());

    // Turn off Green LED
}

//****************************************************************************************
//                                   L O O P                                             *
//****************************************************************************************

void loop()
{
}
//*****************************************************************************************
//                                  E V E N T S                                           *
//*****************************************************************************************

void audio_info(const char *info)
{
    Serial.print("info        ");
    Serial.println(info);
}