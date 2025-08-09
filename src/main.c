/****************************************************************************
 * main.c
 * openacousticdevices.info
 * October 2022
 *****************************************************************************/

#include <time.h>
#include <math.h>
#include <ctype.h>
#include <stdio.h>
#include <string.h>
#include <stdbool.h>

#include "em_core.h"
#include "em_gpio.h"

#include "ff.h"
#include "pinouts.h"
#include "audiomoth.h"
#include "nmeaparser.h"
#include "butterworth.h"
#include "configparser.h"
#include "gpsinterface.h"
#include "gpsutilities.h"
#include "configchecker.h"

/* Debug constants */

#define LOG                                     true

/* GPS pins */

#define MAGNETIC_SWITCH_PIN                     9
#define MAGNETIC_SWITCH_GPIOPORT                gpioPortB

#define GPS_ENABLE_PIN                          7
#define GPS_ENABLE_GPIOPORT                     gpioPortA

/* GPS fix constants */

#define MINIMUM_VALID_PPS_TO_SET_TIME           5
#define MINIMUM_VALID_RMC_TO_SET_TIME           5

#define GPS_TICK_EVENTS_PER_SECOND              12

/* Sleep and LED constants */

#define LOW_BATTERY_LED_FLASHES                 10

#define SHORT_LED_FLASH_DURATION                100
#define LONG_LED_FLASH_DURATION                 500

#define WAITING_LED_FLASH_DURATION              10
#define WAITING_LED_FLASH_INTERVAL              2000

#define MINIMUM_LED_FLASH_INTERVAL              500

#define SHORT_WAIT_INTERVAL                     100
#define DEFAULT_WAIT_INTERVAL                   1000

/* Magnetic switch constants */

#define MAGNETIC_SWITCH_WAIT_MULTIPLIER         2
#define MAGNETIC_SWITCH_CHANGE_FLASHES          10

/* Log constants */

#define LOG_BUFFER_LENGTH                       128

/* File constants */

#define FILENAME_LENGTH                         128
#define GUANO_BUFFER_LENGTH                     1024

#define LAST_RMC_BUFFER_LENGTH                  128
#define FILE_WRITE_BUFFER_LENGTH                256
#define FILE_READ_BUFFER_LENGTH                 512
#define MAX_FILE_READ_CHARACTERS                (1024 * 1024)

#define NUMBER_OF_BYTES_IN_SAMPLE               2

#define MAXIMUM_WAV_DATA_SIZE                   (UINT32_MAX - 16 * 1024 * 1024)

/* Microphone constants */

#define ACQUISITION_CYCLES                      16
#define CLOCK_DIVIDER                           4

/* Sample rate constants */

#define BIT_SHIFT_AT_UNFILTERED_SAMPLE_RATE     3
#define MAXIMUM_FILTERED_SAMPLE_RATE            96000
#define MAXIMUM_REFERENCE_SAMPLE_RATE           384000

/* Useful time constants */

#define SECONDS_IN_MINUTE                       60
#define SECONDS_IN_HOUR                         (60 * SECONDS_IN_MINUTE)
#define SECONDS_IN_DAY                          (24 * SECONDS_IN_HOUR)
#define MINUTES_IN_HOUR                         60
#define MINUTES_IN_SIX_HOURS                    360
#define MINUTES_IN_DAY                          (SECONDS_IN_DAY / SECONDS_IN_MINUTE)

#define MILLISECONDS_IN_SECOND                  1000
#define MICROSECONDS_IN_SECOND                  1000000

#define YEAR_OFFSET                             1900
#define MONTH_OFFSET                            1

#define START_OF_CENTURY                        946684800
#define MIDPOINT_OF_CENTURY                     2524608000

/* Useful type constants */

#define UINT32_SIZE_IN_BYTES                    4

/* Sunrise and sunset constant */

#define MAXIMUM_SUNRISE_SUNSET_SHIFT            300

/* SRAM buffer constants */

#define NUMBER_OF_BUFFERS                       8
#define EXTERNAL_SRAM_SIZE_IN_SAMPLES           (AM_EXTERNAL_SRAM_SIZE_IN_BYTES / 2)
#define NUMBER_OF_SAMPLES_IN_BUFFER             (EXTERNAL_SRAM_SIZE_IN_SAMPLES / NUMBER_OF_BUFFERS)

/* Supply monitor constant */

#define MINIMUM_SUPPLY_VOLTAGE                  2800

/* Recording error constant */

#define MAXIMUM_NUMBER_OF_RECORDING_ERRORS      5

/* DC filter constants */

#define LOW_DC_BLOCKING_FREQ                    8
#define DEFAULT_DC_BLOCKING_FREQ                48

/* WAV header constants */

#define PCM_FORMAT                              1
#define RIFF_ID_LENGTH                          4
#define LENGTH_OF_ARTIST                        32
#define LENGTH_OF_COMMENT                       384

/* GPS constants */

#define MINUTES_IN_DEGREE                       60
#define GPS_LOCATION_PRECISION                  1000000

#define MINIMUM_GPS_POWER_DOWN_TIME             4

/* Useful macro */

#define FLASH_LED(led, duration) { \
    AudioMoth_set ## led ## LED(true); \
    AudioMoth_delay(duration); \
    AudioMoth_set ## led ## LED(false); \
}

#define FLASH_REPEAT_LED(led, repeats, duration) { \
    for (uint32_t i = 0; i < repeats; i += 1) { \
        AudioMoth_set ## led ## LED(true); \
        AudioMoth_delay(duration); \
        AudioMoth_set ## led ## LED(false); \
        AudioMoth_delay(duration); \
    } \
}

#define FLASH_LED_ON_FALSE(fn) { \
    bool success = (fn); \
    if (!success) { \
        FLASH_LED(Both, LONG_LED_FLASH_DURATION) \
    } \
}

#define RETURN_ON_FALSE(fn) { \
    bool success = (fn); \
    if (!success) return; \
}

#define TURN_LED_OFF_AND_RETURN_ERROR_ON_FALSE(fn) { \
    bool success = (fn); \
    if (!success) { \
        AudioMoth_setBothLED(false); \
        return SDCARD_WRITE_ERROR; \
    } \
}

#define TURN_LED_OFF_AND_RETURN_ERROR_ON_FILE_ERROR(fn) { \
    FRESULT res = (fn); \
    if (res != FR_OK) { \
        AudioMoth_setBothLED(false); \
        return SDCARD_WRITE_ERROR; \
    } \
}

#define SAVE_SWITCH_POSITION_AND_POWER_DOWN(milliseconds) { \
    *previousSwitchPosition = switchPosition; \
    AudioMoth_powerDownAndWakeMilliseconds(milliseconds); \
}

#define SERIAL_NUMBER                           "%08X%08X"

#define FORMAT_SERIAL_NUMBER(src)               (unsigned int)*((uint32_t*)src + 1),  (unsigned int)*((uint32_t*)src)

#define ABS(a)                                  ((a) < (0) ? (-a) : (a))

#define MAX(a,b)                                (((a) > (b)) ? (a) : (b))

#define MIN(a,b)                                (((a) < (b)) ? (a) : (b))

#define ROUNDED_DIV(a, b)                       (((a) + ((b)/2)) / (b))

#define ROUNDED_UP_DIV(a, b)                    (((a) + (b) - 1) / (b))

#define UNSIGNED_ROUND(n, d)                    ((d) * (((n) + (d) / 2) / (d)))

/* Fix state enumeration */

typedef enum {SUCCESS, CANCELLED_BY_SWITCH, CANCELLED_BY_MAGNET, TIMEOUT} AM_fixState_t;

/* Device state enumeration */

typedef enum {RESET, WAITING_FOR_INITIAL_CONFIG, WAITING_FOR_MAGNETIC_TRIGGER_BEFORE_INITIAL_GPS_FIX, WAITING_FOR_INITIAL_GPS_FIX, WAITING_FOR_MAGNETIC_TRIGGER_BEFORE_RECORDING, RECORDING, POWER_DOWN_AFTER_RECORDING, POWER_DOWN_AFTER_RECORDING_CANCELLED_BY_MAGNETIC_SWITCH} AM_acquisitionState_t;

/* Recording state enumeration */

typedef enum {RECORDING_OKAY, START_TIMEOUT, CANCELLED_BEFORE_START_BY_MICROPHONE_CHANGE, CANCELLED_BEFORE_START_BY_SWITCH, CANCELLED_BEFORE_START_BY_MAGNET, CANCELLED_BEFORE_START_DUE_TO_LOW_SUPPLY_VOLTAGE, CANCELLED_DURING_RECORDING_BY_MICROPHONE_CHANGE, CANCELLED_DURING_RECORDING_BY_SWITCH, CANCELLED_DURING_RECORDING_BY_MAGNET, SDCARD_WRITE_ERROR, SUPPLY_VOLTAGE_LOW, FILE_SIZE_LIMITED} AM_recordingState_t;

/* Convert recording period type */

typedef CP_recordingPeriod_t recordingPeriod_t;

/* WAV header */

#pragma pack(push, 1)

typedef struct {
    char id[RIFF_ID_LENGTH];
    uint32_t size;
} chunk_t;

typedef struct {
    chunk_t icmt;
    char comment[LENGTH_OF_COMMENT];
} icmt_t;

typedef struct {
    chunk_t iart;
    char artist[LENGTH_OF_ARTIST];
} iart_t;

typedef struct {
    uint16_t format;
    uint16_t numberOfChannels;
    uint32_t samplesPerSecond;
    uint32_t bytesPerSecond;
    uint16_t bytesPerCapture;
    uint16_t bitsPerSample;
} wavFormat_t;

typedef struct {
    chunk_t riff;
    char format[RIFF_ID_LENGTH];
    chunk_t fmt;
    wavFormat_t wavFormat;
    chunk_t list;
    char info[RIFF_ID_LENGTH];
    icmt_t icmt;
    iart_t iart;
    chunk_t data;
} wavHeader_t;

#pragma pack(pop)

static wavHeader_t wavHeader = {
    .riff = {.id = "RIFF", .size = 0},
    .format = "WAVE",
    .fmt = {.id = "fmt ", .size = sizeof(wavFormat_t)},
    .wavFormat = {.format = PCM_FORMAT, .numberOfChannels = 1, .samplesPerSecond = 0, .bytesPerSecond = 0, .bytesPerCapture = 2, .bitsPerSample = 16},
    .list = {.id = "LIST", .size = RIFF_ID_LENGTH + sizeof(icmt_t) + sizeof(iart_t)},
    .info = "INFO",
    .icmt = {.icmt.id = "ICMT", .icmt.size = LENGTH_OF_COMMENT, .comment = ""},
    .iart = {.iart.id = "IART", .iart.size = LENGTH_OF_ARTIST, .artist = ""},
    .data = {.id = "data", .size = 0}
};

static void setHeaderDetails(wavHeader_t *wavHeader, uint32_t sampleRate, uint32_t numberOfSamples, uint32_t guanoDataSize) {

    wavHeader->wavFormat.samplesPerSecond = sampleRate;
    wavHeader->wavFormat.bytesPerSecond = NUMBER_OF_BYTES_IN_SAMPLE * sampleRate;
    wavHeader->data.size = NUMBER_OF_BYTES_IN_SAMPLE * numberOfSamples;
    wavHeader->riff.size = NUMBER_OF_BYTES_IN_SAMPLE * numberOfSamples + sizeof(wavHeader_t) + guanoDataSize - sizeof(chunk_t);

}

static void setHeaderComment(wavHeader_t *wavHeader, CP_configSettings_t *configSettings, uint32_t currentTime, uint8_t *serialNumber, AM_extendedBatteryState_t extendedBatteryState, int32_t temperature, bool externalMicrophone, AM_recordingState_t recordingState) {

    time_t rawTime = currentTime;

    struct tm *time = gmtime(&rawTime);

    /* Format artist field */

    char *artist = wavHeader->iart.artist;

    sprintf(artist, "AudioMoth " SERIAL_NUMBER, FORMAT_SERIAL_NUMBER(serialNumber));

    /* Format comment field */

    char *comment = wavHeader->icmt.comment;

    comment += sprintf(comment, "Recorded at %02d:%02d:%02d %02d/%02d/%04d (UTC) by %s ", time->tm_hour, time->tm_min, time->tm_sec, time->tm_mday, MONTH_OFFSET + time->tm_mon, YEAR_OFFSET + time->tm_year, artist);

    if (externalMicrophone) {

        comment += sprintf(comment, "using external microphone ");

    }

    static char *gainSettings[5] = {"low", "low-medium", "medium", "medium-high", "high"};

    comment += sprintf(comment, "at %s gain while battery was ", gainSettings[configSettings->gain]);

    if (extendedBatteryState == AM_EXT_BAT_LOW) {

        comment += sprintf(comment, "less than 2.5V");

    } else if (extendedBatteryState >= AM_EXT_BAT_FULL) {

        comment += sprintf(comment, "greater than 4.9V");

    } else {

        uint32_t batteryVoltage = extendedBatteryState + AM_EXT_BAT_STATE_OFFSET / AM_BATTERY_STATE_INCREMENT;

        comment += sprintf(comment, "%01ld.%01ldV", batteryVoltage / 10, batteryVoltage % 10);

    }

    char *sign = temperature < 0 ? "-" : "";

    uint32_t temperatureInDecidegrees = ROUNDED_DIV(ABS(temperature), 100);

    comment += sprintf(comment, " and temperature was %s%ld.%ldC.", sign, temperatureInDecidegrees / 10, temperatureInDecidegrees % 10);

    if (recordingState != RECORDING_OKAY) {

        comment += sprintf(comment, " Recording stopped due to ");

        if (recordingState == CANCELLED_DURING_RECORDING_BY_MICROPHONE_CHANGE) {

            comment += sprintf(comment, "microphone change.");

        } else if (recordingState == CANCELLED_DURING_RECORDING_BY_SWITCH) {

            comment += sprintf(comment, "switch position change.");

        } else if (recordingState == CANCELLED_DURING_RECORDING_BY_MAGNET) {

            comment += sprintf(comment, "magnetic switch.");

        } else if (recordingState == SUPPLY_VOLTAGE_LOW) {

            comment += sprintf(comment, "low voltage.");

        } else if (recordingState == FILE_SIZE_LIMITED) {

            comment += sprintf(comment, "file size limit.");

        }

    }

}

/* Function to write the GUANO data */

static uint32_t writeGuanoData(char *buffer, CP_configSettings_t *configSettings, uint32_t currentTime, int32_t *gpsLastFixLatitude, int32_t *gpsLastFixLongitude, uint8_t *firmwareDescription, uint8_t *firmwareVersion, uint8_t *serialNumber, char *filename, AM_extendedBatteryState_t extendedBatteryState, int32_t temperature) {
    
    uint32_t length = sprintf(buffer, "guan") + UINT32_SIZE_IN_BYTES;

    /* General information */
    
    length += sprintf(buffer + length, "GUANO|Version:1.0\nMake:Open Acoustic Devices\nModel:AudioMoth\nSerial:" SERIAL_NUMBER "\n", FORMAT_SERIAL_NUMBER(serialNumber));

    length += sprintf(buffer + length, "Firmware Version:%s (%u.%u.%u)\n", firmwareDescription, firmwareVersion[0], firmwareVersion[1], firmwareVersion[2]);

    /* Timestamp */

    time_t rawTime = currentTime;

    struct tm time;

    gmtime_r(&rawTime, &time);

    length += sprintf(buffer + length, "Timestamp:%04d-%02d-%02dT%02d:%02d:%02d", YEAR_OFFSET + time.tm_year, MONTH_OFFSET + time.tm_mon, time.tm_mday, time.tm_hour, time.tm_min, time.tm_sec);

    length += sprintf(buffer + length, "Z\n");

    /* Location position and source */

    char *latitudeSign = *gpsLastFixLatitude < 0 ? "-" : "";

    char *longitudeSign = *gpsLastFixLongitude < 0 ? "-" : "";

    length += sprintf(buffer + length, "Loc Position:%s%ld.%06ld %s%ld.%06ld\nOAD|Loc Source:GPS\n", latitudeSign, ABS(*gpsLastFixLatitude) / GPS_LOCATION_PRECISION, ABS(*gpsLastFixLatitude) % GPS_LOCATION_PRECISION, longitudeSign, ABS(*gpsLastFixLongitude) / GPS_LOCATION_PRECISION, ABS(*gpsLastFixLongitude) % GPS_LOCATION_PRECISION);

    /* Filename */

    char *start = strchr(filename, '/');

    length += sprintf(buffer + length, "Original Filename:%s\n", start ? start + 1 : filename);

    /* Recording settings */

    length += sprintf(buffer + length, "OAD|Recording Settings:%lu GAIN %u", configSettings->sampleRate, configSettings->gain);

    if (configSettings->enableLowGainRange) length += sprintf(buffer + length, " LGR");

    if (configSettings->disable48HzDCBlockingFilter) length += sprintf(buffer + length, " D48");

    /* Battery and temperature */

    uint32_t batteryVoltage = extendedBatteryState == AM_EXT_BAT_LOW ? 24 : extendedBatteryState >= AM_EXT_BAT_FULL ? 50 : extendedBatteryState + AM_EXT_BAT_STATE_OFFSET / AM_BATTERY_STATE_INCREMENT;

    length += sprintf(buffer + length, "\nOAD|Battery Voltage:%01lu.%01lu\n", batteryVoltage / 10, batteryVoltage % 10);
    
    char *temperatureSign = temperature < 0 ? "-" : "";

    uint32_t temperatureInDecidegrees = ROUNDED_DIV(ABS(temperature), 100);

    length += sprintf(buffer + length, "Temperature Int:%s%lu.%lu", temperatureSign, temperatureInDecidegrees / 10, temperatureInDecidegrees % 10);

    /* Set GUANO chunk size */

    *(uint32_t*)(buffer + RIFF_ID_LENGTH) = length - sizeof(chunk_t);;

    return length;

}

/* Configuration data structure */

static CP_configSettings_t tempConfigSettings;

static float *latitude = (float*)(AM_BACKUP_DOMAIN_START_ADDRESS);

static float *longitude = (float*)(AM_BACKUP_DOMAIN_START_ADDRESS + 4);

static int32_t *gpsLastFixLatitude = (int32_t*)(AM_BACKUP_DOMAIN_START_ADDRESS + 8);

static int32_t *gpsLastFixLongitude = (int32_t*)(AM_BACKUP_DOMAIN_START_ADDRESS + 12);

static uint32_t *timeOfNextRecording = (uint32_t*)(AM_BACKUP_DOMAIN_START_ADDRESS + 16);

static uint32_t *previousSwitchPosition = (uint32_t*)(AM_BACKUP_DOMAIN_START_ADDRESS + 20);

static uint32_t *durationOfNextRecording = (uint32_t*)(AM_BACKUP_DOMAIN_START_ADDRESS + 24);

static uint32_t *currentAcquisitionState = (uint32_t*)(AM_BACKUP_DOMAIN_START_ADDRESS + 28);

static uint32_t *numberOfRecordingErrors = (uint32_t*)(AM_BACKUP_DOMAIN_START_ADDRESS + 32);

static uint32_t *timeOfNextInitalGPSAttempt = (uint32_t*)(AM_BACKUP_DOMAIN_START_ADDRESS + 36);

static uint32_t *timeOfNextSunriseAndSunsetCalculation = (uint32_t*)(AM_BACKUP_DOMAIN_START_ADDRESS + 40);

static uint32_t *previousRecordingState = (uint32_t*)(AM_BACKUP_DOMAIN_START_ADDRESS + 44);

static uint32_t *lastPowerDownTimeOfGPS = (uint32_t*)(AM_BACKUP_DOMAIN_START_ADDRESS + 48);

static CP_configSettings_t *configSettings = (CP_configSettings_t*)(AM_BACKUP_DOMAIN_START_ADDRESS + 52);

/* Firmware version and description */

static uint8_t firmwareVersion[AM_FIRMWARE_VERSION_LENGTH] = {1, 2, 1};

static uint8_t firmwareDescription[AM_FIRMWARE_DESCRIPTION_LENGTH] = "AudioMoth-GPS-Sync";

/* Functions of copy to and from the backup domain */

static void copyFromBackupDomain(uint8_t *dst, uint32_t *src, uint32_t length) {

    for (uint32_t i = 0; i < length; i += 1) {
        *(dst + i) = *((uint8_t*)src + i);
    }

}

static void copyToBackupDomain(uint32_t *dst, uint8_t *src, uint32_t length) {

    uint32_t value = 0;

    for (uint32_t i = 0; i < length / 4; i += 1) {
        *(dst + i) = *((uint32_t*)src + i);
    }

    for (uint32_t i = 0; i < length % 4; i += 1) {
        value = (value << 8) + *(src + length - 1 - i);
    }

    *(dst + length / 4) = value;

}

/* GPS RMC and PPS variables */

static volatile bool receivedRMC;

static volatile bool receivedPPS;

static volatile uint32_t sampleCount;

static volatile uint32_t lastSampleCount;

static volatile uint32_t sampleCountInLastSecond;

/* Recording interrupt flags */

static volatile bool microphoneChanged;

static volatile bool magneticSwitchChanged;

static volatile bool switchPositionChanged;

/* Recording state variables */

static volatile bool recording;

static volatile bool toggleGreenLED;

static volatile bool nextStateGreenLED;

static volatile bool enableRecordingOnNextPPS;

/* Sunrise and sunset variables */

static uint32_t sunriseMinutes;

static uint32_t sunsetMinutes;

/* NMEA parser structure */

static NMEA_parserResultRMC_t parserResultRMC;

/* File variables */

static FIL filePPS;

static FIL fileSAMPLES;

static char foldername[FILENAME_LENGTH];

static char filenamePPS[FILENAME_LENGTH];

static char filenameSAMPLES[FILENAME_LENGTH];

/* Generic file write variables */

static UINT bw;

static char fileWriteBuffer[FILE_WRITE_BUFFER_LENGTH];

static char lastRMCBuffer[LAST_RMC_BUFFER_LENGTH];

/* Log variables */

static bool fileSystemEnabled;

static char *logFilename = "LOG.TXT";

static char *deviceInfoFilename = "DEVICE.TXT";

static char logBuffer[LOG_BUFFER_LENGTH];

/* Temporary buffer */

static char buffer[LOG_BUFFER_LENGTH];

/* GUANO buffer */

static char guanoBuffer[GUANO_BUFFER_LENGTH];

/* File read variables */

static char fileReadBuffer[FILE_READ_BUFFER_LENGTH];

/* SRAM buffer variables */

static volatile uint32_t writeBuffer;

static volatile uint32_t writeBufferIndex;

static volatile uint32_t writeBufferCount;

static int16_t* buffers[NUMBER_OF_BUFFERS];

/* Current PPS and RMC variables */

static volatile uint32_t currentPPSTime;

static volatile uint32_t currentPPSMilliSeconds;

static volatile uint32_t currentRMCTime;

static volatile uint32_t currentRMCMilliSeconds;

static volatile uint32_t currentPPSMicrophoneCounter;

/* GPS fix variables */

static bool gpsEnableLED;

static volatile bool gpsPPSEvent;

static volatile bool gpsMessageEvent;

static uint32_t gpsTickEventCount = 1;

static uint32_t gpsTickEventModulo = GPS_TICK_EVENTS_PER_SECOND;

/* Filter components */

static BW_filter_t dcFilter;

static BW_filterCoefficients_t dcFilterCoefficients;

/* Required time zone handler */

inline void AudioMoth_timezoneRequested(int8_t *timezoneHours, int8_t *timezoneMinutes) { }

/* Required interrupt handlers */

inline void AudioMoth_handleSwitchInterrupt(void) {
    
    switchPositionChanged = true;

}

inline void AudioMoth_handleMicrophoneChangeInterrupt(void) { 

    microphoneChanged = true;

}

inline void AudioMoth_handleDirectMemoryAccessInterrupt(bool primaryChannel, int16_t **nextBuffer) { }

/* Required USB message handlers */

inline void AudioMoth_usbFirmwareVersionRequested(uint8_t **firmwareVersionPtr) {

    *firmwareVersionPtr = firmwareVersion;

}

void AudioMoth_usbFirmwareDescriptionRequested(uint8_t **firmwareDescriptionPtr) {

    *firmwareDescriptionPtr = firmwareDescription;

}

inline void AudioMoth_usbApplicationPacketRequested(uint32_t messageType, uint8_t *transmitBuffer, uint32_t size) { }

inline void AudioMoth_usbApplicationPacketReceived(uint32_t messageType, uint8_t *receiveBuffer, uint8_t *transmitBuffer, uint32_t size) { }

/* Required GPS handlers */

inline void GPSInterface_handlePulsePerSecond(uint32_t counter, uint32_t counterPeriod, uint32_t counterFrequency) {

    /* Record the ADC counter value */

    currentPPSMicrophoneCounter = counter;

    /* Record the current PPS time */

    AudioMoth_getTime((uint32_t*)&currentPPSTime, (uint32_t*)&currentPPSMilliSeconds);

    /* Update the sample count */

    sampleCountInLastSecond = sampleCount - lastSampleCount;

    lastSampleCount = sampleCount;

    /* Update the green LED */

    if (toggleGreenLED) AudioMoth_setGreenLED(nextStateGreenLED);

    /* Update flags */

    if (enableRecordingOnNextPPS) recording = true;
    
    receivedPPS = true;

}

inline void GPSInterface_handleTick() { 

    if (gpsTickEventCount == 0) {

        gpsTickEventModulo = GPS_TICK_EVENTS_PER_SECOND;
    
        if (gpsPPSEvent) {

            gpsTickEventModulo /= 3;

        } else if (gpsMessageEvent) {

            gpsTickEventModulo /= 2;

        }

        gpsMessageEvent = false;

        gpsPPSEvent = false;

    }

    if (gpsEnableLED) AudioMoth_setRedLED(gpsTickEventCount % gpsTickEventModulo == 0);

    gpsTickEventCount = (gpsTickEventCount + 1) % GPS_TICK_EVENTS_PER_SECOND;

}

inline void GPSInterface_handleReceivedByte(uint8_t byte) {

    NMEA_parserStatus_t parserStatusRMC = NMEAParser_parseRMC(byte, &parserResultRMC);

    if (parserStatusRMC == NMEA_SUCCESS) {

        /* Record time */

        AudioMoth_getTime((uint32_t*)&currentRMCTime, (uint32_t*)&currentRMCMilliSeconds);

        /* Determine next green LED state */

        uint32_t timestampInRMC;

        GPSUtilities_getTime(&parserResultRMC, &timestampInRMC);

        nextStateGreenLED = timestampInRMC & 0x01;

        /* Update flag */

        receivedRMC = true;

    }

}

inline void AudioMoth_handleMicrophoneInterrupt(int16_t sample) {

    /* Apply DC filter */

    if (configSettings->sampleRate <= MAXIMUM_FILTERED_SAMPLE_RATE) {

        float filterOutput = Butterworth_applyHighPassFilter(sample, &dcFilter, &dcFilterCoefficients);

        if (filterOutput > INT16_MAX) {
            
            filterOutput = INT16_MAX;

        } else if (filterOutput < -INT16_MAX) {
            
            filterOutput = -INT16_MAX;

        }

        sample = (int16_t)filterOutput;

    } else {
        
        sample <<= BIT_SHIFT_AT_UNFILTERED_SAMPLE_RATE;

    }

    /* Output value if recording */

    if (recording) {

        buffers[writeBuffer][writeBufferIndex] = sample;

        writeBufferIndex += 1;

        if (writeBufferIndex == NUMBER_OF_SAMPLES_IN_BUFFER) {

            writeBuffer = (writeBuffer + 1) & (NUMBER_OF_BUFFERS - 1);

            writeBufferCount += 1;

            writeBufferIndex = 0;

        }

        sampleCount += 1;

    }

}

/* Functions to power up and power down GPS */

static void powerUpGPS() {

    GPIO_PinModeSet(GPS_ENABLE_GPIOPORT, GPS_ENABLE_PIN, gpioModePushPull, 0);

}

static void powerDownGPS() {

    GPIO_PinModeSet(GPS_ENABLE_GPIOPORT, GPS_ENABLE_PIN, gpioModeDisabled, 0);

}

/* Interrupt handler for magnetic switch */

void GPIO_ODD_IRQHandler(void) {

    /* Get the interrupt mask */

    uint32_t interruptMask = GPIO_IntGet();

    /* Clear the interrupt */

    GPIO_IntClear(interruptMask);

    /* Handle the interrupt */

    if (interruptMask & (1 << MAGNETIC_SWITCH_PIN)) magneticSwitchChanged = true;

}

/* Functions to handle the magnetic switch */

static void enableMagneticSwitch() {
        
    GPIO_PinModeSet(MAGNETIC_SWITCH_GPIOPORT, MAGNETIC_SWITCH_PIN, gpioModeInputPull, 1);

}

static void enableMagneticSwitchInterrupt() {

    GPIO_IntConfig(MAGNETIC_SWITCH_GPIOPORT, MAGNETIC_SWITCH_PIN, false, true, true);

    NVIC_ClearPendingIRQ(GPIO_ODD_IRQn);

    NVIC_EnableIRQ(GPIO_ODD_IRQn);

}

static bool isMagneticSwitchClosed() {

    return GPIO_PinInGet(MAGNETIC_SWITCH_GPIOPORT, MAGNETIC_SWITCH_PIN) == 0;

}

/* GPS format conversion */

static int32_t convertToDecimalDegrees(uint32_t degrees, uint32_t minutes, uint32_t tenThousandths, char direction) {

    int32_t value = degrees * GPS_LOCATION_PRECISION;

    value += ROUNDED_DIV(minutes * GPS_LOCATION_PRECISION, MINUTES_IN_DEGREE);

    value += ROUNDED_DIV(tenThousandths * (GPS_LOCATION_PRECISION / 10000), MINUTES_IN_DEGREE); 

    if (direction == 'S' || direction == 'W') value *= -1;

    return value;

}

/* Function to acquire GPS time */

static AM_fixState_t setTimeFromGPS(uint32_t timeout, uint32_t *acquisitionTime) {

    gpsEnableLED = true;

    *acquisitionTime = 0;

    /* Main loop. Escape if fix has set the time or if timeout has occurred */

    uint32_t validRMC = 0;

    uint32_t validPPS = 0;

    bool timeHasBeenSet = false;

    bool cancelledBySwitch = false;

    bool cancelledByMagnet = false;

    uint32_t timeToBeSetOnNextPPS = 0;

    while (!timeHasBeenSet && !cancelledBySwitch && !cancelledByMagnet && *acquisitionTime < timeout) {

        AudioMoth_getTime(acquisitionTime, NULL);

        if (receivedRMC) {
            
            /* Set GPS LED flag */

            gpsMessageEvent = true;

            /* Check RMC is valid */

            uint32_t timeSincePPS = (currentRMCTime - currentPPSTime) * MILLISECONDS_IN_SECOND + currentRMCMilliSeconds - currentPPSMilliSeconds;

            if (parserResultRMC.status == 'A' && parserResultRMC.milliseconds == 0 && timeSincePPS < MILLISECONDS_IN_SECOND) {

                validRMC += 1;

                if (validRMC > MINIMUM_VALID_RMC_TO_SET_TIME) {

                    uint32_t timestampInRMC;

                    GPSUtilities_getTime(&parserResultRMC, &timestampInRMC);

                    timeToBeSetOnNextPPS = timestampInRMC + 1;

                }

            } else {

                timeToBeSetOnNextPPS = 0;

                validRMC = 0;

            }

            /* Reset flag */

            receivedRMC = false;

        }

        if (receivedPPS) {

            /* Set GPS LED flag */
            
            gpsPPSEvent = true;

            /* Check PPS is valid */

            uint32_t timeSinceRMC = (currentPPSTime - currentRMCTime) * MILLISECONDS_IN_SECOND + currentPPSMilliSeconds - currentRMCMilliSeconds;

            if (timeSinceRMC < MILLISECONDS_IN_SECOND) {

                validPPS += 1;

                if (validPPS > MINIMUM_VALID_PPS_TO_SET_TIME && timeToBeSetOnNextPPS > 0) {

                    AudioMoth_setTime(timeToBeSetOnNextPPS, 0);

                    currentPPSTime = timeToBeSetOnNextPPS;

                    currentPPSMilliSeconds = 0;

                    timeHasBeenSet = true;

                }

            } else {

                validPPS = 0;

            }

            /* Reset flag */

            receivedPPS = false;

        }

        /* Check for cancellation from magnetic or position switch */
        
        cancelledBySwitch = AudioMoth_getSwitchPosition() == AM_SWITCH_USB;

        cancelledByMagnet = configSettings->enableMagneticSwitch && isMagneticSwitchClosed();

        /* Sleep and feed watch dog */

        AudioMoth_feedWatchdog();

        AudioMoth_sleep();

    }

    /* Disable LED */

    gpsEnableLED = false;

    /* Determine return value */

    if (cancelledBySwitch) return CANCELLED_BY_SWITCH;

    if (cancelledByMagnet) return CANCELLED_BY_MAGNET;

    if (timeHasBeenSet) return SUCCESS;

    return TIMEOUT;

}

/* Logging function */

static void writeDeviceInformation() {

    if (fileSystemEnabled == false) fileSystemEnabled = AudioMoth_enableFileSystem(AM_SD_CARD_HIGH_SPEED);

    if (fileSystemEnabled == false) return;

    RETURN_ON_FALSE(AudioMoth_openFile(deviceInfoFilename));

    uint32_t length = sprintf(logBuffer, "Device: %08X%08X\r\n", (unsigned int)*((uint32_t*)(uint8_t*)AM_UNIQUE_ID_START_ADDRESS + 1), (unsigned int)*((uint32_t*)(uint8_t*)AM_UNIQUE_ID_START_ADDRESS));

    RETURN_ON_FALSE(AudioMoth_writeToFile(logBuffer, length));

    length = sprintf(logBuffer, "Firmware version: %d.%d.%d\r\n", firmwareVersion[0], firmwareVersion[1], firmwareVersion[2]);

    RETURN_ON_FALSE(AudioMoth_writeToFile(logBuffer, length));

    length = sprintf(logBuffer, "Firmware description: %s\r\n", firmwareDescription);

    RETURN_ON_FALSE(AudioMoth_writeToFile(logBuffer, length));

    RETURN_ON_FALSE(AudioMoth_closeFile());

}

static void writeLog(uint32_t currentTime, char *message) {

    if (fileSystemEnabled == false) fileSystemEnabled = AudioMoth_enableFileSystem(AM_SD_CARD_HIGH_SPEED);

    if (fileSystemEnabled == false) return;

    RETURN_ON_FALSE(AudioMoth_appendFile(logFilename));

    time_t rawTime = currentTime;

    struct tm *time = gmtime(&rawTime);

    uint32_t length = sprintf(logBuffer, "%02d/%02d/%04d %02d:%02d:%02d: ", time->tm_mday, MONTH_OFFSET + time->tm_mon, YEAR_OFFSET + time->tm_year, time->tm_hour, time->tm_min, time->tm_sec);

    RETURN_ON_FALSE(AudioMoth_writeToFile(logBuffer, length));

    RETURN_ON_FALSE(AudioMoth_writeToFile(message, strlen(message)));

    RETURN_ON_FALSE(AudioMoth_writeToFile("\r\n", 1));

    RETURN_ON_FALSE(AudioMoth_closeFile());

}

/* Function prototypes */

static void flashLedToIndicateBatteryLife(void);

static bool determineSunriseAndSunsetTimes(uint32_t currentTime);

static void scheduleRecording(uint32_t currentTime, uint32_t *timeOfNextRecording, uint32_t *durationOfNextRecording, uint32_t *startOfRecordingPeriod, uint32_t *endOfRecordingPeriod);

static AM_recordingState_t makeRecording(uint32_t startTime, uint32_t duration, AM_extendedBatteryState_t extendedBatteryState, int32_t temperature, uint32_t *recordingStartTime);

/* Function to schedule recordings */

static void determineSunriseAndSunsetTimesAndScheduleRecording(uint32_t currentTime, uint32_t scheduleTime, char *text) {

    /* Use sunrise and sunset time if appropriate */

    if (configSettings->useFixedRecordingPeriods == false) {

        if (scheduleTime >= *timeOfNextSunriseAndSunsetCalculation) {

            /* Determine sunrise and sunset time */

            static char* eventTypes[4] = {"", " civil", " nautical", " astronomical"};

            char *sunriseEvent = configSettings->sunRecordingEvent == SR_SUNRISE_AND_SUNSET ? "sunrise" : "dawn";

            char *sunsetEvent = configSettings->sunRecordingEvent == SR_SUNRISE_AND_SUNSET ? "sunset" : "dusk";

            bool success = determineSunriseAndSunsetTimes(scheduleTime);

            if (success) {

                if (LOG) {

                    sprintf(fileWriteBuffer, "Calculated%s %s is %02lu:%02lu UTC and %s is %02lu:%02lu UTC.", eventTypes[configSettings->sunRecordingEvent], sunriseEvent, sunriseMinutes / 60, sunriseMinutes % 60, sunsetEvent, sunsetMinutes / 60, sunsetMinutes % 60);

                    writeLog(currentTime, fileWriteBuffer);

                }

            } else {

                if (LOG) {
                    
                    sprintf(fileWriteBuffer, "Could not determine%s %s and %s times.", eventTypes[configSettings->sunRecordingEvent], sunriseEvent, sunsetEvent);

                    writeLog(currentTime, fileWriteBuffer);

                }

            }

            /* Increment schedule time to prevent immediate restart of recording if sunrise or sunset time has shifted */

            if (*timeOfNextSunriseAndSunsetCalculation > 0) {

                scheduleTime += MAXIMUM_SUNRISE_SUNSET_SHIFT + SECONDS_IN_MINUTE * configSettings->sunRoundingMinutes;

            }

        }

    }

    /* Schedule the actual recording */

    scheduleRecording(scheduleTime, timeOfNextRecording, durationOfNextRecording, NULL, timeOfNextSunriseAndSunsetCalculation);

    if (LOG) {

        time_t rawTime = *timeOfNextRecording;

        struct tm *time = gmtime(&rawTime);

        uint32_t value = *durationOfNextRecording % SECONDS_IN_HOUR == 0 ? *durationOfNextRecording / SECONDS_IN_HOUR : *durationOfNextRecording % SECONDS_IN_MINUTE == 0 ? *durationOfNextRecording / SECONDS_IN_MINUTE : *durationOfNextRecording;

        char *units = *durationOfNextRecording % SECONDS_IN_HOUR == 0 ? "hour" : *durationOfNextRecording % SECONDS_IN_MINUTE == 0 ? "minute" : "second";

        char *plural = value > 1 ? "s" : "";

        sprintf(fileWriteBuffer, "Time of %s recording is %02d/%02d/%04d %02d:%02d:%02d UTC and duration is %lu %s%s.", text, time->tm_mday, MONTH_OFFSET + time->tm_mon, YEAR_OFFSET + time->tm_year, time->tm_hour, time->tm_min, time->tm_sec, value, units, plural);

        writeLog(currentTime, fileWriteBuffer);

    }

}

/* Functions to generate foldername and filename */

static void generateFoldername(uint32_t timestamp, char *foldername) {

    struct tm time;

    time_t rawTime = timestamp;

    gmtime_r(&rawTime, &time);

    sprintf(foldername, "%04d%02d%02d", YEAR_OFFSET + time.tm_year, MONTH_OFFSET + time.tm_mon, time.tm_mday);

}

static void generateFilename(uint32_t timestamp, char *foldername, char *extension, char *filename) {

    struct tm time;

    time_t rawTime = timestamp;

    gmtime_r(&rawTime, &time);

    uint32_t length = foldername ? sprintf(filename, "%s/", foldername) : 0;

    if (configSettings->enableFilenameWithDeviceID) {

        length += sprintf(filename + length, SERIAL_NUMBER "_", FORMAT_SERIAL_NUMBER(AM_UNIQUE_ID_START_ADDRESS));

    }

    length += sprintf(filename + length, "%04d%02d%02d_%02d%02d%02d", YEAR_OFFSET + time.tm_year, MONTH_OFFSET + time.tm_mon, time.tm_mday, time.tm_hour, time.tm_min, time.tm_sec);
 
    strcpy(filename + length, extension);

}

/* Main function */

int main(void) {

    /* Initialise device */

    AudioMoth_initialise();

    /* Configure interrupt priorities */

    NVIC_SetPriorityGrouping(5);

    /* Set highest priority for ADC and TIMER2 capture */

    NVIC_SetPriority(TIMER2_IRQn, 0);

    NVIC_SetPriority(ADC0_IRQn, 1);

    /* Set lower priority for all other used interrupts so that these are all preempted */

    NVIC_SetPriority(GPIO_EVEN_IRQn, 2);

    NVIC_SetPriority(GPIO_ODD_IRQn, 2);

    NVIC_SetPriority(TIMER0_IRQn, 2);

    NVIC_SetPriority(TIMER1_IRQn, 2);

    NVIC_SetPriority(TIMER3_IRQn, 2);

    NVIC_SetPriority(UART1_RX_IRQn, 2);

    NVIC_SetPriority(BURTC_IRQn, 2);

    NVIC_SetPriority(RTC_IRQn, 2);

    /* Initial configuration */

    if (AudioMoth_isInitialPowerUp()) {

        *latitude = 0.0f;

        *longitude = 0.0f;

        *gpsLastFixLatitude = 0;

        *gpsLastFixLongitude = 0;
    
        *timeOfNextRecording = 0;

        *lastPowerDownTimeOfGPS = 0;

        *durationOfNextRecording = 0;

        *timeOfNextInitalGPSAttempt = 0;

        *currentAcquisitionState = RESET;

        *numberOfRecordingErrors = 0;

        *previousRecordingState = RECORDING_OKAY;

        *timeOfNextSunriseAndSunsetCalculation = 0;

        *previousSwitchPosition = AM_SWITCH_NONE;

    }

    /* Read the current time */

    uint32_t currentTime;

    uint32_t currentMilliseconds;

    AudioMoth_getTime(&currentTime, &currentMilliseconds);

    /* Report on watch dog reset */

    if (AudioMoth_hasWatchdogResetOccurred()) {

        if (LOG) writeLog(currentTime, "Reset after watch dog timer overflow.");

    }

    /* Handle the case that the switch is in USB position  */

    AM_switchPosition_t switchPosition = AudioMoth_getSwitchPosition();

    if (switchPosition == AM_SWITCH_USB) {

        if (*previousSwitchPosition == AM_SWITCH_DEFAULT || *previousSwitchPosition == AM_SWITCH_CUSTOM) {

            if (LOG) writeLog(currentTime, "Switched to USB/OFF.");

            flashLedToIndicateBatteryLife();

        }

        if (fileSystemEnabled) {
            
            AudioMoth_disableFileSystem();

            fileSystemEnabled = false;

        }

        AudioMoth_handleUSB();

        SAVE_SWITCH_POSITION_AND_POWER_DOWN(DEFAULT_WAIT_INTERVAL);

    }

    /* Handle the case that the switch position has changed */

    if (switchPosition != *previousSwitchPosition) {

        if (LOG) {

            char *text = switchPosition == AM_SWITCH_DEFAULT ? "DEFAULT" : "CUSTOM";
            
            sprintf(buffer, "Switched to %s.", text);

            writeLog(currentTime, buffer);

        }

        *numberOfRecordingErrors = 0;

        *previousRecordingState = RECORDING_OKAY;

        *timeOfNextSunriseAndSunsetCalculation = 0;

        *currentAcquisitionState = WAITING_FOR_INITIAL_CONFIG;

    }

    /* Read initial configuration from SD card */

    if (*currentAcquisitionState == WAITING_FOR_INITIAL_CONFIG) {

        /* Enable file system and open settings file */

        bool fileOpened = false;

        if (fileSystemEnabled == false) fileSystemEnabled = AudioMoth_enableFileSystem(AM_SD_CARD_HIGH_SPEED);
        
        if (fileSystemEnabled) fileOpened = AudioMoth_openFileToRead("SETTINGS.TXT");

        if (fileOpened) {

            /* Parse the settings file */

            char character;

            uint32_t count = 0;

            CP_parserStatus_t status = CP_WAITING;

            while (count < MAX_FILE_READ_CHARACTERS) {

                if (count % FILE_READ_BUFFER_LENGTH == 0) {

                    AudioMoth_readFile(fileReadBuffer, FILE_READ_BUFFER_LENGTH);

                }

                character = fileReadBuffer[count % FILE_READ_BUFFER_LENGTH];

                status = ConfigParser_parse(character, &tempConfigSettings);

                if (status == CP_SUCCESS || status == CP_CHARACTER_ERROR) break;

                count += 1;

            }

            AudioMoth_closeFile();

            if (status == CP_SUCCESS) {

                /* Check the configuration */

                bool configurationOkay = ConfigChecker_checkAndFinalise(&tempConfigSettings, buffer);

                if (configurationOkay) {

                    /* Write configuration to backup domain */

                    copyToBackupDomain((uint32_t*)configSettings, (uint8_t*)&tempConfigSettings, sizeof(CP_configSettings_t));

                    /* Write log and device file */

                    if (LOG) writeLog(currentTime, "Successfully parsed SETTINGS.TXT file.");

                    writeDeviceInformation();

                    /* Determine next state */

                    if (switchPosition == AM_SWITCH_DEFAULT) {

                        *timeOfNextRecording = 0;

                        *durationOfNextRecording = UINT32_MAX;

                        *currentAcquisitionState = RECORDING;

                    } else if (configSettings->useFixedRecordingPeriods && AudioMoth_hasTimeBeenSet()) {

                        determineSunriseAndSunsetTimesAndScheduleRecording(currentTime, currentTime, "first");

                        if (configSettings->enableMagneticSwitch) {

                            if (LOG) writeLog(currentTime, "Waiting for magnetic trigger.");

                            *currentAcquisitionState = WAITING_FOR_MAGNETIC_TRIGGER_BEFORE_RECORDING;

                        } else {
                            
                            *currentAcquisitionState = RECORDING;

                        }

                    } else {

                        if (configSettings->enableMagneticSwitch) {

                            if (LOG) writeLog(currentTime, "Waiting for magnetic trigger.");

                            *currentAcquisitionState = WAITING_FOR_MAGNETIC_TRIGGER_BEFORE_INITIAL_GPS_FIX;

                        } else {

                            *currentAcquisitionState = WAITING_FOR_INITIAL_GPS_FIX;

                        }

                    }

                } else {

                    if (LOG) writeLog(currentTime, buffer);

                    FLASH_LED(Both, LONG_LED_FLASH_DURATION);

                    AudioMoth_delay(LONG_LED_FLASH_DURATION >> 1);

                    FLASH_LED(Both, LONG_LED_FLASH_DURATION);

                }

            } else {

                if (LOG) {

                    uint32_t line, position;

                    ConfigParser_getCurrentState(&line, &position, &character);

                    if (isprint(character)) {

                        sprintf(buffer, "Failed to parse SETTINGS.TXT file after reading character \'%c\' at line %lu and position %lu.", character, line, position);

                    } else {

                        sprintf(buffer, "Failed to parse SETTINGS.TXT file after reading unprintable character at line %lu and position %lu.", line, position);

                    }                 
                    
                    writeLog(currentTime, buffer);

                }

                FLASH_LED(Both, LONG_LED_FLASH_DURATION);

                AudioMoth_delay(LONG_LED_FLASH_DURATION >> 1);

                FLASH_LED(Both, LONG_LED_FLASH_DURATION);

            }

        } else {
            
            if (LOG) writeLog(currentTime, "Could not open SETTINGS.TXT file.");

            FLASH_LED(Both, LONG_LED_FLASH_DURATION);
            
        }

    }
    
    /* Enable magnetic switch */

    if (configSettings->enableMagneticSwitch) enableMagneticSwitch();

    /* Perform recording */

    int64_t timeToEarliestEvent = (int64_t)*timeOfNextRecording * MILLISECONDS_IN_SECOND - (int64_t)configSettings->recordingFixDuration * MILLISECONDS_IN_SECOND - (int64_t)currentTime * MILLISECONDS_IN_SECOND - (int64_t)currentMilliseconds;

    if (*currentAcquisitionState == RECORDING && timeToEarliestEvent < 0) {

        /* Check minimum GPS power down time */

        if (*lastPowerDownTimeOfGPS + MINIMUM_GPS_POWER_DOWN_TIME > currentTime) {

            uint32_t seconds = *lastPowerDownTimeOfGPS + MINIMUM_GPS_POWER_DOWN_TIME - currentTime;

            AudioMoth_powerDownAndWakeMilliseconds(seconds * MILLISECONDS_IN_SECOND);

        }

        /* Generate warning from previous recording */

        if (*previousRecordingState != RECORDING_OKAY) {

            if (LOG) writeLog(currentTime, "Previous recording affected by an SD card write error.");      

            *previousRecordingState = RECORDING_OKAY;

        }

        if (LOG) writeLog(currentTime, "Acquiring recording GPS fix.");

        /* Initialise GPS interface and power up GPS */

        GPSInterface_enable(GPS_TICK_EVENTS_PER_SECOND);

        AudioMoth_setGreenLED(true);

        powerUpGPS();

        /* Try to set time and update the time afterwards */

        uint32_t acquisitionTime;

        AM_fixState_t state = setTimeFromGPS(*timeOfNextRecording + *durationOfNextRecording, &acquisitionTime);

        AudioMoth_getTime(&currentTime, &currentMilliseconds);

        AudioMoth_setRedLED(false);

        /* Check state of fix */

        bool retryImmediately = false;

        if (state == CANCELLED_BY_SWITCH) {

            if (LOG) writeLog(currentTime, "Acquisition of recording GPS fix cancelled.");

            *currentAcquisitionState = RESET;

        } else if (state == CANCELLED_BY_MAGNET) {

            if (LOG) {
                
                writeLog(currentTime, "Acquisition of recording GPS fix cancelled.");

                writeLog(currentTime, "Reverting to waiting for magnetic trigger.");

            }

            AudioMoth_setGreenLED(false);

            FLASH_REPEAT_LED(Red, MAGNETIC_SWITCH_CHANGE_FLASHES, SHORT_LED_FLASH_DURATION);

            *currentAcquisitionState = POWER_DOWN_AFTER_RECORDING_CANCELLED_BY_MAGNETIC_SWITCH;

        } else if (state == TIMEOUT) {

            if (LOG) writeLog(currentTime, "Failed to acquire recording GPS fix.");

            *currentAcquisitionState = POWER_DOWN_AFTER_RECORDING;

        } else {

            /* Get longitude and latitude from NMEA results */

            GPSUtilities_getLatitude(&parserResultRMC, latitude);

            GPSUtilities_getLongitude(&parserResultRMC, longitude);

            *gpsLastFixLatitude = convertToDecimalDegrees(parserResultRMC.latitudeDegrees, parserResultRMC.latitudeMinutes, parserResultRMC.latitudeTenThousandths, parserResultRMC.latitudeDirection);

            *gpsLastFixLongitude = convertToDecimalDegrees(parserResultRMC.longitudeDegrees, parserResultRMC.longitudeMinutes, parserResultRMC.longitudeTenThousandths, parserResultRMC.longitudeDirection);        

            /* Report success */

            if (LOG) {

                sprintf(fileWriteBuffer, "Acquired recording GPS fix. Position is %02d°%02d.%04d'%c %03d°%02d.%04d'%c.", parserResultRMC.latitudeDegrees, parserResultRMC.latitudeMinutes, parserResultRMC.latitudeTenThousandths, parserResultRMC.latitudeDirection, parserResultRMC.longitudeDegrees, parserResultRMC.longitudeMinutes, parserResultRMC.longitudeTenThousandths, parserResultRMC.longitudeDirection);

                writeLog(currentTime, fileWriteBuffer);

            }

            /* Measure temperature */

            AudioMoth_enableTemperature();

            int32_t temperature = AudioMoth_getTemperature();

            AudioMoth_disableTemperature();

            /* Measure supply voltage */

            uint32_t supplyVoltage = AudioMoth_getSupplyVoltage();

            AM_extendedBatteryState_t extendedBatteryState = AudioMoth_getExtendedBatteryState(supplyVoltage);
        
            /* Enable supply voltage monitoring */

            AudioMoth_enableSupplyMonitor();

            AudioMoth_setSupplyMonitorThreshold(MINIMUM_SUPPLY_VOLTAGE);

            /* Test initial supply voltage */

            AM_recordingState_t recordingState = RECORDING_OKAY;

            bool voltageAboveThreshold = AudioMoth_isSupplyAboveThreshold();

            if (voltageAboveThreshold) {

                /* Start recording and update time afterwards */

                uint32_t recordingStartTime = 0;

                recordingState = makeRecording(*timeOfNextRecording, *durationOfNextRecording, extendedBatteryState, temperature, &recordingStartTime);

                AudioMoth_getTime(&currentTime, &currentMilliseconds);

                /* Check result of recording */

                if (recordingState == CANCELLED_BEFORE_START_BY_MICROPHONE_CHANGE) {

                    if (LOG) writeLog(currentTime, "Recording cancelled before start of recording due to microphone change.");           

                    retryImmediately = true;

                    *currentAcquisitionState = POWER_DOWN_AFTER_RECORDING;
                    
                } else if (recordingState == CANCELLED_BEFORE_START_BY_SWITCH) {

                    if (LOG) writeLog(currentTime, "Recording cancelled before start of recording due to change of switch position.");

                    *currentAcquisitionState = RESET;

                } else if (recordingState == CANCELLED_BEFORE_START_BY_MAGNET) {

                    if (LOG) {
                        
                        writeLog(currentTime, "Recording cancelled before start of recording due to magnetic switch.");
                    
                        writeLog(currentTime, "Reverting to waiting for magnetic trigger.");

                    }

                    AudioMoth_setGreenLED(false);

                    FLASH_REPEAT_LED(Red, MAGNETIC_SWITCH_CHANGE_FLASHES, SHORT_LED_FLASH_DURATION);

                    *currentAcquisitionState = POWER_DOWN_AFTER_RECORDING_CANCELLED_BY_MAGNETIC_SWITCH;

                } else if (recordingState == CANCELLED_BEFORE_START_DUE_TO_LOW_SUPPLY_VOLTAGE) {

                    if (LOG) writeLog(currentTime, "Recording cancelled before start of recording due to low supply voltage.");     

                    *currentAcquisitionState = POWER_DOWN_AFTER_RECORDING;

                } else if (recordingState == CANCELLED_DURING_RECORDING_BY_MICROPHONE_CHANGE) {

                    if (LOG) {

                        writeLog(recordingStartTime, "Recording started.");

                        writeLog(currentTime, "Recording cancelled during recording due to microphone change.");

                    }

                    retryImmediately = true;

                    *currentAcquisitionState = POWER_DOWN_AFTER_RECORDING;

                } else if (recordingState == CANCELLED_DURING_RECORDING_BY_SWITCH) {

                    if (LOG) {

                        writeLog(recordingStartTime, "Recording started.");

                        writeLog(currentTime, "Recording cancelled during recording due to change of switch position.");

                    }

                    *currentAcquisitionState = RESET;

                } else if (recordingState == CANCELLED_DURING_RECORDING_BY_MAGNET) {

                    if (LOG) {

                        writeLog(recordingStartTime, "Recording started.");

                        writeLog(currentTime, "Recording cancelled during recording due to magnetic switch.");
                        
                        writeLog(currentTime, "Reverting to waiting for magnetic trigger.");

                    }

                    AudioMoth_setGreenLED(false);

                    FLASH_REPEAT_LED(Red, MAGNETIC_SWITCH_CHANGE_FLASHES, SHORT_LED_FLASH_DURATION);

                    *currentAcquisitionState = POWER_DOWN_AFTER_RECORDING_CANCELLED_BY_MAGNETIC_SWITCH;

                } else if (recordingState == SDCARD_WRITE_ERROR) {

                    if (LOG) {

                        writeLog(recordingStartTime, "Recording started.");

                        writeLog(currentTime, "Recording stopped early due to an SD card write error.");

                    }

                    FLASH_LED(Both, LONG_LED_FLASH_DURATION);

                    *currentAcquisitionState = POWER_DOWN_AFTER_RECORDING;

                } else if (recordingState == SUPPLY_VOLTAGE_LOW) {

                    if (LOG) {

                        writeLog(recordingStartTime, "Recording started.");

                        writeLog(currentTime, "Recording stopped early due to low supply voltage.");

                    }

                    *currentAcquisitionState = POWER_DOWN_AFTER_RECORDING;

                } else if (recordingState == FILE_SIZE_LIMITED) {

                    if (LOG) {

                        writeLog(recordingStartTime, "Recording started.");

                        writeLog(currentTime, "Recording stopped early due to file size limit.");

                    }

                    retryImmediately = true;

                    *currentAcquisitionState = POWER_DOWN_AFTER_RECORDING;

                } else if (recordingState == START_TIMEOUT) {

                    if (LOG) writeLog(currentTime, "Recording timed out waiting for PPS signal.");

                    *currentAcquisitionState = POWER_DOWN_AFTER_RECORDING;

                } else if (recordingState == RECORDING_OKAY) {

                    if (LOG) {

                        writeLog(recordingStartTime, "Recording started.");

                        writeLog(currentTime, "Recording completed.");

                    }

                    *currentAcquisitionState = POWER_DOWN_AFTER_RECORDING;

                }

            } else {

                if (LOG) writeLog(currentTime, "Supply voltage too low to start recording.");

                *currentAcquisitionState = POWER_DOWN_AFTER_RECORDING;

                recordingState = SUPPLY_VOLTAGE_LOW;

            }

            AudioMoth_disableSupplyMonitor();

            /* Enable the error warning flashes */

            if (recordingState == SDCARD_WRITE_ERROR || recordingState == SUPPLY_VOLTAGE_LOW || recordingState == CANCELLED_BEFORE_START_DUE_TO_LOW_SUPPLY_VOLTAGE) {

                AudioMoth_delay(LONG_LED_FLASH_DURATION);

                FLASH_LED(Both, LONG_LED_FLASH_DURATION);

                if (recordingState == SDCARD_WRITE_ERROR) *previousRecordingState = SDCARD_WRITE_ERROR;

                *numberOfRecordingErrors += 1;

            }

        }

        /* Disable the GPS interface and power down the GPS */

        AudioMoth_getTime(lastPowerDownTimeOfGPS, NULL);

        AudioMoth_setBothLED(false);

        GPSInterface_disable();

        powerDownGPS();

        /* Schedule next recording period, ignoring the rest of the current one */

        if (*numberOfRecordingErrors >= MAXIMUM_NUMBER_OF_RECORDING_ERRORS) {

            if (LOG) writeLog(currentTime, "Recording schedule stopped due to repeated recording failures.");

            *timeOfNextRecording = UINT32_MAX;

            *durationOfNextRecording = 0;

        } else if (*currentAcquisitionState == POWER_DOWN_AFTER_RECORDING && switchPosition == AM_SWITCH_CUSTOM && retryImmediately == false) {

            uint32_t endOfCurrentRecordingPeriod = *timeOfNextRecording + *durationOfNextRecording;

            determineSunriseAndSunsetTimesAndScheduleRecording(currentTime, endOfCurrentRecordingPeriod, "next");

        } 

    }

    /* Set time and read position from GPS */
    
    if (*currentAcquisitionState == WAITING_FOR_INITIAL_GPS_FIX && currentTime >= *timeOfNextInitalGPSAttempt) {

        /* Check minimum GPS power down time */

        if (*lastPowerDownTimeOfGPS + MINIMUM_GPS_POWER_DOWN_TIME > currentTime) {

            uint32_t seconds = *lastPowerDownTimeOfGPS + MINIMUM_GPS_POWER_DOWN_TIME - currentTime;

            AudioMoth_powerDownAndWakeMilliseconds(seconds * MILLISECONDS_IN_SECOND);

        }

        /* Update log */

        if (LOG) writeLog(currentTime, "Acquiring initial GPS fix.");

        /* Initialise GPS interface and power up GPS */

        GPSInterface_enable(GPS_TICK_EVENTS_PER_SECOND);

        AudioMoth_setGreenLED(true);

        powerUpGPS();

        /* Try to set time and update the time afterwards */

        uint32_t acquisitionTime;

        AM_fixState_t state = setTimeFromGPS(currentTime + configSettings->initialFixDuration, &acquisitionTime);

        AudioMoth_getTime(&currentTime, &currentMilliseconds);

        /* Move on if successful or wait for next attempt */

        if (state == SUCCESS) {

            /* Get longitude and latitude from NMEA results */

            GPSUtilities_getLatitude(&parserResultRMC, latitude);

            GPSUtilities_getLongitude(&parserResultRMC, longitude);

            /* Report success */

            if (LOG) {

                time_t rawTime = currentTime;

                struct tm *time = gmtime(&rawTime);

                sprintf(fileWriteBuffer, "Acquired initial GPS fix. Time is %02d/%02d/%04d %02d:%02d:%02d UTC and position is %02d°%02d.%04d'%c %03d°%02d.%04d'%c.", time->tm_mday, MONTH_OFFSET + time->tm_mon, YEAR_OFFSET + time->tm_year, time->tm_hour, time->tm_min, time->tm_sec, parserResultRMC.latitudeDegrees, parserResultRMC.latitudeMinutes, parserResultRMC.latitudeTenThousandths, parserResultRMC.latitudeDirection, parserResultRMC.longitudeDegrees, parserResultRMC.longitudeMinutes, parserResultRMC.longitudeTenThousandths, parserResultRMC.longitudeDirection);

                writeLog(acquisitionTime, fileWriteBuffer);

            }

            /* Schedule recording */

            determineSunriseAndSunsetTimesAndScheduleRecording(currentTime, currentTime, "first");

            *currentAcquisitionState = RECORDING;

        } else if (state == CANCELLED_BY_SWITCH) {

            if (LOG) writeLog(currentTime, "Acquisition of initial GPS fix cancelled.");

            *currentAcquisitionState = RESET;

        } else if (state == CANCELLED_BY_MAGNET) {

            if (LOG) {

                writeLog(currentTime, "Acquisition of initial GPS fix cancelled.");

                writeLog(currentTime, "Reverting to waiting for magnetic trigger.");

            }

            AudioMoth_setGreenLED(false);

            FLASH_REPEAT_LED(Red, MAGNETIC_SWITCH_CHANGE_FLASHES, SHORT_LED_FLASH_DURATION);

            *currentAcquisitionState = WAITING_FOR_MAGNETIC_TRIGGER_BEFORE_INITIAL_GPS_FIX;

        } else if (state == TIMEOUT) {

            if (LOG) writeLog(currentTime, "Failed to acquired initial GPS fix.");

            *timeOfNextInitalGPSAttempt = currentTime + configSettings->intervalToNextFixAttempt;

        }

        /* Disable the GPS interface and power down the GPS */

        AudioMoth_getTime(lastPowerDownTimeOfGPS, NULL);

        AudioMoth_setBothLED(false);

        GPSInterface_disable();

        powerDownGPS();

    }

    /* Power down if appropriate */
    
    if (*currentAcquisitionState == RESET) {

        SAVE_SWITCH_POSITION_AND_POWER_DOWN(SHORT_WAIT_INTERVAL);

    } else if (*currentAcquisitionState == WAITING_FOR_INITIAL_CONFIG) {

        SAVE_SWITCH_POSITION_AND_POWER_DOWN(DEFAULT_WAIT_INTERVAL);        

    } else if (*currentAcquisitionState == WAITING_FOR_INITIAL_GPS_FIX) {

        if (configSettings->enableMagneticSwitch && isMagneticSwitchClosed()) {

            if (LOG) writeLog(currentTime, "Reverting to waiting for magnetic trigger.");

            FLASH_REPEAT_LED(Red, MAGNETIC_SWITCH_CHANGE_FLASHES, SHORT_LED_FLASH_DURATION);     

            *currentAcquisitionState = WAITING_FOR_MAGNETIC_TRIGGER_BEFORE_INITIAL_GPS_FIX;

        } else {

            FLASH_LED(Green, WAITING_LED_FLASH_DURATION);

            SAVE_SWITCH_POSITION_AND_POWER_DOWN(DEFAULT_WAIT_INTERVAL);

        }

    } else if (*currentAcquisitionState == POWER_DOWN_AFTER_RECORDING) {

        *currentAcquisitionState = RECORDING;

        SAVE_SWITCH_POSITION_AND_POWER_DOWN(SHORT_WAIT_INTERVAL);     

    } else if (*currentAcquisitionState == POWER_DOWN_AFTER_RECORDING_CANCELLED_BY_MAGNETIC_SWITCH) {

        *currentAcquisitionState = WAITING_FOR_MAGNETIC_TRIGGER_BEFORE_RECORDING;

        SAVE_SWITCH_POSITION_AND_POWER_DOWN(SHORT_WAIT_INTERVAL);   

    }

    /* Disable file system if still enabled */

    if (fileSystemEnabled) {
        
        AudioMoth_disableFileSystem();

        fileSystemEnabled = false;

    }

    /* Calculate the wait intervals */

    int64_t waitIntervalMilliseconds = WAITING_LED_FLASH_INTERVAL;

    uint32_t waitIntervalSeconds = WAITING_LED_FLASH_INTERVAL / MILLISECONDS_IN_SECOND;

    if (*currentAcquisitionState == WAITING_FOR_MAGNETIC_TRIGGER_BEFORE_INITIAL_GPS_FIX || *currentAcquisitionState == WAITING_FOR_MAGNETIC_TRIGGER_BEFORE_RECORDING) {
        
        waitIntervalMilliseconds *= MAGNETIC_SWITCH_WAIT_MULTIPLIER;

        waitIntervalSeconds *= MAGNETIC_SWITCH_WAIT_MULTIPLIER;

    }

    /* Enable magnetic switch interrupt */

    enableMagneticSwitchInterrupt();

    /* Wait for the next event whilst flashing the LED */

    bool firstIteration = true;

    bool startedRealTimeClock = false;

    while (true) {

        /* Update the time */

        AudioMoth_getTime(&currentTime, &currentMilliseconds);

        /* Handle switch position change */
       
        if (switchPosition != AudioMoth_getSwitchPosition()) {

            *currentAcquisitionState = RESET;
            
            SAVE_SWITCH_POSITION_AND_POWER_DOWN(SHORT_WAIT_INTERVAL);

        }

        /* Handle magnetic switch events */

        if (configSettings->enableMagneticSwitch && isMagneticSwitchClosed()) {

            if (*currentAcquisitionState == WAITING_FOR_MAGNETIC_TRIGGER_BEFORE_INITIAL_GPS_FIX || *currentAcquisitionState == WAITING_FOR_MAGNETIC_TRIGGER_BEFORE_RECORDING) {

                if (LOG) writeLog(currentTime, "Magnetic trigger detected.");

                FLASH_REPEAT_LED(Green, MAGNETIC_SWITCH_CHANGE_FLASHES, SHORT_LED_FLASH_DURATION);            

                if (*currentAcquisitionState == WAITING_FOR_MAGNETIC_TRIGGER_BEFORE_INITIAL_GPS_FIX) {

                    *currentAcquisitionState = WAITING_FOR_INITIAL_GPS_FIX;

                } else {

                    if (configSettings->useFixedRecordingPeriods && AudioMoth_hasTimeBeenSet()) {

                        determineSunriseAndSunsetTimesAndScheduleRecording(currentTime, currentTime, "next");

                        *currentAcquisitionState = RECORDING;

                    } else {

                        *currentAcquisitionState = WAITING_FOR_INITIAL_GPS_FIX;

                    }

                }

            } else {

                if (LOG) writeLog(currentTime, "Reverting to waiting for magnetic trigger.");

                FLASH_REPEAT_LED(Red, MAGNETIC_SWITCH_CHANGE_FLASHES, SHORT_LED_FLASH_DURATION);            

                *currentAcquisitionState = WAITING_FOR_MAGNETIC_TRIGGER_BEFORE_RECORDING;

            }

            SAVE_SWITCH_POSITION_AND_POWER_DOWN(SHORT_WAIT_INTERVAL);

        }

        /* Calculate the time to the next event */

        if (*currentAcquisitionState == WAITING_FOR_MAGNETIC_TRIGGER_BEFORE_INITIAL_GPS_FIX || *currentAcquisitionState == WAITING_FOR_MAGNETIC_TRIGGER_BEFORE_RECORDING) {

            timeToEarliestEvent = (int64_t)UINT32_MAX * MILLISECONDS_IN_SECOND;

        } else {

            timeToEarliestEvent = (int64_t)*timeOfNextRecording * MILLISECONDS_IN_SECOND - (int64_t)configSettings->recordingFixDuration * MILLISECONDS_IN_SECOND - (int64_t)currentTime * MILLISECONDS_IN_SECOND - (int64_t)currentMilliseconds;
           
        }

        /* Flash LED */

        if (firstIteration == false && timeToEarliestEvent > MINIMUM_LED_FLASH_INTERVAL) {

            if (*numberOfRecordingErrors > 0) {

                FLASH_LED(Both, WAITING_LED_FLASH_DURATION);

            } else {

                FLASH_LED(Green, WAITING_LED_FLASH_DURATION);

            }

        }

        /* Check there is time to sleep */

        if (timeToEarliestEvent < waitIntervalMilliseconds) {
            
            /* Calculate the remaining time to power down */

            uint32_t timeToWait = timeToEarliestEvent < 0 ? 0 : timeToEarliestEvent;

            SAVE_SWITCH_POSITION_AND_POWER_DOWN(timeToWait);

        }

        /* Start the real time clock if it isn't running */

        if (startedRealTimeClock == false) {

            AudioMoth_startRealTimeClock(waitIntervalSeconds);

            startedRealTimeClock = true;

        }

        /* Enter deep sleep */

        AudioMoth_deepSleep();

        /* Handle time overflow on awakening */

        AudioMoth_checkAndHandleTimeOverflow();

        /* Reset flag */

        firstIteration = false;

    }

}

static AM_recordingState_t makeRecording(uint32_t startTime, uint32_t duration, AM_extendedBatteryState_t extendedBatteryState, int32_t temperature, uint32_t *recordingStartTime) {
    
    /* Calculate recording settings */

    uint32_t overSampleRate = 1;

    uint32_t sampleRate = configSettings->sampleRate;

    while (MAXIMUM_REFERENCE_SAMPLE_RATE / overSampleRate / 2 >= sampleRate) overSampleRate <<= 1;

    /* Design the DC filter */

    uint32_t blockingFilterFrequency = configSettings->disable48HzDCBlockingFilter ? LOW_DC_BLOCKING_FREQ : DEFAULT_DC_BLOCKING_FREQ;

    Butterworth_designHighPassFilter(&dcFilterCoefficients, sampleRate, blockingFilterFrequency);

    if (overSampleRate < 16) dcFilterCoefficients.gain *= 16.0f / (float)overSampleRate;

    Butterworth_initialise(&dcFilter);

    /* Set up the microphone */

    microphoneChanged = false;

    AudioMoth_enableExternalSRAM();

    AM_gainRange_t gainRange = configSettings->enableLowGainRange ? AM_LOW_GAIN_RANGE : AM_NORMAL_GAIN_RANGE;

    bool externalMicrophone = AudioMoth_enableMicrophone(gainRange, configSettings->gain, CLOCK_DIVIDER, ACQUISITION_CYCLES, overSampleRate);

    AudioMoth_initialiseMicrophoneInterrupts();

    AudioMoth_startMicrophoneSamples(sampleRate);

    /* Enable file system */

    if (fileSystemEnabled == false) fileSystemEnabled = AudioMoth_enableFileSystem(AM_SD_CARD_HIGH_SPEED);

    if (fileSystemEnabled == false) return SDCARD_WRITE_ERROR;

    /* Open files */

    TURN_LED_OFF_AND_RETURN_ERROR_ON_FILE_ERROR(f_open(&filePPS, "PPS.CSV",  FA_CREATE_ALWAYS | FA_WRITE));

    TURN_LED_OFF_AND_RETURN_ERROR_ON_FILE_ERROR(f_open(&fileSAMPLES, "SAMPLES.WAV",  FA_CREATE_ALWAYS | FA_WRITE));

    /* Write headers */

    uint32_t length = sprintf(fileWriteBuffer, "PPS_NUMBER,AUDIOMOTH_TIME,SAMPLES,TOTAL_SAMPLES,TIMER_COUNT,BUFFERS_FILLED,BUFFERS_WRITTEN,LAST_RMC_AUDIOMOTH_TIME,LAST_RMC_GPS_TIME,STATUS,LAT_DEG,LAT_MIN,LAT_DIR,LONG_DEG,LONG_MIN,LONG_DIR\r\n");

    TURN_LED_OFF_AND_RETURN_ERROR_ON_FILE_ERROR(f_write(&filePPS, fileWriteBuffer, length, &bw));

    TURN_LED_OFF_AND_RETURN_ERROR_ON_FILE_ERROR(f_write(&fileSAMPLES, &wavHeader, sizeof(wavHeader), &bw));

    /* Initialise sample counters  */

    sampleCount = 0;

    lastSampleCount = 0;

    sampleCountInLastSecond = 0;

    /* Initialise buffer counts  */

    writeBuffer = 0;

    writeBufferIndex = 0;

    writeBufferCount = 0;

    /* Initialise buffers */

    buffers[0] = (int16_t*)AM_EXTERNAL_SRAM_START_ADDRESS;

    for (int i = 1; i < NUMBER_OF_BUFFERS; i += 1) {
        buffers[i] = buffers[i - 1] + NUMBER_OF_SAMPLES_IN_BUFFER;
    }

    /* Wait for appropriate PPS pulse. Timeout if PPS signal not received before the end of the recording period */

    toggleGreenLED = true;

    uint32_t currentTime = 0;

    bool supplyVoltageLow = false;

    bool cancelledBySwitch = false;

    bool cancelledByMagnet = false;

    while (!enableRecordingOnNextPPS && !microphoneChanged && !cancelledBySwitch && !cancelledByMagnet && currentTime < startTime + duration) {

        AudioMoth_getTime(&currentTime, NULL);

        if (receivedPPS) {

            /* Check time is appropriate to start recording on next PPS */

            uint32_t timeOfNextPPS = currentPPSTime + (currentPPSMilliSeconds > 500 ? 2 : 1);

            if (timeOfNextPPS >= startTime) {

                enableRecordingOnNextPPS = true;

            }

            /* Reset flag */

            receivedPPS = false;

        }

        if (receivedRMC) {

            /* Reset flag */

            receivedRMC = false;

        }

        /* Check for cancellation from magnetic, position switch, or low supply voltage */

        supplyVoltageLow = AudioMoth_isSupplyAboveThreshold() == false;

        cancelledBySwitch = AudioMoth_getSwitchPosition() == AM_SWITCH_USB;

        cancelledByMagnet = configSettings->enableMagneticSwitch && isMagneticSwitchClosed();

        /* Feed watch dog and sleep */

        AudioMoth_feedWatchdog();

        AudioMoth_sleep();

    }

    /* Wait for recording to start */

    while (!recording && !supplyVoltageLow && !microphoneChanged && !cancelledBySwitch && !cancelledByMagnet && currentTime < startTime + duration) {

        AudioMoth_getTime(&currentTime, NULL);

        /* Check for cancellation from magnetic, position switch, or low supply voltage */

        supplyVoltageLow = AudioMoth_isSupplyAboveThreshold() == false;

        cancelledBySwitch = AudioMoth_getSwitchPosition() == AM_SWITCH_USB;

        cancelledByMagnet = configSettings->enableMagneticSwitch && isMagneticSwitchClosed();

        /* Feed watch dog and sleep */

        AudioMoth_feedWatchdog();

        AudioMoth_sleep();

    }

    /* Cancel ticks */

    GPSInterface_disableTicks();

    /* Check if cancelled or timed out */

    if (!recording || supplyVoltageLow || microphoneChanged || cancelledBySwitch || cancelledByMagnet) {

        AudioMoth_setGreenLED(false);

        toggleGreenLED = false;

        f_close(&fileSAMPLES);

        f_close(&filePPS);

    }
    
    /* Return early in appropriate */

    if (supplyVoltageLow) return CANCELLED_BEFORE_START_DUE_TO_LOW_SUPPLY_VOLTAGE;

    if (microphoneChanged) return CANCELLED_BEFORE_START_BY_MICROPHONE_CHANGE;

    if (cancelledBySwitch) return CANCELLED_BEFORE_START_BY_SWITCH;

    if (cancelledByMagnet) return CANCELLED_BEFORE_START_BY_MAGNET;

    if (recording == false) return START_TIMEOUT;

    /* Calculate time that the recording actually started and update duration */

    *recordingStartTime = currentPPSTime + (currentPPSMilliSeconds > 500 ? 1 : 0);

    if (*recordingStartTime >= startTime + duration) return START_TIMEOUT;

    if (*recordingStartTime > startTime) {

        uint32_t difference = *recordingStartTime - startTime;

        duration = difference < duration ? duration - difference : 0;

    }

    /* Termination conditions */

    cancelledBySwitch = false;

    cancelledByMagnet = false;

    /* Main recording loop */

    receivedRMC = false;

    uint32_t ppsCount = 0;

    uint32_t readBuffer = 0;

    uint32_t samplesWritten = 0;

    uint32_t readBufferCount = 0;

    /* Calculate recording parameters */

    uint32_t maximumNumberOfSeconds = MAXIMUM_WAV_DATA_SIZE / NUMBER_OF_BYTES_IN_SAMPLE / sampleRate;

    bool fileSizeLimited = (duration > maximumNumberOfSeconds);

    uint32_t samplesToWrite = sampleRate * (fileSizeLimited ? maximumNumberOfSeconds : duration);

    /* Main recording loop */

    while (samplesWritten < samplesToWrite && !microphoneChanged && !cancelledBySwitch && !cancelledByMagnet && !supplyVoltageLow) {

        /* Handle interrupt events */

        if (receivedPPS) {

            /* Output to file */

            time_t rawTime = currentPPSTime;

            struct tm *time = gmtime(&rawTime);

            char *string = strlen(lastRMCBuffer) == 0 ? ",,,,,,,," : lastRMCBuffer;

            uint32_t length = sprintf(fileWriteBuffer, "%lu,%04d-%02d-%02dT%02d:%02d:%02d.%03lu,%lu,%lu,%lu,%lu,%lu,%s\r\n", ppsCount, YEAR_OFFSET + time->tm_year, MONTH_OFFSET + time->tm_mon, time->tm_mday, time->tm_hour, time->tm_min, time->tm_sec, currentPPSMilliSeconds, sampleCountInLastSecond, lastSampleCount, currentPPSMicrophoneCounter, writeBufferCount, readBufferCount, string);
            
            TURN_LED_OFF_AND_RETURN_ERROR_ON_FILE_ERROR(f_write(&filePPS, fileWriteBuffer, length, &bw));

            /* Update counter */

            ppsCount += 1;
            
            /* Reset flag */

            receivedPPS = false;

        }

        if (receivedRMC) {

            /* Prepare RMC output */

            time_t rawTime = currentRMCTime;

            struct tm *time = gmtime(&rawTime);

            uint32_t length = sprintf(lastRMCBuffer, "%04d-%02d-%02dT%02d:%02d:%02d.%03ld,", YEAR_OFFSET + time->tm_year, MONTH_OFFSET + time->tm_mon, time->tm_mday, time->tm_hour, time->tm_min, time->tm_sec, currentRMCMilliSeconds);

            length += sprintf(lastRMCBuffer + length, "%04d-%02d-%02dT%02d:%02d:%02d.%03d,", parserResultRMC.year, parserResultRMC.month, parserResultRMC.day, parserResultRMC.hours, parserResultRMC.minutes, parserResultRMC.seconds, parserResultRMC.milliseconds);

            if (parserResultRMC.status == 'A') {

                sprintf(lastRMCBuffer + length, "A,%02d,%02d.%04d,%c,%03d,%02d.%04d,%c", parserResultRMC.latitudeDegrees, parserResultRMC.latitudeMinutes, parserResultRMC.latitudeTenThousandths, parserResultRMC.latitudeDirection, parserResultRMC.longitudeDegrees, parserResultRMC.longitudeMinutes, parserResultRMC.longitudeTenThousandths, parserResultRMC.longitudeDirection);

            } else {

                sprintf(lastRMCBuffer + length, "V,00,00.0000,N,000,00.0000,E");

            }

            /* Reset flag */

            receivedRMC = false;

        }

        /* Check read and write buffers */

        if (readBuffer != writeBuffer && samplesWritten < samplesToWrite) {

            /* Write audio data */

            AudioMoth_setRedLED(true);

            uint32_t numberOfSamples = MIN(NUMBER_OF_SAMPLES_IN_BUFFER, samplesToWrite - samplesWritten);

            TURN_LED_OFF_AND_RETURN_ERROR_ON_FILE_ERROR(f_write(&fileSAMPLES, buffers[readBuffer], 2 * numberOfSamples, &bw));

            readBuffer = (readBuffer + 1) & (NUMBER_OF_BUFFERS - 1);

            readBufferCount += 1;

            samplesWritten += numberOfSamples;

            AudioMoth_setRedLED(false);

        }

        /* Check cancel conditions */

        cancelledBySwitch = AudioMoth_getSwitchPosition() == AM_SWITCH_USB;

        cancelledByMagnet = configSettings->enableMagneticSwitch && isMagneticSwitchClosed();

        supplyVoltageLow = AudioMoth_isSupplyAboveThreshold() == false;

        /* Feed watch dog and sleep */

        AudioMoth_feedWatchdog();

        AudioMoth_sleep();

    }

    /* Turn off green LED */

    toggleGreenLED = false;

    AudioMoth_setGreenLED(false);

    /* Determine recording state */

    AM_recordingState_t recordingState = microphoneChanged ? CANCELLED_DURING_RECORDING_BY_MICROPHONE_CHANGE :
                                         cancelledBySwitch ? CANCELLED_DURING_RECORDING_BY_SWITCH :
                                         cancelledByMagnet ? CANCELLED_DURING_RECORDING_BY_MAGNET :
                                         supplyVoltageLow ? SUPPLY_VOLTAGE_LOW :
                                         fileSizeLimited ? FILE_SIZE_LIMITED :
                                         RECORDING_OKAY;

    /* Generate filenames from recording start time */

    if (configSettings->enableDailyFolders) {

        generateFoldername(*recordingStartTime, foldername);

        bool directoryExists = AudioMoth_doesDirectoryExist(foldername);

        if (directoryExists == false) TURN_LED_OFF_AND_RETURN_ERROR_ON_FALSE(AudioMoth_makeDirectory(foldername));

    }

    generateFilename(*recordingStartTime, configSettings->enableDailyFolders ? foldername : NULL, ".CSV", filenamePPS);

    generateFilename(*recordingStartTime, configSettings->enableDailyFolders ? foldername : NULL, ".WAV", filenameSAMPLES);

    /* Write the GUANO data */

    uint32_t guanoDataSize = writeGuanoData(guanoBuffer, configSettings, *recordingStartTime, gpsLastFixLatitude, gpsLastFixLongitude, firmwareDescription, firmwareVersion, (uint8_t*)AM_UNIQUE_ID_START_ADDRESS, filenameSAMPLES, extendedBatteryState, temperature);

    TURN_LED_OFF_AND_RETURN_ERROR_ON_FILE_ERROR(f_write(&fileSAMPLES, guanoBuffer, guanoDataSize, &bw));

    /* Write the samples header */

    AudioMoth_setRedLED(true);

    setHeaderDetails(&wavHeader, sampleRate, samplesWritten, guanoDataSize);

    setHeaderComment(&wavHeader, configSettings, *recordingStartTime, (uint8_t*)AM_UNIQUE_ID_START_ADDRESS, extendedBatteryState, temperature, externalMicrophone, recordingState);

    TURN_LED_OFF_AND_RETURN_ERROR_ON_FILE_ERROR(f_lseek(&fileSAMPLES, 0));

    TURN_LED_OFF_AND_RETURN_ERROR_ON_FILE_ERROR(f_write(&fileSAMPLES, &wavHeader, sizeof(wavHeader), &bw));

    /* Close the files */

    TURN_LED_OFF_AND_RETURN_ERROR_ON_FILE_ERROR(f_close(&filePPS));

    TURN_LED_OFF_AND_RETURN_ERROR_ON_FILE_ERROR(f_close(&fileSAMPLES));

    /* Rename the files */

    TURN_LED_OFF_AND_RETURN_ERROR_ON_FALSE(AudioMoth_renameFile("PPS.CSV", filenamePPS));

    TURN_LED_OFF_AND_RETURN_ERROR_ON_FALSE(AudioMoth_renameFile("SAMPLES.WAV", filenameSAMPLES));

    AudioMoth_setRedLED(false);

    /* Return with state */

    return recordingState;

}

static bool determineSunriseAndSunsetTimes(uint32_t currentTime) {

    /* Calculate future sunrise and sunset time if recording is limited by earliest recording time */

    if (configSettings->earliestRecordingTime > 0) {

        currentTime = MAX(currentTime, configSettings->earliestRecordingTime);

    }

    /* Copy configuration settings from backup domain to local copy */

    copyFromBackupDomain((uint8_t*)&tempConfigSettings, (uint32_t*)configSettings, sizeof(CP_configSettings_t));

    /* Determine sunrise and sunset times */

    SR_trend_t trend;

    SR_solution_t solution;

    Sunrise_calculateFromUnix(configSettings->sunRecordingEvent, currentTime, *latitude, *longitude, &solution, &trend, &sunriseMinutes, &sunsetMinutes);

    if (solution == SR_NORMAL_SOLUTION) {

        /* Round the sunrise and sunset times */

        uint32_t roundedSunriseMinutes = configSettings->sunRoundingMinutes > 0 ? UNSIGNED_ROUND(sunriseMinutes, configSettings->sunRoundingMinutes) : sunriseMinutes;

        uint32_t roundedSunsetMinutes = configSettings->sunRoundingMinutes > 0 ? UNSIGNED_ROUND(sunsetMinutes, configSettings->sunRoundingMinutes) : sunsetMinutes;

        /* Calculate start and end of potential recording periods */

        uint32_t beforeSunrise = (MINUTES_IN_DAY + roundedSunriseMinutes - configSettings->sunriseInterval.beforeMinutes) % MINUTES_IN_DAY;

        uint32_t afterSunrise = (MINUTES_IN_DAY + roundedSunriseMinutes + configSettings->sunriseInterval.afterMinutes) % MINUTES_IN_DAY;

        uint32_t beforeSunset = (MINUTES_IN_DAY + roundedSunsetMinutes - configSettings->sunsetInterval.beforeMinutes) % MINUTES_IN_DAY;

        uint32_t afterSunset = (MINUTES_IN_DAY + roundedSunsetMinutes + configSettings->sunsetInterval.afterMinutes) % MINUTES_IN_DAY;
        
        /* Set up recording periods */

        if (configSettings->sunRecordingMode == SUNRISE_RECORDING) {

            /* Recording from before sunrise to after sunrise */

            tempConfigSettings.activeRecordingPeriods = 1;

            tempConfigSettings.recordingPeriods[0].startMinutes = beforeSunrise;
            
            tempConfigSettings.recordingPeriods[0].endMinutes = afterSunrise;
            
        } else if (configSettings->sunRecordingMode == SUNSET_RECORDING) {

            /* Recording from before sunset to after sunset */

            tempConfigSettings.activeRecordingPeriods = 1;

            tempConfigSettings.recordingPeriods[0].startMinutes = beforeSunset;
            
            tempConfigSettings.recordingPeriods[0].endMinutes = afterSunset;

        } else if (configSettings->sunRecordingMode == SUNRISE_AND_SUNSET_RECORDING) {

            /* Order the recording periods */

            uint32_t firstPeriodStartMinutes = beforeSunrise < beforeSunset ? beforeSunrise : beforeSunset;
            uint32_t firstPeriodEndMinutes = beforeSunrise < beforeSunset ? afterSunrise : afterSunset;
           
            uint32_t secondPeriodStartMinutes = beforeSunrise < beforeSunset ? beforeSunset : beforeSunrise;
            uint32_t secondPeriodEndMinutes  = beforeSunrise < beforeSunset ? afterSunset : afterSunrise;

            /* Determine whether the recording periods wrap */

            bool firstPeriodWraps = firstPeriodEndMinutes <= firstPeriodStartMinutes;

            bool secondPeriodWraps = secondPeriodEndMinutes <= secondPeriodStartMinutes;

            /* Combine recording periods together if they overlap */

            if (firstPeriodWraps) {

                tempConfigSettings.activeRecordingPeriods = 1;

                tempConfigSettings.recordingPeriods[0].startMinutes = firstPeriodStartMinutes;

                tempConfigSettings.recordingPeriods[0].endMinutes = secondPeriodWraps ? MIN(firstPeriodStartMinutes, MAX(firstPeriodEndMinutes, secondPeriodEndMinutes)) : firstPeriodEndMinutes;

            } else if (secondPeriodStartMinutes <= firstPeriodEndMinutes) {

                tempConfigSettings.activeRecordingPeriods = 1;

                tempConfigSettings.recordingPeriods[0].startMinutes = firstPeriodStartMinutes;

                tempConfigSettings.recordingPeriods[0].endMinutes = secondPeriodWraps ? MIN(firstPeriodStartMinutes, secondPeriodEndMinutes) : MAX(firstPeriodEndMinutes, secondPeriodEndMinutes);

            } else if (secondPeriodWraps && secondPeriodEndMinutes >= firstPeriodStartMinutes) {

                tempConfigSettings.activeRecordingPeriods = 1;

                tempConfigSettings.recordingPeriods[0].startMinutes = secondPeriodStartMinutes;

                tempConfigSettings.recordingPeriods[0].endMinutes = MAX(firstPeriodEndMinutes, secondPeriodEndMinutes);

            } else {

                tempConfigSettings.activeRecordingPeriods = 2;

                tempConfigSettings.recordingPeriods[0].startMinutes = firstPeriodStartMinutes;

                tempConfigSettings.recordingPeriods[0].endMinutes = firstPeriodEndMinutes;

                tempConfigSettings.recordingPeriods[1].startMinutes = secondPeriodStartMinutes;

                tempConfigSettings.recordingPeriods[1].endMinutes = secondPeriodEndMinutes;

            }

        } else if (configSettings->sunRecordingMode == SUNSET_TO_SUNRISE_RECORDING) {

            /* Recording from before sunset to after sunrise */

            tempConfigSettings.activeRecordingPeriods = 1;

            tempConfigSettings.recordingPeriods[0].startMinutes = beforeSunset;

            uint32_t timeFromSunsetToSunrise = roundedSunriseMinutes < roundedSunsetMinutes ? MINUTES_IN_DAY + roundedSunriseMinutes - roundedSunsetMinutes : roundedSunriseMinutes - roundedSunsetMinutes;

            uint32_t duration = timeFromSunsetToSunrise + configSettings->sunsetInterval.beforeMinutes + configSettings->sunriseInterval.afterMinutes;

            tempConfigSettings.recordingPeriods[0].endMinutes = duration < MINUTES_IN_DAY ? afterSunrise : beforeSunset;

        } else if (configSettings->sunRecordingMode == SUNRISE_TO_SUNSET_RECORDING) {

            /* Recording from before sunrise to after sunset */

            tempConfigSettings.activeRecordingPeriods = 1;

            tempConfigSettings.recordingPeriods[0].startMinutes = beforeSunrise;

            uint32_t timeFromSunriseToSunset = roundedSunsetMinutes < roundedSunriseMinutes ? MINUTES_IN_DAY + roundedSunsetMinutes - roundedSunriseMinutes : roundedSunsetMinutes - roundedSunriseMinutes;

            uint32_t duration = timeFromSunriseToSunset + configSettings->sunriseInterval.beforeMinutes + configSettings->sunsetInterval.afterMinutes;

            tempConfigSettings.recordingPeriods[0].endMinutes = duration < MINUTES_IN_DAY ? afterSunset : beforeSunrise;

        }

    } else {

        tempConfigSettings.activeRecordingPeriods = 0;

    }

    /* Copy local configuration settings to backup domain */

    copyToBackupDomain((uint32_t*)configSettings, (uint8_t*)&tempConfigSettings, sizeof(CP_configSettings_t));

    return solution == SR_NORMAL_SOLUTION;

}

static void adjustRecordingDuration(uint32_t *duration, uint32_t recordDuration, uint32_t sleepDuration) {

    uint32_t durationOfCycle = recordDuration + sleepDuration;

    uint32_t numberOfCycles = *duration / durationOfCycle;

    uint32_t partialCycle = *duration % durationOfCycle;

    if (partialCycle == 0) {

        *duration = *duration > sleepDuration ? *duration - sleepDuration : 0;

    } else {

        *duration = MIN(*duration, numberOfCycles * durationOfCycle + recordDuration);

    }

}

static void calculateStartAndDuration(uint32_t currentTime, uint32_t currentSeconds, recordingPeriod_t *period, uint32_t *startTime, uint32_t *duration) {

    *startTime = currentTime - currentSeconds + SECONDS_IN_MINUTE * period->startMinutes;

    *duration = period->endMinutes <= period->startMinutes ? MINUTES_IN_DAY + period->endMinutes - period->startMinutes : period->endMinutes - period->startMinutes;
    
    *duration *= SECONDS_IN_MINUTE;

}

static void scheduleRecording(uint32_t currentTime, uint32_t *timeOfNextRecording, uint32_t *durationOfNextRecording, uint32_t *startOfRecordingPeriod, uint32_t *endOfRecordingPeriod) {

    /* Enforce minumum schedule date */

    currentTime = MAX(currentTime, START_OF_CENTURY);

    /* Check if recording should be limited by earliest recording time */

    if (configSettings->earliestRecordingTime > 0) {

        currentTime = MAX(currentTime, configSettings->earliestRecordingTime);

    }

    /* No suitable recording periods */

    uint32_t activeRecordingPeriods = MIN(configSettings->activeRecordingPeriods, MAX_RECORDING_PERIODS);

    if (activeRecordingPeriods == 0) {

        *timeOfNextRecording = UINT32_MAX;

        if (startOfRecordingPeriod) *startOfRecordingPeriod = UINT32_MAX;

        if (endOfRecordingPeriod) *endOfRecordingPeriod = UINT32_MAX;

        *durationOfNextRecording = 0;

        return;

    }

    /* Calculate the number of seconds of this day */

    time_t rawTime = currentTime;

    struct tm *time = gmtime(&rawTime);

    uint32_t currentSeconds = SECONDS_IN_HOUR * time->tm_hour + SECONDS_IN_MINUTE * time->tm_min + time->tm_sec;

    /* Check the last active period on the previous day */

    recordingPeriod_t *lastPeriod = configSettings->recordingPeriods + configSettings->activeRecordingPeriods - 1;

    uint32_t startTime, duration;
    
    calculateStartAndDuration(currentTime - SECONDS_IN_DAY, currentSeconds, lastPeriod, &startTime, &duration);

    if (configSettings->disableSleepRecordCycle == false) {
    
        adjustRecordingDuration(&duration, configSettings->recordDuration, configSettings->sleepDuration);

    }

    if (currentTime < startTime + duration && duration > 0) goto done;

    /* Check each active recording period on the same day*/

    for (uint32_t i = 0; i < activeRecordingPeriods; i += 1) {

        recordingPeriod_t *currentPeriod = configSettings->recordingPeriods + i;

        calculateStartAndDuration(currentTime, currentSeconds, currentPeriod, &startTime, &duration);

        if (configSettings->disableSleepRecordCycle == false) {
        
            adjustRecordingDuration(&duration, configSettings->recordDuration, configSettings->sleepDuration);

        }
        
        if (currentTime < startTime + duration && duration > 0) goto done;

    }

    /* Calculate time until first period tomorrow */

    recordingPeriod_t *firstPeriod = configSettings->recordingPeriods;

    calculateStartAndDuration(currentTime + SECONDS_IN_DAY, currentSeconds, firstPeriod, &startTime, &duration);

    if (configSettings->disableSleepRecordCycle == false) {
    
        adjustRecordingDuration(&duration, configSettings->recordDuration, configSettings->sleepDuration);

    }

done:

    /* Set the time for start and end of the recording period */

    if (startOfRecordingPeriod) *startOfRecordingPeriod = startTime;

    if (endOfRecordingPeriod) *endOfRecordingPeriod = startTime + duration;

    /* Resolve sleep and record cycle */

    if (configSettings->disableSleepRecordCycle) {

        *timeOfNextRecording = startTime;

        *durationOfNextRecording = duration;

    } else {
        
        if (currentTime <= startTime) {

            /* Recording should start at the start of the recording period */

            *timeOfNextRecording = startTime;

            *durationOfNextRecording = MIN(duration, configSettings->recordDuration);

        } else {

            /* Recording should start immediately or at the start of the next recording cycle */

            uint32_t secondsFromStartOfPeriod = currentTime - startTime;

            uint32_t durationOfCycle = configSettings->recordDuration + configSettings->sleepDuration;

            uint32_t partialCycle = secondsFromStartOfPeriod % durationOfCycle;

            *timeOfNextRecording = currentTime - partialCycle;

            if (partialCycle >= configSettings->recordDuration) {

                /* Wait for next cycle to begin */

                *timeOfNextRecording += durationOfCycle;

            }

            uint32_t remainingDuration = startTime + duration - *timeOfNextRecording;

            *durationOfNextRecording = MIN(configSettings->recordDuration, remainingDuration);

        }

    }

    /* Check if recording should be limited by last recording time */

    uint32_t latestRecordingTime = configSettings->latestRecordingTime > 0 ? configSettings->latestRecordingTime : MIDPOINT_OF_CENTURY;

    if (*timeOfNextRecording >= latestRecordingTime) {

        *timeOfNextRecording = UINT32_MAX;

        if (startOfRecordingPeriod) *startOfRecordingPeriod = UINT32_MAX;

        if (endOfRecordingPeriod) *endOfRecordingPeriod = UINT32_MAX;

        *durationOfNextRecording = 0;

    } else {

        int64_t excessTime = (int64_t)*timeOfNextRecording + (int64_t)*durationOfNextRecording - (int64_t)latestRecordingTime;

        if (excessTime > 0) {
            
            *durationOfNextRecording -= excessTime;

            if (endOfRecordingPeriod) *endOfRecordingPeriod = *timeOfNextRecording + *durationOfNextRecording;

        }

    }

}

/* Flash LED according to battery life */

static void flashLedToIndicateBatteryLife() {

    uint32_t numberOfFlashes = LOW_BATTERY_LED_FLASHES;

    uint32_t supplyVoltage = AudioMoth_getSupplyVoltage();

    if (configSettings->batteryLevelDisplayType == NIMH_LIPO_BATTERY_VOLTAGE) {

        /* Set number of flashes according to battery voltage */

        AM_extendedBatteryState_t batteryState = AudioMoth_getExtendedBatteryState(supplyVoltage);

        if (batteryState > AM_EXT_BAT_4V3) {

            numberOfFlashes = 1;

        } else if (batteryState > AM_EXT_BAT_3V5) {

            numberOfFlashes = AM_EXT_BAT_4V4 - batteryState;

        }

    } else {

        /* Set number of flashes according to battery state */

        AM_batteryState_t batteryState = AudioMoth_getBatteryState(supplyVoltage);

        if (batteryState > AM_BATTERY_LOW) {

            numberOfFlashes = (batteryState >= AM_BATTERY_4V6) ? 4 : (batteryState >= AM_BATTERY_4V4) ? 3 : (batteryState >= AM_BATTERY_4V0) ? 2 : 1;

        }

    }

    /* Flash LED */

    for (uint32_t i = 0; i < numberOfFlashes; i += 1) {

        FLASH_LED(Red, SHORT_LED_FLASH_DURATION)

        if (numberOfFlashes == LOW_BATTERY_LED_FLASHES) {

            AudioMoth_delay(SHORT_LED_FLASH_DURATION);

        } else {

            AudioMoth_delay(LONG_LED_FLASH_DURATION);

        }

    }

}