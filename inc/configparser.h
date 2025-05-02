/****************************************************************************
 * configparser.h
 * openacousticdevices.info
 * October 2022
 *****************************************************************************/

#ifndef __CONFIGPARSER_H
#define __CONFIGPARSER_H

#include <stdint.h>
#include <stdbool.h>

#include "sunrise.h"

#define MAX_RECORDING_PERIODS 10

typedef enum {CP_WAITING, CP_PARSING, CP_CHARACTER_ERROR, CP_SUCCESS} CP_parserStatus_t;

typedef enum {NONE, BATTERY_LEVEL, NIMH_LIPO_BATTERY_VOLTAGE} CP_batteryLevelDisplayType_t;

typedef enum {SUNRISE_RECORDING, SUNSET_RECORDING, SUNRISE_AND_SUNSET_RECORDING, SUNSET_TO_SUNRISE_RECORDING, SUNRISE_TO_SUNSET_RECORDING} CP_sunRecordingMode_t;

typedef struct {
    uint16_t startMinutes;
    uint16_t endMinutes;
} CP_recordingPeriod_t;

typedef struct {
    uint16_t beforeMinutes;
    uint16_t afterMinutes;
} CP_interval_t;

typedef struct {
    uint8_t gain;
    uint32_t sampleRate;
    bool enableDailyFolders;
    bool enableFilenameWithDeviceID;
    bool enableLowGainRange;
    bool enableMagneticSwitch;
    bool disable48HzDCBlockingFilter;
    CP_batteryLevelDisplayType_t batteryLevelDisplayType;
    uint32_t initialFixDuration;
    uint32_t intervalToNextFixAttempt;
    uint32_t recordingFixDuration;
    bool disableSleepRecordCycle;
    uint32_t sleepDuration;
    uint32_t recordDuration;
    uint32_t activeRecordingPeriods;
    bool useFixedRecordingPeriods;
    CP_recordingPeriod_t recordingPeriods[MAX_RECORDING_PERIODS];
    CP_sunRecordingMode_t sunRecordingMode;
    SR_event_t sunRecordingEvent;
    uint32_t sunRoundingMinutes;
    CP_interval_t sunriseInterval;
    CP_interval_t sunsetInterval;
    bool firstRecordingDate;
    uint32_t firstRecordingDay;
    uint32_t firstRecordingMonth;
    uint32_t firstRecordingYear;
    uint32_t earliestRecordingTime;
    bool lastRecordingDate;
    uint32_t lastRecordingDay;
    uint32_t lastRecordingMonth;
    uint32_t lastRecordingYear;
    uint32_t latestRecordingTime;
} CP_configSettings_t;

void ConfigParser_reset();

void ConfigParser_getCurrentState(uint32_t *line, uint32_t *position, char *character);

CP_parserStatus_t ConfigParser_parse(char c, CP_configSettings_t *result);

#endif /* __CONFIGPARSER_H */
