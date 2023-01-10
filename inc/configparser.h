/****************************************************************************
 * configParser.h
 * openacousticdevices.info
 * October 2022
 *****************************************************************************/

#ifndef __CONFIGPARSER_H
#define __CONFIGPARSER_H

#include <stdint.h>
#include <stdbool.h>

#define MAX_RECORDING_PERIODS 10

typedef enum {CP_WAITING, CP_PARSING, CP_CHARACTER_ERROR, CP_VALUE_ERROR, CP_SUCCESS} CP_parserStatus_t;

typedef enum {NONE, BATTERY_LEVEL, NIMH_LIPO_BATTERY_VOLTAGE} CP_batteryLevelDisplayType_t;

typedef struct {
    uint32_t startMinutes;
    uint32_t durationMinutes;
} CP_recordingPeriod_t;

typedef struct {
    uint32_t beforeMinutes;
    uint32_t afterMinutes;
} CP_interval_t;

typedef struct {
    uint8_t gain;
    uint32_t sampleRate;
    bool enableDailyFolders;
    bool enableLowGainRange;
    bool enableMagneticSwitch;
    bool disable48HzDCBlockingFilter;
    CP_batteryLevelDisplayType_t batteryLevelDisplayType;
    uint32_t initialFixDuration;
    uint32_t intervalToNextFixAttempt;
    uint32_t recordingFixDuration;
    uint32_t activeRecordingPeriods;
    bool useFixedRecordingPeriods;
    CP_recordingPeriod_t recordingPeriods[MAX_RECORDING_PERIODS];
    uint32_t sunRecordingMode;
    uint32_t sunRoundingMinutes;
    CP_interval_t sunriseInterval;
    CP_interval_t sunsetInterval;
    uint32_t earliestRecordingTime;
    uint32_t latestRecordingTime;
} CP_configSettings_t;

void ConfigParser_reset();

void ConfigParser_getCurrentState(uint32_t *line, uint32_t *position, char *character);

CP_parserStatus_t ConfigParser_parse(char c, CP_configSettings_t *result);

#endif /* __CONFIGPARSER_H */
