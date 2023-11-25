/****************************************************************************
 * configchecker.c
 * openacousticdevices.info
 * October 2022
 *****************************************************************************/

#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "configparser.h"

/* Sample rate constants */

#define NUMBER_OF_SUPPORTED_SAMPLE_RATES        7

/* Custom time constants */

#define MINUTES_IN_HOUR                         60
#define SECONDS_IN_MINUTE                       60
#define SECONDS_IN_HOUR                         3600
#define MINUTES_IN_DAY                          1440
#define MINUTES_IN_SIX_HOURS                    (60 * 60)
#define SECONDS_IN_DAY                          (1440 * 60)

#define MAXIMUM_DURATION                        MINUTES_IN_DAY
#define MAXIMUM_START_MINUTES                   (MINUTES_IN_DAY - 1)

#define MONTH_JAN                               1
#define MONTH_FEB                               2
#define MONTH_DEC                               12

#define DAYS_IN_YEAR                            365
#define UNIX_EPOCH_START                        1970

/* Custom variables */

static const uint32_t daysInMonth[12] = {31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};

/* Custom functions to check dates */

static bool isLeapYear(uint32_t year) {

    return (year & 3) == 0 && ((year % 25) != 0 || (year & 15) == 0);

}

static bool checkDate(uint32_t day, uint32_t month, uint32_t year, uint32_t *timestamp) {

    *timestamp = 0;

    if (month < MONTH_JAN || month > MONTH_DEC) return false;

    uint32_t dayInCurrentMonth = daysInMonth[month - 1] + (month == MONTH_FEB && isLeapYear(year) ? 1 : 0);

    if (day == 0 || day > dayInCurrentMonth) return false;

    for (uint32_t y = UNIX_EPOCH_START; y < year; y += 1) *timestamp += SECONDS_IN_DAY * (isLeapYear(y) ? DAYS_IN_YEAR + 1 : DAYS_IN_YEAR);

    for (uint32_t m = 0; m < month - 1; m += 1) *timestamp += SECONDS_IN_DAY * daysInMonth[m];

    if (isLeapYear(year) && month > MONTH_FEB) *timestamp += SECONDS_IN_DAY;

    *timestamp += SECONDS_IN_DAY * (day - 1);

    return true;

}

/* Public function to check configuration */

bool ConfigChecker_checkAndFinalise(CP_configSettings_t *configSettings, char *buffer) {

    char *errorText = "Error in SETTINGS.TXT file."; 

    /* Check sample rate */

    static uint32_t supportedSampleRates[NUMBER_OF_SUPPORTED_SAMPLE_RATES] = {8000, 16000, 32000, 48000, 96000, 125000, 192000};

    bool sampleRateOkay = false;

    for (uint32_t i = 0; i < NUMBER_OF_SUPPORTED_SAMPLE_RATES; i += 1) {

        sampleRateOkay = sampleRateOkay || supportedSampleRates[i] == configSettings->sampleRate;
        
    }

    if (sampleRateOkay == false) {

        sprintf(buffer, "%s Sample rate is not supported.", errorText);

        return false;

    }

    /* Check initial fix duration */

    bool configurationOkay = configSettings->initialFixDuration > 0 && configSettings->initialFixDuration <= SECONDS_IN_HOUR;

    if (configurationOkay == false) {

        char* text = configSettings->initialFixDuration == 0 ? "be equal to 0 seconds." : "exceed 3600 seconds.";
        
        sprintf(buffer, "%s Initial fix duration cannot %s", errorText, text);
            
        return false;

    }

    /* Check interval to next fix attempt */

    configurationOkay = configSettings->intervalToNextFixAttempt > 0 && configSettings->intervalToNextFixAttempt <= SECONDS_IN_HOUR;

    if (configurationOkay == false) {
        
        char* text = configSettings->intervalToNextFixAttempt == 0 ? "be equal to 0 seconds." : "exceed 3600 seconds.";
        
        sprintf(buffer, "%s Interval to next fix attempt cannot %s", errorText, text);
            
        return false;

    }

    /* Check recording fix duration */

    configurationOkay = configSettings->recordingFixDuration > 0 && configSettings->recordingFixDuration <= SECONDS_IN_HOUR;

    if (configurationOkay == false) {
        
        char* text = configSettings->recordingFixDuration == 0 ? "be equal to 0 seconds." : "exceed 3600 seconds.";
        
        sprintf(buffer, "%s Recording fix duration cannot %s", errorText, text);
             
        return false;

    }

    if (configSettings->disableSleepRecordCycle == false) {

        /* Check sleep duration */

        configurationOkay = configSettings->sleepDuration <= SECONDS_IN_HOUR;

        if (configurationOkay == false) {
            
            sprintf(buffer, "%s Sleep duration cannot exceed 3600 seconds.", errorText);
                
            return false;

        }

        /* Check sleep duration greater than recording fix duration */

        configurationOkay = configSettings->sleepDuration >= configSettings->recordingFixDuration;

        if (configurationOkay == false) {
            
            sprintf(buffer, "%s Sleep duration cannot be less than the recording fix duration.", errorText);
                
            return false;

        }

        /* Check recording duration */

        configurationOkay = configSettings->recordDuration > 0 && configSettings->recordDuration <= SECONDS_IN_HOUR;

        if (configurationOkay == false) {
            
            char* text = configSettings->recordDuration == 0 ? "be equal to 0 seconds." : "exceed 3600 seconds.";
        
            sprintf(buffer, "%s Record duration cannot %s", errorText, text);
                
            return false;

        }

    }

    if (configSettings->useFixedRecordingPeriods) {

        for (uint32_t i = 0; i < configSettings->activeRecordingPeriods; i += 1) {

            /* Check recording period start time */

            configurationOkay = configSettings->recordingPeriods[i].startMinutes < MINUTES_IN_DAY;

            if (configurationOkay == false) {
                
                sprintf(buffer, "%s Recording period start time cannot exceed 23:59.", errorText);
                    
                return false;

            }

            /* Check recording period end time */
            
            configurationOkay = configSettings->recordingPeriods[i].endMinutes <= MINUTES_IN_DAY;

            if (configurationOkay == false) {
                
                sprintf(buffer, "%s Recording period stop time cannot exceed 24:00.", errorText);
                    
                return false;

            }

        }

        /* Check recording periods are in order */

        for (uint32_t i = 0; i < configSettings->activeRecordingPeriods - 1; i += 1) {

            configurationOkay = configSettings->recordingPeriods[i+1].startMinutes > configSettings->recordingPeriods[i].startMinutes;

            if (configurationOkay == false) {

                sprintf(buffer, "%s Recording periods must be ordered by their start time.", errorText);
                    
                return false;

            }

        }

        /* Check recording periods do not overlap */

        for (uint32_t i = 0; i < configSettings->activeRecordingPeriods - 1; i += 1) {

            configurationOkay = configSettings->recordingPeriods[i].endMinutes > configSettings->recordingPeriods[i].startMinutes && configSettings->recordingPeriods[i+1].startMinutes > configSettings->recordingPeriods[i].endMinutes % MINUTES_IN_DAY;

            if (configurationOkay == false) {
                
                sprintf(buffer, "%s Recording periods cannot overlap.", errorText);

                return false;

            }

        }

        if (configSettings->activeRecordingPeriods > 1) {

            configurationOkay = configSettings->recordingPeriods[configSettings->activeRecordingPeriods-1].startMinutes < configSettings->recordingPeriods[configSettings->activeRecordingPeriods-1].endMinutes % MINUTES_IN_DAY || configSettings->recordingPeriods[0].startMinutes > configSettings->recordingPeriods[configSettings->activeRecordingPeriods-1].endMinutes % MINUTES_IN_DAY;

            if (configurationOkay == false) {
                
                sprintf(buffer, "%s Last recording period cannot overlap with first recording period.", errorText);

                return false;

            }

        }

    } else {

        /* Check sun rounding minutes */

        configurationOkay = configSettings->sunRoundingMinutes <= MINUTES_IN_HOUR;

        if (configurationOkay == false) {
            
            sprintf(buffer, "%s Error in SETTINGS.TXT file. Sun rounding minutes cannot be greater than 60 minutes.", errorText);
                
            return false;

        }

        /* Check sunrise interval before minutes */

        configurationOkay = configSettings->sunriseInterval.beforeMinutes <= MINUTES_IN_SIX_HOURS;

        if (configurationOkay == false) {
            
            sprintf(buffer, "%s Sunrise interval before minutes cannot be greater than 360 minutes.", errorText);
                
            return false;

        }

        /* Check sunrise interval after minutes */

        configurationOkay = configSettings->sunriseInterval.afterMinutes <= MINUTES_IN_SIX_HOURS;

        if (configurationOkay == false) {
            
            sprintf(buffer, "%s Sunrise interval after minutes cannot be greater than 360 minutes.", errorText);
                
            return false;

        }

        /* Check sunset interval before minutes */

        configurationOkay = configSettings->sunsetInterval.beforeMinutes <= MINUTES_IN_SIX_HOURS;

        if (configurationOkay == false) {
            
            sprintf(buffer, "%s Sunset interval before minutes cannot be greater than 360 minutes.", errorText);
                
            return false;

        }

        /* Check sunset interval after minutes */

        configurationOkay = configSettings->sunsetInterval.afterMinutes <= MINUTES_IN_SIX_HOURS;

        if (configurationOkay == false) {
            
            sprintf(buffer, "%s Sunset interval after minutes cannot be greater than 360 minutes.", errorText);
                
            return false;

        }

        /* Check sunrise recording period */

        configurationOkay = (configSettings->sunriseInterval.beforeMinutes == 0 && configSettings->sunriseInterval.afterMinutes == 0 && (configSettings->sunRecordingMode == SUNRISE_RECORDING || configSettings->sunRecordingMode == SUNRISE_AND_SUNSET_RECORDING)) == false;

        if (configurationOkay == false) {
            
            sprintf(buffer, "%s Sunrise recording duration cannot be equal to 0 minutes.", errorText);

            return false;

        }

        /* Check sunset recording period */

        configurationOkay = (configSettings->sunsetInterval.beforeMinutes == 0 && configSettings->sunsetInterval.afterMinutes == 0 && (configSettings->sunRecordingMode == SUNSET_RECORDING || configSettings->sunRecordingMode == SUNRISE_AND_SUNSET_RECORDING)) == false;

        if (configurationOkay == false) {
            
            sprintf(buffer, "%s Sunset recording duration cannot be equal to 0 minutes.", errorText);
                
            return false;

        }

    }

    /* Check first recording date */

    if (configSettings->firstRecordingDate) {

        bool dateOkay = checkDate(configSettings->firstRecordingDay, configSettings->firstRecordingMonth, configSettings->firstRecordingYear, &configSettings->earliestRecordingTime);

        if (dateOkay == false) {

            sprintf(buffer, "%s First recording date is not valid.", errorText);

            return false;

        }

    }

    /* Check last recording date */

    if (configSettings->lastRecordingDate) {

        bool dateOkay = checkDate(configSettings->lastRecordingDay, configSettings->lastRecordingMonth, configSettings->lastRecordingYear, &configSettings->latestRecordingTime);

        configSettings->latestRecordingTime += SECONDS_IN_DAY;

        if (dateOkay == false) {

            sprintf(buffer, "%s Last recording date is not valid.", errorText);

            return false;

        }

    }

    /* Return */

    return true;

}