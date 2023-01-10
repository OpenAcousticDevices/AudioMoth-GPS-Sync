/****************************************************************************
 * configParser.c
 * openacousticdevices.info
 * October 2022
 *****************************************************************************/

#include <string.h>
#include <stdlib.h>

#include "configparser.h"

#define MAX_BUFFER_LENGTH       32

/* Useful macros */

#define MAX(a,b)                (((a) > (b)) ? (a) : (b))

#define MIN(a,b)                (((a) < (b)) ? (a) : (b))

/* Structure to maintain state */

typedef struct {
    uint32_t state;
    uint32_t index;
    uint32_t count;
    uint32_t returnState;
    char buffer[MAX_BUFFER_LENGTH];
    CP_parserStatus_t status;
    char character;
    uint32_t line;
    uint32_t position;
} CP_parserState_t;

/* Macro definitions for updating state */

#define INC_STATE state->state += 1
#define ZERO_STATE state->state = 0
#define SET_STATE(X) state->state = X
#define ADD_STATE(X) state->state += X
#define SUB_STATE(X) state->state -= X

#define INDEX state->index
#define INC_INDEX state->index += 1
#define ZERO_INDEX state->index = 0
#define SET_INDEX(X) state->index = X

#define COUNT state->count
#define INC_COUNT state->count += 1
#define ZERO_COUNT state->count = 0

#define RETURN state->returnState
#define SET_RETURN(X) state->returnState = X

#define BUFFER state->buffer
#define ADD_TO_BUFFER if (state->count < MAX_BUFFER_LENGTH - 1) {state->buffer[state->count] = c; state->count += 1;}
#define CLEAR_BUFFER memset(state->buffer, 0, MAX_BUFFER_LENGTH); state->count = 0

#define CLEAR_STATE ZERO_STATE; ZERO_INDEX; SET_RETURN(0); CLEAR_BUFFER; state->status = CP_WAITING

#define SET_STATUS_SUCCESS state->status = CP_SUCCESS

/* Private macro definition for defining jump table functions */

#define _FUNCTION_START(NAME, NUMBER) \
void NAME ## NUMBER (char c, CP_parserState_t *state, CP_configSettings_t *configSettings) {state->character = c; state->position += 1; if (c == '\n') {state->line += 1; state->position = 0;}; if (c == ' ' || c == '\t' || c == '\n' || c == '\r') {} else

#define _FUNCTION_END(STATE, STATUS) \
{state->state = STATE; state->status = STATUS;} }

/* Macro definitions for defining jump table functions */

#define DEFINE_FUNCTION_INIT(NAME, NUMBER, CONDITION) \
_FUNCTION_START(NAME, NUMBER) if (CONDITION) {CLEAR_STATE; state->state = 1; state->status = CP_PARSING;} else _FUNCTION_END(0, CP_WAITING)

#define DEFINE_FUNCTION_STEP(NAME, NUMBER, CONDITION, ACTION) \
_FUNCTION_START(NAME, NUMBER) if (CONDITION) {ACTION;} else _FUNCTION_END(0, CP_CHARACTER_ERROR)

#define DEFINE_FUNCTION_ELSE(NAME, NUMBER, CONDITION1, ACTION1, CONDITION2, ACTION2) \
_FUNCTION_START(NAME, NUMBER) if (CONDITION1) {ACTION1;} else if (CONDITION2) {ACTION2;} else _FUNCTION_END(0, CP_CHARACTER_ERROR)

#define DEFINE_FUNCTION_CND3(NAME, NUMBER, CONDITION1, ACTION1, CONDITION2, ACTION2, CONDITION3, ACTION3) \
_FUNCTION_START(NAME, NUMBER) if (CONDITION1) {ACTION1;} else if (CONDITION2) {ACTION2;} else if (CONDITION3) {ACTION3;} else _FUNCTION_END(0, CP_CHARACTER_ERROR)

#define DEFINE_FUNCTION_CND4(NAME, NUMBER, CONDITION1, ACTION1, CONDITION2, ACTION2, CONDITION3, ACTION3, CONDITION4, ACTION4) \
_FUNCTION_START(NAME, NUMBER) if (CONDITION1) {ACTION1;} else if (CONDITION2) {ACTION2;} else if (CONDITION3) {ACTION3;} else if (CONDITION4) {ACTION4;} else _FUNCTION_END(0, CP_CHARACTER_ERROR)

#define DEFINE_FUNCTION_STRG(NAME, NUMBER, STRING, ACTION) \
_FUNCTION_START(NAME, NUMBER) {char* pattern = STRING; uint32_t length = strlen(pattern); if (c == pattern[COUNT]) {INC_COUNT; if (COUNT == length) {ACTION;}} else _FUNCTION_END(0, CP_CHARACTER_ERROR)}

/* Macro definitions for various character combinations */

#define IS(X) (c == X)

#define VALUE (c - '0')

#define ISDIGIT ('0' <= c && c <= '9')

#define ISNUMBER (ISDIGIT || IS('-'))

/* Macro definitions for error cases */

#define VALUE_ERROR \
state->state = 0; state->status = CP_VALUE_ERROR

/* Custom time constants */

#define MINUTES_IN_HOUR                 60
#define SECONDS_IN_HOUR                 3600
#define MINUTES_IN_DAY                  1440
#define MINUTES_IN_SIX_HOURS            (60 * 60)
#define SECONDS_IN_DAY                  (1440 * 60)

#define MAXIMUM_DURATION                MINUTES_IN_DAY
#define MAXIMUM_START_MINUTES           (MINUTES_IN_DAY - 1)

#define MONTH_JAN                       1
#define MONTH_FEB                       2
#define MONTH_DEC                       12

#define DAYS_IN_YEAR                    365
#define UNIX_EPOCH_START                1970

/* Custom variables */

static uint32_t day, month, year, timestamp;

static const uint32_t daysInMonth[12] = {31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};

/* Custom function to check configuration */

static inline bool isLeapYear(uint32_t year) {

    return (year & 3) == 0 && ((year % 25) != 0 || (year & 15) == 0);

}

static inline bool checkDate() {

    timestamp = 0;

    if (month < MONTH_JAN || month > MONTH_DEC) return false;

    uint32_t dayInCurrentMonth = daysInMonth[month - 1] + (month == MONTH_FEB && isLeapYear(year) ? 1 : 0);

    if (day == 0 || day > dayInCurrentMonth) return false;

    for (uint32_t y = UNIX_EPOCH_START; y < year; y += 1) timestamp += SECONDS_IN_DAY * (isLeapYear(y) ? DAYS_IN_YEAR + 1 : DAYS_IN_YEAR);

    for (uint32_t m = 0; m < month - 1; m += 1) timestamp += SECONDS_IN_DAY * daysInMonth[m];

    if (isLeapYear(year) && month > MONTH_FEB) timestamp += SECONDS_IN_DAY;

    timestamp += SECONDS_IN_DAY * (day - 1);

    return true;

}

/* Define jump table functions for configuration settings */
/* Must use CLEAR_BUFFER before DEFINE_FUNCTION_STRG or ADD_TO_BUFFER functionality */

DEFINE_FUNCTION_INIT(CP, 00, IS('{'))
DEFINE_FUNCTION_STRG(CP, 01, "gain:", INC_STATE)
DEFINE_FUNCTION_STEP(CP, 02, IS('0') || IS('1') || IS('2') || IS('3') || IS('4'), configSettings->gain = VALUE; CLEAR_BUFFER; INC_STATE)
DEFINE_FUNCTION_STRG(CP, 03, ",sampleRate:", CLEAR_BUFFER; INC_STATE)
DEFINE_FUNCTION_ELSE(CP, 04, ISDIGIT, ADD_TO_BUFFER, IS(','), configSettings->sampleRate = atoi(BUFFER); CLEAR_BUFFER; INC_STATE)
DEFINE_FUNCTION_STRG(CP, 05, "enableDailyFolders:", INC_STATE)
DEFINE_FUNCTION_STEP(CP, 06, IS('0') || IS('1'), configSettings->enableDailyFolders = VALUE; CLEAR_BUFFER; INC_STATE)
DEFINE_FUNCTION_STRG(CP, 07, ",enableLowGainRange:", INC_STATE)
DEFINE_FUNCTION_STEP(CP, 08, IS('0') || IS('1'), configSettings->enableLowGainRange = VALUE; CLEAR_BUFFER; INC_STATE)
DEFINE_FUNCTION_STRG(CP, 09, ",enableMagneticSwitch:", INC_STATE)
DEFINE_FUNCTION_STEP(CP, 10, IS('0') || IS('1'), configSettings->enableMagneticSwitch = VALUE; CLEAR_BUFFER; INC_STATE)
DEFINE_FUNCTION_STRG(CP, 11, ",disable48HzDCBlockingFilter:", INC_STATE)
DEFINE_FUNCTION_STEP(CP, 12, IS('0') || IS('1'), configSettings->disable48HzDCBlockingFilter = VALUE; CLEAR_BUFFER; INC_STATE)
DEFINE_FUNCTION_STRG(CP, 13, ",enableAlternativeBatteryVoltageRange:", INC_STATE)
DEFINE_FUNCTION_STEP(CP, 14, IS('0') || IS('1'), configSettings->batteryLevelDisplayType = VALUE == 1 ? NIMH_LIPO_BATTERY_VOLTAGE : BATTERY_LEVEL; CLEAR_BUFFER; INC_STATE)
DEFINE_FUNCTION_STRG(CP, 15, ",initialFixDuration:", CLEAR_BUFFER; INC_STATE)
DEFINE_FUNCTION_ELSE(CP, 16, ISDIGIT, ADD_TO_BUFFER, IS(','), configSettings->initialFixDuration = MIN(SECONDS_IN_HOUR, atoi(BUFFER)); CLEAR_BUFFER; INC_STATE)
DEFINE_FUNCTION_STRG(CP, 17, "intervalToNextFixAttempt:", CLEAR_BUFFER; INC_STATE)
DEFINE_FUNCTION_ELSE(CP, 18, ISDIGIT, ADD_TO_BUFFER, IS(','), configSettings->intervalToNextFixAttempt = MIN(SECONDS_IN_HOUR, atoi(BUFFER)); CLEAR_BUFFER; INC_STATE)
DEFINE_FUNCTION_STRG(CP, 19, "recordingFixDuration:", CLEAR_BUFFER; INC_STATE)
DEFINE_FUNCTION_ELSE(CP, 20, ISDIGIT, ADD_TO_BUFFER, IS(','), configSettings->recordingFixDuration = MIN(SECONDS_IN_HOUR, atoi(BUFFER)); INC_STATE)
DEFINE_FUNCTION_ELSE(CP, 21, IS('s'), configSettings->useFixedRecordingPeriods = false; CLEAR_BUFFER; ADD_STATE(17), IS('r'), configSettings->useFixedRecordingPeriods = true; CLEAR_BUFFER; INC_STATE)
DEFINE_FUNCTION_STRG(CP, 22, "ecordingPeriods:[", CLEAR_BUFFER; INC_STATE)
DEFINE_FUNCTION_STRG(CP, 23, "{startTime:\"", CLEAR_BUFFER; INC_STATE)
DEFINE_FUNCTION_STEP(CP, 24, IS('0') || IS('1') || IS('2'), ADD_TO_BUFFER; INC_STATE)
DEFINE_FUNCTION_STEP(CP, 25, ISDIGIT, ADD_TO_BUFFER; configSettings->recordingPeriods[INDEX].startMinutes = MINUTES_IN_HOUR * atoi(BUFFER); INC_STATE)
DEFINE_FUNCTION_STEP(CP, 26, IS(':'), CLEAR_BUFFER; INC_STATE)
DEFINE_FUNCTION_STEP(CP, 27, IS('0') || IS('1') || IS('2') || IS('3') || IS('4') || IS('5'), ADD_TO_BUFFER; INC_STATE)
DEFINE_FUNCTION_STEP(CP, 28, ISDIGIT, ADD_TO_BUFFER; configSettings->recordingPeriods[INDEX].startMinutes = MIN(MAXIMUM_START_MINUTES, configSettings->recordingPeriods[INDEX].startMinutes + atoi(BUFFER)); CLEAR_BUFFER; INC_STATE)
DEFINE_FUNCTION_STRG(CP, 29, "\",stopTime:\"", CLEAR_BUFFER; INC_STATE)
DEFINE_FUNCTION_STEP(CP, 30, IS('0') || IS('1') || IS('2'), ADD_TO_BUFFER; INC_STATE)
DEFINE_FUNCTION_STEP(CP, 31, ISDIGIT, ADD_TO_BUFFER; configSettings->recordingPeriods[INDEX].durationMinutes = MINUTES_IN_HOUR * atoi(BUFFER); INC_STATE)
DEFINE_FUNCTION_STEP(CP, 32, IS(':'), CLEAR_BUFFER; INC_STATE)
DEFINE_FUNCTION_STEP(CP, 33, IS('0') || IS('1') || IS('2') || IS('3') || IS('4') || IS('5'), ADD_TO_BUFFER; INC_STATE)
DEFINE_FUNCTION_STEP(CP, 34, ISDIGIT, ADD_TO_BUFFER; configSettings->recordingPeriods[INDEX].durationMinutes += atoi(BUFFER); if (configSettings->recordingPeriods[INDEX].durationMinutes <= configSettings->recordingPeriods[INDEX].startMinutes) configSettings->recordingPeriods[INDEX].durationMinutes += MINUTES_IN_DAY; configSettings->recordingPeriods[INDEX].durationMinutes -= configSettings->recordingPeriods[INDEX].startMinutes; CLEAR_BUFFER; INC_STATE)
DEFINE_FUNCTION_STRG(CP, 35, "\"}", INC_STATE)
DEFINE_FUNCTION_ELSE(CP, 36, IS(',') && INDEX < (MAX_RECORDING_PERIODS - 1), INC_INDEX; CLEAR_BUFFER; SUB_STATE(13), IS(']'), configSettings->activeRecordingPeriods = INDEX + 1; INC_STATE)
DEFINE_FUNCTION_ELSE(CP, 37, IS('}'), SET_STATUS_SUCCESS, IS(','), ADD_STATE(17))
DEFINE_FUNCTION_STRG(CP, 38, "unRecording:{sunRecordingMode:", INC_STATE)
DEFINE_FUNCTION_STEP(CP, 39, IS('0') || IS('1') || IS('2') | IS('3') || IS('4'), configSettings->sunRecordingMode = VALUE; CLEAR_BUFFER; INC_STATE)
DEFINE_FUNCTION_STRG(CP, 40, ",sunRoundingMinutes:", CLEAR_BUFFER; INC_STATE)
DEFINE_FUNCTION_ELSE(CP, 41, ISDIGIT, ADD_TO_BUFFER, IS(','), configSettings->sunRoundingMinutes = MIN(MINUTES_IN_HOUR, atoi(BUFFER)); CLEAR_BUFFER; INC_STATE)
DEFINE_FUNCTION_STRG(CP, 42, "sunriseInterval:{", CLEAR_BUFFER; INC_STATE)
DEFINE_FUNCTION_STRG(CP, 43, "beforeMinutes:", CLEAR_BUFFER; INC_STATE)
DEFINE_FUNCTION_ELSE(CP, 44, ISDIGIT, ADD_TO_BUFFER, IS(','), configSettings->sunriseInterval.beforeMinutes = (uint32_t)MIN(MINUTES_IN_SIX_HOURS, atoi(BUFFER)); CLEAR_BUFFER; INC_STATE)
DEFINE_FUNCTION_STRG(CP, 45, "afterMinutes:", CLEAR_BUFFER; INC_STATE)
DEFINE_FUNCTION_ELSE(CP, 46, ISDIGIT, ADD_TO_BUFFER, IS('}'), configSettings->sunriseInterval.afterMinutes = (uint32_t)MIN(MINUTES_IN_SIX_HOURS, atoi(BUFFER)); CLEAR_BUFFER; INC_STATE)
DEFINE_FUNCTION_STRG(CP, 47, ",sunsetInterval:{", CLEAR_BUFFER; INC_STATE)
DEFINE_FUNCTION_STRG(CP, 48, "beforeMinutes:", CLEAR_BUFFER; INC_STATE)
DEFINE_FUNCTION_ELSE(CP, 49, ISDIGIT, ADD_TO_BUFFER, IS(','), configSettings->sunsetInterval.beforeMinutes = (uint32_t)MIN(MINUTES_IN_SIX_HOURS, atoi(BUFFER)); CLEAR_BUFFER; INC_STATE)
DEFINE_FUNCTION_STRG(CP, 50, "afterMinutes:", CLEAR_BUFFER; INC_STATE)
DEFINE_FUNCTION_ELSE(CP, 51, ISDIGIT, ADD_TO_BUFFER, IS('}'), configSettings->sunsetInterval.afterMinutes = (uint32_t)MIN(MINUTES_IN_SIX_HOURS, atoi(BUFFER)); INC_STATE)
DEFINE_FUNCTION_STEP(CP, 52, IS('}'), INC_STATE);
DEFINE_FUNCTION_ELSE(CP, 53, IS('}'), SET_STATUS_SUCCESS, IS(','), INC_STATE);
DEFINE_FUNCTION_ELSE(CP, 54, IS('f'), INC_STATE; CLEAR_BUFFER, IS('l'), CLEAR_BUFFER; ADD_STATE(13))
DEFINE_FUNCTION_STRG(CP, 55, "irstRecordingDate:", INC_STATE)
DEFINE_FUNCTION_STEP(CP, 56, IS('\"'), INC_STATE; CLEAR_BUFFER)
DEFINE_FUNCTION_STEP(CP, 57, IS('0') || IS('1') || IS('2') || IS('3'), ADD_TO_BUFFER; INC_STATE)
DEFINE_FUNCTION_STEP(CP, 58, ISDIGIT, ADD_TO_BUFFER; day = atoi(BUFFER); INC_STATE)
DEFINE_FUNCTION_STEP(CP, 59, IS('/'), INC_STATE; CLEAR_BUFFER)
DEFINE_FUNCTION_STEP(CP, 60, IS('0') || IS('1'), ADD_TO_BUFFER; INC_STATE)
DEFINE_FUNCTION_STEP(CP, 61, ISDIGIT, ADD_TO_BUFFER; month = atoi(BUFFER); CLEAR_BUFFER; INC_STATE)
DEFINE_FUNCTION_STRG(CP, 62, "/202", INC_STATE)
DEFINE_FUNCTION_STEP(CP, 63, ISDIGIT, year = 2020 + VALUE; if (checkDate()) {configSettings->earliestRecordingTime = timestamp; INC_STATE;} else {VALUE_ERROR;})
DEFINE_FUNCTION_STEP(CP, 64, IS('\"'), INC_STATE)
DEFINE_FUNCTION_ELSE(CP, 65, IS('}'), SET_STATUS_SUCCESS, IS(','), INC_STATE)
DEFINE_FUNCTION_STEP(CP, 66, IS('l'), INC_STATE; CLEAR_BUFFER)
DEFINE_FUNCTION_STRG(CP, 67, "astRecordingDate:", INC_STATE)
DEFINE_FUNCTION_STEP(CP, 68, IS('\"'), INC_STATE; CLEAR_BUFFER)
DEFINE_FUNCTION_STEP(CP, 69, IS('0') || IS('1') || IS('2') || IS('3'), ADD_TO_BUFFER; INC_STATE)
DEFINE_FUNCTION_STEP(CP, 70, ISDIGIT, ADD_TO_BUFFER; day = atoi(BUFFER); INC_STATE)
DEFINE_FUNCTION_STEP(CP, 71, IS('/'), INC_STATE; CLEAR_BUFFER)
DEFINE_FUNCTION_STEP(CP, 72, IS('0') || IS('1'), ADD_TO_BUFFER; INC_STATE)
DEFINE_FUNCTION_STEP(CP, 73, ISDIGIT, ADD_TO_BUFFER; month = atoi(BUFFER); CLEAR_BUFFER; INC_STATE)
DEFINE_FUNCTION_STRG(CP, 74, "/202", INC_STATE)
DEFINE_FUNCTION_STEP(CP, 75, ISDIGIT, year = 2020 + VALUE; if (checkDate()) {configSettings->latestRecordingTime = timestamp + SECONDS_IN_DAY; INC_STATE;} else {VALUE_ERROR;})
DEFINE_FUNCTION_STEP(CP, 76, IS('\"'), INC_STATE)
DEFINE_FUNCTION_STEP(CP, 77, IS('}'), SET_STATUS_SUCCESS)


static void (*CPfunctions[])(char, CP_parserState_t*, CP_configSettings_t*) = {CP00, CP01, CP02, CP03, CP04, CP05, CP06, CP07, \
                                                                               CP08, CP09, CP10, CP11, CP12, CP13, CP14, CP15, \
                                                                               CP16, CP17, CP18, CP19, CP20, CP21, CP22, CP23, \
                                                                               CP24, CP25, CP26, CP27, CP28, CP29, CP30, CP31, \
                                                                               CP32, CP33, CP34, CP35, CP36, CP37, CP38, CP39, \
                                                                               CP40, CP41, CP42, CP43, CP44, CP45, CP46, CP47, \
                                                                               CP48, CP49, CP50, CP51, CP52, CP53, CP54, CP55, \
                                                                               CP56, CP57, CP58, CP59, CP60, CP61, CP62, CP63, \
                                                                               CP64, CP65, CP66, CP67, CP68, CP69, CP70, CP71, \
                                                                               CP72, CP73, CP74, CP75, CP76, CP77};

/* Define parser */

static CP_parserState_t parserState;

void ConfigParser_reset() {

    memset(&parserState, 0, sizeof(CP_parserState_t));

}

void ConfigParser_getCurrentState(uint32_t *line, uint32_t *position, char *character) {

    if (line) *line = parserState.line + 1;
    if (position) *position = parserState.position;
    if (character) *character = parserState.character;

    return;

}

CP_parserStatus_t ConfigParser_parse(char c, CP_configSettings_t *settings) {

    CPfunctions[parserState.state](c, &parserState, settings);

    return parserState.status;

}
