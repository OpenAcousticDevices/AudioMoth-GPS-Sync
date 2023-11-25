/****************************************************************************
 * configparser.c
 * openacousticdevices.info
 * October 2022
 *****************************************************************************/

#include <string.h>
#include <stdlib.h>

#include "configparser.h"

#define MAX_BUFFER_LENGTH       32

/* Custom time constants */

#define MINUTES_IN_HOUR         60

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
    bool comment;
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
void NAME ## NUMBER (char c, CP_parserState_t *state, CP_configSettings_t *configSettings) {state->character = c; state->position += 1; if (c == '#') {state->comment = true;}; if (c == '\n') {state->comment = false; state->line += 1; state->position = 0;}; if (state->comment == true || c == ' ' || c == '\t' || c == '\n' || c == '\r') {} else

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

/* Define jump table functions for configuration settings */
/* Must use CLEAR_BUFFER before DEFINE_FUNCTION_STRG or ADD_TO_BUFFER functionality */

DEFINE_FUNCTION_INIT(CP, 00, IS('{'))
DEFINE_FUNCTION_STRG(CP, 01, "gain:", INC_STATE)
DEFINE_FUNCTION_STEP(CP, 02, IS('0') || IS('1') || IS('2') || IS('3') || IS('4'), configSettings->gain = VALUE; CLEAR_BUFFER; INC_STATE)
DEFINE_FUNCTION_STRG(CP, 03, ",sampleRate:", CLEAR_BUFFER; INC_STATE)
DEFINE_FUNCTION_ELSE(CP, 04, COUNT < 6 && ISDIGIT, ADD_TO_BUFFER, IS(','), configSettings->sampleRate = atoi(BUFFER); CLEAR_BUFFER; INC_STATE)
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
DEFINE_FUNCTION_ELSE(CP, 16, COUNT < 4 && ISDIGIT, ADD_TO_BUFFER, IS(','), configSettings->initialFixDuration = atoi(BUFFER); CLEAR_BUFFER; INC_STATE)
DEFINE_FUNCTION_STRG(CP, 17, "intervalToNextFixAttempt:", CLEAR_BUFFER; INC_STATE)
DEFINE_FUNCTION_ELSE(CP, 18, COUNT < 4 && ISDIGIT, ADD_TO_BUFFER, IS(','), configSettings->intervalToNextFixAttempt = atoi(BUFFER); CLEAR_BUFFER; INC_STATE)
DEFINE_FUNCTION_STRG(CP, 19, "recordingFixDuration:", CLEAR_BUFFER; INC_STATE)
DEFINE_FUNCTION_ELSE(CP, 20, COUNT < 4 && ISDIGIT, ADD_TO_BUFFER, IS(','), configSettings->recordingFixDuration = atoi(BUFFER); INC_STATE)
DEFINE_FUNCTION_ELSE(CP, 21, IS('s'), INC_STATE, IS('r'), configSettings->disableSleepRecordCycle = true; configSettings->useFixedRecordingPeriods = true; CLEAR_BUFFER; ADD_STATE(8))
DEFINE_FUNCTION_ELSE(CP, 22, IS('l'), CLEAR_BUFFER; INC_STATE, IS('u'), configSettings->disableSleepRecordCycle = true; CLEAR_BUFFER; ADD_STATE(27))
DEFINE_FUNCTION_STRG(CP, 23, "eepRecordCycle:{sleepDuration:", CLEAR_BUFFER; INC_STATE)
DEFINE_FUNCTION_ELSE(CP, 24, COUNT < 4 && ISDIGIT, ADD_TO_BUFFER, IS(','), configSettings->sleepDuration = atoi(BUFFER); CLEAR_BUFFER; INC_STATE)
DEFINE_FUNCTION_STRG(CP, 25, "recordDuration:", CLEAR_BUFFER; INC_STATE)
DEFINE_FUNCTION_ELSE(CP, 26, COUNT < 4 && ISDIGIT, ADD_TO_BUFFER, IS('}'), configSettings->recordDuration = atoi(BUFFER); INC_STATE)
DEFINE_FUNCTION_STEP(CP, 27, IS(','), INC_STATE)
DEFINE_FUNCTION_ELSE(CP, 28, IS('s'), ADD_STATE(20), IS('r'), configSettings->useFixedRecordingPeriods = true; CLEAR_BUFFER; INC_STATE)
DEFINE_FUNCTION_STRG(CP, 29, "ecordingPeriods:[", CLEAR_BUFFER; INC_STATE)
DEFINE_FUNCTION_STRG(CP, 30, "{startTime:\"", CLEAR_BUFFER; INC_STATE)
DEFINE_FUNCTION_STEP(CP, 31, IS('0') || IS('1') || IS('2'), ADD_TO_BUFFER; INC_STATE)
DEFINE_FUNCTION_STEP(CP, 32, ISDIGIT, ADD_TO_BUFFER; configSettings->recordingPeriods[INDEX].startMinutes = MINUTES_IN_HOUR * atoi(BUFFER); INC_STATE)
DEFINE_FUNCTION_STEP(CP, 33, IS(':'), CLEAR_BUFFER; INC_STATE)
DEFINE_FUNCTION_STEP(CP, 34, IS('0') || IS('1') || IS('2') || IS('3') || IS('4') || IS('5'), ADD_TO_BUFFER; INC_STATE)
DEFINE_FUNCTION_STEP(CP, 35, ISDIGIT, ADD_TO_BUFFER; configSettings->recordingPeriods[INDEX].startMinutes += atoi(BUFFER); CLEAR_BUFFER; INC_STATE)
DEFINE_FUNCTION_STRG(CP, 36, "\",", CLEAR_BUFFER; INC_STATE)
DEFINE_FUNCTION_ELSE(CP, 37, IS('s'), CLEAR_BUFFER; INC_STATE, IS('e'), CLEAR_BUFFER; ADD_STATE(2));
DEFINE_FUNCTION_STRG(CP, 38, "topTime:\"", CLEAR_BUFFER; ADD_STATE(2));
DEFINE_FUNCTION_STRG(CP, 39, "ndTime:\"", CLEAR_BUFFER; INC_STATE);
DEFINE_FUNCTION_STEP(CP, 40, IS('0') || IS('1') || IS('2'), ADD_TO_BUFFER; INC_STATE)
DEFINE_FUNCTION_STEP(CP, 41, ISDIGIT, ADD_TO_BUFFER; configSettings->recordingPeriods[INDEX].endMinutes = MINUTES_IN_HOUR * atoi(BUFFER); INC_STATE)
DEFINE_FUNCTION_STEP(CP, 42, IS(':'), CLEAR_BUFFER; INC_STATE)
DEFINE_FUNCTION_STEP(CP, 43, IS('0') || IS('1') || IS('2') || IS('3') || IS('4') || IS('5'), ADD_TO_BUFFER; INC_STATE)
DEFINE_FUNCTION_STEP(CP, 44, ISDIGIT, ADD_TO_BUFFER; configSettings->recordingPeriods[INDEX].endMinutes += atoi(BUFFER); CLEAR_BUFFER; INC_STATE)
DEFINE_FUNCTION_STRG(CP, 45, "\"}", INC_STATE)
DEFINE_FUNCTION_ELSE(CP, 46, IS(',') && INDEX < (MAX_RECORDING_PERIODS - 1), INC_INDEX; CLEAR_BUFFER; SUB_STATE(16), IS(']'), configSettings->activeRecordingPeriods = INDEX + 1; INC_STATE)
DEFINE_FUNCTION_ELSE(CP, 47, IS('}'), SET_STATUS_SUCCESS, IS(','), ADD_STATE(18))
DEFINE_FUNCTION_STEP(CP, 48, IS('u'), CLEAR_BUFFER; INC_STATE)
DEFINE_FUNCTION_STRG(CP, 49, "nRecording:{sunRecordingMode:", INC_STATE)
DEFINE_FUNCTION_STEP(CP, 50, IS('0') || IS('1') || IS('2') | IS('3') || IS('4'), configSettings->sunRecordingMode = VALUE; CLEAR_BUFFER; INC_STATE)
DEFINE_FUNCTION_STRG(CP, 51, ",sunRoundingMinutes:", CLEAR_BUFFER; INC_STATE)
DEFINE_FUNCTION_ELSE(CP, 52, COUNT < 2 && ISDIGIT, ADD_TO_BUFFER, IS(','), configSettings->sunRoundingMinutes = atoi(BUFFER); CLEAR_BUFFER; INC_STATE)
DEFINE_FUNCTION_STRG(CP, 53, "sunriseInterval:{", CLEAR_BUFFER; INC_STATE)
DEFINE_FUNCTION_STRG(CP, 54, "beforeMinutes:", CLEAR_BUFFER; INC_STATE)
DEFINE_FUNCTION_ELSE(CP, 55, COUNT < 3 && ISDIGIT, ADD_TO_BUFFER, IS(','), configSettings->sunriseInterval.beforeMinutes = atoi(BUFFER); CLEAR_BUFFER; INC_STATE)
DEFINE_FUNCTION_STRG(CP, 56, "afterMinutes:", CLEAR_BUFFER; INC_STATE)
DEFINE_FUNCTION_ELSE(CP, 57, COUNT < 3 && ISDIGIT, ADD_TO_BUFFER, IS('}'), configSettings->sunriseInterval.afterMinutes = atoi(BUFFER); CLEAR_BUFFER; INC_STATE)
DEFINE_FUNCTION_STRG(CP, 58, ",sunsetInterval:{", CLEAR_BUFFER; INC_STATE)
DEFINE_FUNCTION_STRG(CP, 59, "beforeMinutes:", CLEAR_BUFFER; INC_STATE)
DEFINE_FUNCTION_ELSE(CP, 60, COUNT < 3 && ISDIGIT, ADD_TO_BUFFER, IS(','), configSettings->sunsetInterval.beforeMinutes = atoi(BUFFER); CLEAR_BUFFER; INC_STATE)
DEFINE_FUNCTION_STRG(CP, 61, "afterMinutes:", CLEAR_BUFFER; INC_STATE)
DEFINE_FUNCTION_ELSE(CP, 62, COUNT < 3 && ISDIGIT, ADD_TO_BUFFER, IS('}'), configSettings->sunsetInterval.afterMinutes = atoi(BUFFER); INC_STATE)
DEFINE_FUNCTION_STEP(CP, 63, IS('}'), INC_STATE);
DEFINE_FUNCTION_ELSE(CP, 64, IS('}'), SET_STATUS_SUCCESS, IS(','), INC_STATE);
DEFINE_FUNCTION_ELSE(CP, 65, IS('f'), INC_STATE; CLEAR_BUFFER, IS('l'), CLEAR_BUFFER; ADD_STATE(13))
DEFINE_FUNCTION_STRG(CP, 66, "irstRecordingDate:", configSettings->firstRecordingDate = true; INC_STATE)
DEFINE_FUNCTION_STEP(CP, 67, IS('\"'), INC_STATE; CLEAR_BUFFER)
DEFINE_FUNCTION_STEP(CP, 68, IS('0') || IS('1') || IS('2') || IS('3'), ADD_TO_BUFFER; INC_STATE)
DEFINE_FUNCTION_STEP(CP, 69, ISDIGIT, ADD_TO_BUFFER; configSettings->firstRecordingDay = atoi(BUFFER); INC_STATE)
DEFINE_FUNCTION_STEP(CP, 70, IS('/'), INC_STATE; CLEAR_BUFFER)
DEFINE_FUNCTION_STEP(CP, 71, IS('0') || IS('1'), ADD_TO_BUFFER; INC_STATE)
DEFINE_FUNCTION_STEP(CP, 72, ISDIGIT, ADD_TO_BUFFER; configSettings->firstRecordingMonth = atoi(BUFFER); CLEAR_BUFFER; INC_STATE)
DEFINE_FUNCTION_STRG(CP, 73, "/202", INC_STATE)
DEFINE_FUNCTION_STEP(CP, 74, ISDIGIT, configSettings->firstRecordingYear = 2020 + VALUE; INC_STATE)
DEFINE_FUNCTION_STEP(CP, 75, IS('\"'), INC_STATE)
DEFINE_FUNCTION_ELSE(CP, 76, IS('}'), SET_STATUS_SUCCESS, IS(','), INC_STATE)
DEFINE_FUNCTION_STEP(CP, 77, IS('l'), INC_STATE; CLEAR_BUFFER)
DEFINE_FUNCTION_STRG(CP, 78, "astRecordingDate:", configSettings->lastRecordingDate = true; INC_STATE)
DEFINE_FUNCTION_STEP(CP, 79, IS('\"'), INC_STATE; CLEAR_BUFFER)
DEFINE_FUNCTION_STEP(CP, 80, IS('0') || IS('1') || IS('2') || IS('3'), ADD_TO_BUFFER; INC_STATE)
DEFINE_FUNCTION_STEP(CP, 81, ISDIGIT, ADD_TO_BUFFER; configSettings->lastRecordingDay = atoi(BUFFER); INC_STATE)
DEFINE_FUNCTION_STEP(CP, 82, IS('/'), INC_STATE; CLEAR_BUFFER)
DEFINE_FUNCTION_STEP(CP, 83, IS('0') || IS('1'), ADD_TO_BUFFER; INC_STATE)
DEFINE_FUNCTION_STEP(CP, 84, ISDIGIT, ADD_TO_BUFFER; configSettings->lastRecordingMonth = atoi(BUFFER); CLEAR_BUFFER; INC_STATE)
DEFINE_FUNCTION_STRG(CP, 85, "/202", INC_STATE)
DEFINE_FUNCTION_STEP(CP, 86, ISDIGIT, configSettings->lastRecordingYear = 2020 + VALUE; INC_STATE)
DEFINE_FUNCTION_STEP(CP, 87, IS('\"'), INC_STATE)
DEFINE_FUNCTION_STEP(CP, 88, IS('}'), SET_STATUS_SUCCESS)

static void (*CPfunctions[])(char, CP_parserState_t*, CP_configSettings_t*) = {CP00, CP01, CP02, CP03, CP04, CP05, CP06, CP07, \
                                                                               CP08, CP09, CP10, CP11, CP12, CP13, CP14, CP15, \
                                                                               CP16, CP17, CP18, CP19, CP20, CP21, CP22, CP23, \
                                                                               CP24, CP25, CP26, CP27, CP28, CP29, CP30, CP31, \
                                                                               CP32, CP33, CP34, CP35, CP36, CP37, CP38, CP39, \
                                                                               CP40, CP41, CP42, CP43, CP44, CP45, CP46, CP47, \
                                                                               CP48, CP49, CP50, CP51, CP52, CP53, CP54, CP55, \
                                                                               CP56, CP57, CP58, CP59, CP60, CP61, CP62, CP63, \
                                                                               CP64, CP65, CP66, CP67, CP68, CP69, CP70, CP71, \
                                                                               CP72, CP73, CP74, CP75, CP76, CP77, CP78, CP79, \
                                                                               CP80, CP81, CP82, CP83, CP84, CP85, CP86, CP87, \
                                                                               CP88};

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
