/****************************************************************************
 * configchecker.h
 * openacousticdevices.info
 * October 2023
 *****************************************************************************/

#ifndef __CONFIGCHECKER_H
#define __CONFIGCHECKER_H

#include "configparser.h"

bool ConfigChecker_checkAndFinalise(CP_configSettings_t *configSettings, char *buffer);

#endif /* __CONFIGCHECKER_H */
