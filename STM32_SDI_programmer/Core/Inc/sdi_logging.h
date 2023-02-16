/**
 * Copyright (C) 2021 SemiosBIO Technologies Inc.
 * All Rights Reserved.
 *
 * Unauthorized copying of these files via any medium is strictly prohibited.
 * Proprietary & Confidential
 */

/*
 * @file sdi_logging.h
 * @brief Logging interface to ouput logs to console
 *
 * @author: Sterling Smith
 * @date: 10 Feb 2022
 */

#ifndef SEMIOS_SDI_LOGGING_H
#define SEMIOS_SDI_LOGGING_H

#include "main.h"

#define NO_DEBUG      0
#define DEBUG         1
#define DEBUG_LOGS    NO_DEBUG

/**
 * @brief Function to output a string to the console
 *
 * @param[in] message Log message to be printed
 */
void printLog(const char *message);

/**
 * @brief Function to output a character to the console
 *
 * @param[in] message Character to be printed
 */
void printChar(char message);

/**
 * @brief Function to output a debug log to the console
 *
 * @param[in] message Log message to be printed
 */
void printDebug(const char *message);

#endif /* SEMIOS_SDI_LOGGING_H */
