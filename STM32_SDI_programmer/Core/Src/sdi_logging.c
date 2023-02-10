/**
 * Copyright (C) 2021 SemiosBIO Technologies Inc.
 * All Rights Reserved.
 *
 * Unauthorized copying of these files via any medium is strictly prohibited.
 * Proprietary & Confidential
 */

/*
 * @file sdi_logging.c
 * @brief Logging interface to ouput logs to console
 *
 * @author: Sterling Smith
 * @date: 10 Feb 2022
 */

#include "sdi_logging.h"
#include <string.h>

//Console UART
extern UART_HandleTypeDef huart2;

void printLog(const char *message)
{
    int  msgLen = strlen(message);
    char logMessage[msgLen + 3];

    sprintf(logMessage, "%s\r\n", message);
    HAL_UART_Transmit(&huart2, (uint8_t *)logMessage, strlen(logMessage), 100);
}

void printChar(char message)
{
    char logMessage[4];

    sprintf(logMessage, "%c\r\n", message);
    HAL_UART_Transmit(&huart2, (uint8_t *)logMessage, strlen(logMessage), 100);
}

void printDebug(const char *message)
{
    if (DEBUG_LOGS == DEBUG)
    {
        int  msgLen = strlen(message);
        char logMessage[msgLen + 3];

        sprintf(logMessage, "%s\r\n", message);
        HAL_UART_Transmit(&huart2, (uint8_t *)logMessage, strlen(logMessage), 100);
    }
}
