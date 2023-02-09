#include "sdi_logging.h"
#include <string.h>

extern UART_HandleTypeDef huart2;

void printLog(const char* message)
{
    int msgLen = strlen(message);
    char logMessage[msgLen + 3];

    if(msgLen == 2 && message[1] != '\0')
    {
        sprintf(logMessage, "%c\r\n", *message);
    }
    else if(msgLen == 3 && message[2] != '\0')
    {
        sprintf(logMessage, "%c\r\n", *message);
    }
    else
    {
        sprintf(logMessage, "%s\r\n", message);
    }
    HAL_UART_Transmit(&huart2, (uint8_t *)logMessage, strlen(logMessage), 100);
}

void printChar(char message)
{
    char logMessage[4];
    sprintf(logMessage, "%c\r\n", message);
    HAL_UART_Transmit(&huart2, (uint8_t *)logMessage, strlen(logMessage), 100);
}