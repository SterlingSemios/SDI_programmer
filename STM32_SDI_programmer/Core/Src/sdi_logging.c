#include "sdi_logging.h"
#include <string.h>

extern UART_HandleTypeDef huart2;

void printLog(const char* message)
{
    int msgLen = strlen(message);
    char logMessage[msgLen + 3];
    sprintf(logMessage, "%s\r\n", message);
    HAL_UART_Transmit(&huart2, (uint8_t *)logMessage, strlen(logMessage), 100);
}
