#include "main.h"

typedef struct Led
{
    GPIO_TypeDef* port;
    uint16_t pin;
} Led;

void setDigit(char digit);

void writeDigitPin(int digit);

void clearLeds();