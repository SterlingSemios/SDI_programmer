/**
 * Copyright (C) 2021 SemiosBIO Technologies Inc.
 * All Rights Reserved.
 *
 * Unauthorized copying of these files via any medium is strictly prohibited.
 * Proprietary & Confidential
 */

/*
 * @file led_driver.c
 * @brief Functions to control the LED display
 *
 * @author: Sterling Smith
 * @date: 10 Feb 2022
 */

#include "led_driver.h"
#include "string.h"

#define MAX_SENSOR_ADDRESS 3

Led ledConfig[7] =
{
    {GPIOA, GPIO_PIN_8},
    {GPIOA, GPIO_PIN_9},
    {GPIOB, GPIO_PIN_11},
    {GPIOB, GPIO_PIN_12},
    {GPIOB, GPIO_PIN_13},
    {GPIOB, GPIO_PIN_14},
    {GPIOB, GPIO_PIN_15}
};

uint8_t HexDisplayCode[10] = {
  0b1111011,  // 0
  0b0011000,  // 1
  0b0110111,  // 2
  0b0111110,  // 3
  0b1011100,  // 4
  0b1101110,  // 5
  0b1101111,  // 6
  0b0111000,  // 7
  0b1111111,  // 8
  0b1111110,  // 9
};

static void writeDigitPin(int digit);

static void clearLeds();

void setDigit(char digit)
{
    clearLeds();
    char charArr[]=  {'0', '1', '2', '3'};

    for(int intDigit = 0; intDigit <= MAX_SENSOR_ADDRESS; intDigit++)
    {
        if(strncmp(&digit, &charArr[intDigit], 1) == 0)
        {
            for (int i=0; i < 7; i++)
            {
                if (HexDisplayCode[intDigit] & (1 << i))
                {
                    writeDigitPin(i);
                }
            }

            break;
        }
    }
}

static void writeDigitPin(int digit)
{
    HAL_GPIO_WritePin(ledConfig[digit].port, ledConfig[digit].pin, 1);
}

static void clearLeds()
{
    for(int i=0; i < 7; i++)
    {
        HAL_GPIO_WritePin(ledConfig[i].port, ledConfig[i].pin, 0);
    }
}