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

#define MAX_SENSOR_ADDRESS    4 //Sensor address of "4" corresponds to an "-" for address failed to read

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

uint8_t HexDisplayCode[10] =
{
    0b1111011, // 0
    0b0011000, // 1
    0b0110111, // 2
    0b0111110, // 3
    0b0000100, // -
};

/**
 * @brief Enables specified LED pin
 *
 * @param[in] digitInc LED pin to enable
 */
static void writeDigitPin(int digitInc);

/**
 * @brief Disables the LED display and resets pins
 */
static void clearLeds();

void setDigit(char digit)
{
    clearLeds();
    char charArr[] = {'0', '1', '2', '3', '8'};

    for (int intDigit = 0; intDigit <= MAX_SENSOR_ADDRESS; intDigit++)
    {
        if (strncmp(&digit, &charArr[intDigit], 1) == 0)
        {
            for (int i = 0; i < 7; i++)
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

static void writeDigitPin(int digitInc)
{
    HAL_GPIO_WritePin(ledConfig[digitInc].port, ledConfig[digitInc].pin, 1);
}

static void clearLeds()
{
    for (int i = 0; i < 7; i++)
    {
        HAL_GPIO_WritePin(ledConfig[i].port, ledConfig[i].pin, 0);
    }
}
