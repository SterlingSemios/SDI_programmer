/**
 * Copyright (C) 2021 SemiosBIO Technologies Inc.
 * All Rights Reserved.
 *
 * Unauthorized copying of these files via any medium is strictly prohibited.
 * Proprietary & Confidential
 */

/*
 * @file led_driver.h
 * @brief Functions to control the LED display
 *
 * @author: Sterling Smith
 * @date: 10 Feb 2022
 */

#ifndef LED_DRIVER
#define LED_DRIVER

#include "main.h"

typedef struct Led
{
    GPIO_TypeDef* port;
    uint16_t pin;
} Led;

/**
 * @brief Function to set the LED display to desired digit
 *
 * @param[in] digit Requested LED display digit
 */
void setDigit(char digit);

#endif /* LED_DRIVER */