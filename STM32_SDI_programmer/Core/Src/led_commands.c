#include "led_commands.h"
#include "sdi_logging.h"


Led leds[7] =
{
    {GPIOA, GPIO_PIN_8},
    {GPIOA, GPIO_PIN_9},
    {GPIOB, GPIO_PIN_11},
    {GPIOB, GPIO_PIN_12},
    {GPIOB, GPIO_PIN_13},
    {GPIOB, GPIO_PIN_14},
    {GPIOB, GPIO_PIN_15}
};

uint8_t HexDisplayCode[17] = {
/*//  abcdefg  .*/
  0b1111011,  // 0
  0b0011000,  // 1
  0b0110111,  // 2
  0b0111110,  // 3
  0b1011100,  // 4
  0b1101101,  // 5
  0b1101111,  // 6
  0b0111000,  // 7
  0b1111111,  // 8
  0b1111101,  // 9
  0b1111110,  // A
  0b1100111,  // B
  0b0001111,  // C
  0b1110011,  // D
  0b1001111,  // E
  0b1001110,  // F
  0b0000000   // .
};

void setDigit(int digit)
{
    char log[250];
    clearLeds();

    for (int i=0; i < 7; i++)
    {
    if ((HexDisplayCode[digit] & 0x1) == 1)
    {
        //*led[i].port |= (led[i].mask);
        //sprintf(log, "Setting digit inc %d", i);
        //printLog(log);
        writeDigitPin(i);
    }
    HexDisplayCode[digit] = HexDisplayCode[digit] >> 1;
    }

}

void writeDigitPin(int digit)
{
    HAL_GPIO_WritePin(leds[digit].port, leds[digit].pin, 1);
}

void clearLeds()
{
    for(int i=0; i < 7; i++)
    {
        HAL_GPIO_WritePin(leds[i].port, leds[i].pin, 0);
    }
}