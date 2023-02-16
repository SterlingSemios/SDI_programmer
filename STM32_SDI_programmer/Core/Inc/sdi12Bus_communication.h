/**
 * @file sdi12Bus_communication.h
 * @brief APIs and helper functions for SDI12 bus communication (Rx-Tx and related)
 *        (This is a stripped down version of the NodeY v3 SDI12 Driver)
 *        Refer: "https://semios.atlassian.net/wiki/x/IQCPU"
 *
 * @author: Prajakta Ranade
 * @date: 02 Nov 2020
 **/

#ifndef SDI12BUS_COMMUNICATION_H
#define SDI12BUS_COMMUNICATION_H

#include "main.h"

#include <stdio.h>
#include <string.h>
#include <stdbool.h>

#define NULL_PTR_CHECK(ptr, errNumber)    if (ptr == NULL) return errNumber; /**< @brief Macro for NULL pointer check */

/**
 * @brief Enum type for possible SDI12 return code / status code
 */
typedef enum
{
    SDI12RetCode_BUSY = -10,           /**< @brief SDI12 bus busy */
    SDI12RetCode_PWR_ERROR,            /**< @brief SDI12 power related error */
    SDI12RetCode_TX_ERROR,             /**< @brief SDI12 transmission error */
    SDI12RetCode_RX_ERROR,             /**< @brief SDI12 reception error */
    SDI12RetCode_ADDRESS_IN_USE,       /**< @brief SDI12 address already in use */
    SDI12RetCode_ADDRESS_DUPLICATE,    /**< @brief SDI12 address is duplicate - mainly used for address change */
    SDI12RetCode_ADDRESS_INVALID,      /**< @brief SDI12 address invalid */
    SDI12RetCode_CRC_ERROR,            /**< @brief SDI12 command response CRC does not match */
    SDI12RetCode_INVALID,              /**< @brief SDI12 unknown / invalid behavior */
    SDI12RetCode_ERROR,                /**< @brief SDI12 general failure or error */
    SDI12RetCode_OK,                   /**< @brief SDI12 communication OK */
}SDI12RetCode;

/**
 * @brief To specify port and pin mapping for SDI12
 */
typedef struct __Sdi12PortPinMap_t
{
    GPIO_TypeDef *pGpioPort;   /**< @brief gpio port */
    uint16_t      gpioPin;     /**< @brief gpio pin number */
}
Sdi12PortPinMap;

/**
 * @brief SDI12 pin configuration
 */
typedef struct __Sdi12PinConfig_t
{
    Sdi12PortPinMap sdiPwrMain;                        /**< @brief SDI12 5V power pin */
    Sdi12PortPinMap sdiTxEnb;                          /**< @brief SDI12 transmit enable pin */
    Sdi12PortPinMap sdiTx;                             /**< @brief SDI12 transmission pin */
}
Sdi12PinConfig;

/**
 * @brief Structure for SDI12 handle
 */
typedef struct __Sdi12Handle_t
{
    UART_HandleTypeDef *hUART;                    /**< @brief UART handle */
    Sdi12PinConfig      Sdi12PinCfg;              /**< @brief SDI12 bus related pin configuration */
    char                sdi12Address;             /**< @brief SDI12 sensor address */
    char                sdi12IdNewOrQuery;        /**< @brief This field is used for query and address change command only */
}
Sdi12Handle;

/**
 * @brief Structure to hold SDI12 responses
 */
typedef struct __Sdi12Receive_t
{
    char   *recBuf;    /**< @brief buffer to receive data */
    uint8_t recSize;   /**< @brief size of data received */
}Sdi12Receive;

/**
 * @brief Structure to hold transmission data/command
 */
typedef struct __Sdi12Transmit_t
{
    char   *pTxBuf;           /**< @brief Pointer to transmit buffer */
    uint8_t size;             /**< @brief Size of data for transmission */
}
Sdi12Transmit;

/**
 * @brief Enum type for SDI12 commands
 */
typedef enum
{
    SDI12_QUERY_ADDR,     /**< @brief Address Query command - ?! */
    SDI12_CHANGE_ADDR,    /**< @brief Change address command - aAb! */
}
SDI12Cmd;

/**
 * @brief Function to to have desired pin state
 *
 * @param[in] pin pointer to SDI12 pin configuration
 * @param[in] state desired status of the gpio pin
 */
void sdi12_WritePin(const Sdi12PortPinMap *pin, const GPIO_PinState state);

/**
 * @brief Function performing SDI12 bus communication-Creates command, transmits it
 *        and reads the response when ready
 *
 * @param[in] hSdi12 SDI12 handle
 * @param[out] rxBuf Buffer to fill in response received
 * @param[out] rxSize number of bytes received in response
 * @param[in] rxTimeout Timeout for reception wait
 * @param[in] commandNum enum type indicating command number being processed
 *
 * @return SDI12RetCode type
 *         SDI12RetCode_TX_ERROR - transmission related error
 *         SDI12RetCode_RX_ERROR - reception related error
 *         SDI12RetCode_ERROR - wakeup signal error
 *         SDI12RetCode_OK - All okay
 */
SDI12RetCode sdi12_BusCommunication(Sdi12Handle *hSdi12, char *rxBuf,
                                    uint8_t *rxSize, uint16_t rxTimeout,
                                    SDI12Cmd commandNum);

/**
 * @brief Gets called from UART RX interrupt
 *        Function collects received data character by character
 *
 * @param[in] character type received byte
 */
void sdi12_receiveRxByteFromIsr(char);

#endif /* SDI12BUS_COMMUNICATION_H */
