#include "main.h"
#include "sdi12_driver.h"



// typedef enum
// {
//     SDI12RetCode_BUSY = -10,         /**< @brief SDI12 bus busy */
//     SDI12RetCode_PWR_ERROR,          /**< @brief SDI12 power related error */
//     SDI12RetCode_TX_ERROR,           /**< @brief SDI12 transmission error */
//     SDI12RetCode_RX_ERROR,           /**< @brief SDI12 reception error */
//     SDI12RetCode_ADDRESS_IN_USE,     /**< @brief SDI12 address already in use */
//     SDI12RetCode_ADDRESS_DUPLICATE,  /**< @brief SDI12 address is duplicate - mainly used for address change */
//     SDI12RetCode_ADDRESS_INVALID,    /**< @brief SDI12 address invalid */
//     SDI12RetCode_CRC_ERROR,          /**< @brief SDI12 command response CRC does not match */
//     SDI12RetCode_INVALID,            /**< @brief SDI12 unknown / invalid behavior */
//     SDI12RetCode_ERROR,              /**< @brief SDI12 general failure or error */
//     SDI12RetCode_OK,                 /**< @brief SDI12 communication OK */
// }SDI12RetCode;

// typedef struct __Sdi12PortPinMap_t
// {
//     GPIO_TypeDef *pGpioPort; /**< @brief gpio port */
//     uint16_t      gpioPin;   /**< @brief gpio pin number */
// }
// Sdi12PortPinMap;

// typedef struct __Sdi12PinConfig_t
// {
//     Sdi12PortPinMap sdiTxEnb;                        /**< @brief sdi12 transmit enable pin */
//     Sdi12PortPinMap sdiTx;                           /**< @brief sdi12 transmission pin */
// }
// Sdi12PinConfig;

void readAddress();

void receiveRx(char sdiByte);

void enableSdi();

void sdi_BusComm(Sdi12Handle *hSdi12, char *rxBuf,
                uint8_t *rxSize, uint16_t rxTimeout);