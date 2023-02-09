/**
 * @file sdi12_driver.h
 * @brief API for SDI12
 *        Refer: "https://semios.atlassian.net/wiki/x/IQCPU"
 *
 * @author: Prajakta Ranade
 * @date: 26 Sept 2020
 **/

#ifndef SDI12_DRIVER_H
#define SDI12_DRIVER_H

#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include "main.h"

//#include "FreeRTOS.h"
//#include "mcu_def.h"

#define MAX_RESPONSE_SIZE                96                                  /**< @brief maximum sdi12 response size - as per standard max size is 83 for aD! command with CRC */

#define MAX_HW_SLOTS_SDI12               4                                   /**< @brief Maximum number of SDI12 slots present and supported by Hardware */
#define MAX_INDEX                        9                                   /**< @brief Maximum allowed index for measurement and data read commands */
// macros for identification data
#define SDI12_VERSION_BYTES              4                                   /**< @brief supported sdi12 version number of sensor */
#define SENSOR_VENDOR_NAME_BYTES         9                                   /**< @brief hold vendor name of sensor */
#define SENSOR_MODEL_NUM_BYTES           7                                   /**< @brief hold model number of sensor */
#define SENSOR_VERSION_NUM_BYTES         4                                   /**< @brief hold sensor version number */
#define INDETIFICATION_METADATA_BYTES    14                                  /**< @brief hold meta / all the remaining optional data from response */

#define SDI12_FLAGS_CONCURRENT           (1 << 0)                            /**< @brief flag to indicate concurrent mode */
#define SDI12_FLAGS_CRC                  (1 << 1)                            /**< @brief flag to indicate request CRC mode */

#define ENABLE_ALL 1
#define ENABLE_ONE 0

#define NULL_PTR_CHECK(ptr, errNumber)    if (ptr == NULL) return errNumber; /**< @brief Macro for NULL pointer check */


/**
 * @brief Enum type for possible SDI12 return code / status code
 */
typedef enum
{
    SDI12RetCode_BUSY = -10,         /**< @brief SDI12 bus busy */
    SDI12RetCode_PWR_ERROR,          /**< @brief SDI12 power related error */
    SDI12RetCode_TX_ERROR,           /**< @brief SDI12 transmission error */
    SDI12RetCode_RX_ERROR,           /**< @brief SDI12 reception error */
    SDI12RetCode_ADDRESS_IN_USE,     /**< @brief SDI12 address already in use */
    SDI12RetCode_ADDRESS_DUPLICATE,  /**< @brief SDI12 address is duplicate - mainly used for address change */
    SDI12RetCode_ADDRESS_INVALID,    /**< @brief SDI12 address invalid */
    SDI12RetCode_CRC_ERROR,          /**< @brief SDI12 command response CRC does not match */
    SDI12RetCode_INVALID,            /**< @brief SDI12 unknown / invalid behavior */
    SDI12RetCode_ERROR,              /**< @brief SDI12 general failure or error */
    SDI12RetCode_OK,                 /**< @brief SDI12 communication OK */
}SDI12RetCode;

/**
 * @brief To specify port and pin mapping for sdi12
 */
typedef struct __Sdi12PortPinMap_t
{
    GPIO_TypeDef *pGpioPort; /**< @brief gpio port */
    uint16_t      gpioPin;   /**< @brief gpio pin number */
}
Sdi12PortPinMap;

/**
 * @brief sdi12 pin configuration
 */
typedef struct __Sdi12PinConfig_t
{
    Sdi12PortPinMap sdiPwrMain;                      /**< @brief sdi12 5V power pin */
    Sdi12PortPinMap sdiTxEnb;                        /**< @brief sdi12 transmit enable pin */
    Sdi12PortPinMap sdiTx;                           /**< @brief sdi12 transmission pin */
    Sdi12PortPinMap sdiPwrSlot[MAX_HW_SLOTS_SDI12];  /**< @brief array of sdi12 slot power pins */
}
Sdi12PinConfig;

/**
 * @brief structure for sdi12 handle
 */
typedef struct __Sdi12Handle_t
{
    UART_HandleTypeDef *hUART;                  /**< @brief UART handle */

    Sdi12PinConfig      Sdi12PinCfg;            /**< @brief sdi12 bus related pin configuration */

    char                sdi12Address;           /**< @brief SDI12 sensor address */

    char                sdi12IdNewOrQuery;      /**< @brief This field is used for query and address change command only */
}
Sdi12Handle;

typedef struct __Sdi12Receive_t
{
    char   *recBuf;  /**< @brief buffer to receive data */
    uint8_t recSize; /**< @brief size of data received */
}Sdi12Receive;

/**
 * @brief Initialize SDI12 bus driver
 *
 * @param[in] reqSlotNum Requested slot support number
 *
 * @return SDI12RetCode_OK if Success
 *         SDI12RetCode_ERROR if fail
 */
SDI12RetCode sdi12Init(Sdi12Handle *sdi12SensorHandle, Sdi12Receive *sdi12recBuf);
/**
 * @brief Function queries address of sensor present on SDI12 bus
 *
 * @param[out] SdiAddrFromResp gets filled by Address received in response for query command
 *
 * @return SDI12RetCode_OK if queried address is valid
 *         SDI12RetCode_ADDRESS_INVALID if wrong address is read
 *         SDI12RetCode_ERROR if failed
 */
SDI12RetCode sdi12QuerySensorAddress(char *sdiAddrFromResp);

/**
 * @brief Function to change current address of sensor into desired new address
 *
 * @param[in] existingAddr Current address of sensor on sdi bus
 * @param[in] desiredAddr desired address to be changed to
 *
 * @return SDI12RetCode_OK if Success
 *         SDI12RetCode_ERROR on failure to change address
 *         SDI12RetCode_ADDRESS_IN_USE if address already exists
 *         SDI12RetCode_ADDRESS_INVALID if address to be changed to if invalid/out of range
 *         SDI12RetCode_ADDRESS_DUPLICATE requested address changed but is not sole sensor having this address on line
 */
SDI12RetCode sdi12ChangeAddress(char existingAddr, char desiredAddr);

/**
 * @brief Function queries address of sensor present on SDI12 bus
 *        NOTE: This function must be used when only one sensor is present on bus
 *              or only single sensor is powered on and able to communicate on bus.
 *
 * NOTE: handleSdi12.sdi12IdNewOrQuery gets updated with address received from query command
 *
 * @return SDI12RetCode_OK if queried address is valid
 *         SDI12RetCode_ADDRESS_INVALID if wrong address is read
 *         SDI12RetCode_ERROR if failed
 */
SDI12RetCode sdi12QueryAddress();

/* @[declare_sdi12_getMeasuredData] */

#endif /* SDI12_DRIVER_H */
