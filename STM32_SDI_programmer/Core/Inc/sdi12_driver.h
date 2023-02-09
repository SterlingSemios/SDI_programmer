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
 * @brief SDI12 bus status
 */
typedef enum
{
    SDI12_UNINITIALIZED = 0,  /**< @brief SDI12 bus status - bus not initialized */
    SDI12_INITIALIZED,        /**< @brief SDI12 bus status - bus initialized */
}Sdi12BusStatus;

/**
 * @brief SDI12 main power supply (5V) status
 */
typedef enum
{
    SDI12_MAINPOWER_OFF = 0,  /**< @brief SDI12 main supply Off */
    SDI12_MAINPOWER_ON,       /**< @brief SDI12 main supply On */
}Sdi12MainPwrStatus;


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
 * @brief Structure to hold SDI12 sensor identification information
 */
typedef struct __Sdi12Identification_t
{
    char sdi12Version[SDI12_VERSION_BYTES];           /**< @brief SDI12 version information */
    char sensorVendorName[SENSOR_VENDOR_NAME_BYTES];  /**< @brief name of SDI12 sensor vendor */
    char modelNumber[SENSOR_MODEL_NUM_BYTES];         /**< @brief SDI12 sensor model number */
    char sensorVersion[SENSOR_VERSION_NUM_BYTES];     /**< @brief SDI12 sensor version */
    char optionalData[INDETIFICATION_METADATA_BYTES]; /**< @brief optional data such as serial number of sensor or
                                                       *          other data that is irrelevant to driver/data logger */
    char sdi12Address;                                /**< @brief SDI12 address of the sensor */
    int  slotNum;                                     /**< @brief Corrosponding HW slot */
}
Sdi12Identification;

/**
 * @brief Structure to hold received raw measurements and its size
 */
typedef struct __Sdi12MeasDataVal_t
{
    uint8_t size;                                 /**< @brief size of data received */
    char    str_sdi12_measVal[MAX_RESPONSE_SIZE]; /**< @brief to hold received data values */
    unsigned int additionalData;                  /**< @brief whether additional data is being requested */
}
Sdi12MeasDataVal;

/**
 * @brief Structure to hold parsed response of start measurement command
 */
typedef struct __Sdi12StartMeasVal_t
{
    uint8_t numberOfMeasurements;    /**< @brief to hold count of total measurements */
    uint8_t secWaitTimeForMeasReady; /**< @brief to hold wait time (in seconds) to get measurements ready */
}
Sdi12StartMeasVal;

/**
 * @brief structure for sdi12 handle
 */
typedef struct __Sdi12Handle_t
{
    UART_HandleTypeDef *hUART;                  /**< @brief UART handle */

    Sdi12PinConfig      Sdi12PinCfg;            /**< @brief sdi12 bus related pin configuration */
    Sdi12BusStatus      Sdi12BusStat;           /**< @brief sdi12 bus status */
    Sdi12MainPwrStatus  Sdi12MainPwrStat;       /**< @brief sdi12 main supply on/off status */

    char                sdi12Address;           /**< @brief SDI12 sensor address */
    uint8_t             bitfieldModeFlags;      /**< @brief byte field for concurrent and crc bit flags */
    uint8_t             requiredSlotSupportNum; /**< @brief number indicating maximum physical slot support requested */
    uint8_t             bitmaskSdiSlotPwrStat;  /**< @brief bitwise power status of 4 slots, when 0: off, 1:on */
    uint8_t             bitmaskAddressInUse;    /**< @brief bitwise address in use status - set bit posistion indicates address used */
    uint8_t             measurementIndex;       /**< @brief extra arguments such as measurement index*/

    char                sdi12IdNewOrQuery;      /**< @brief This field is used for query and address change command only */
}
Sdi12Handle;

/**
 * @brief Initialize SDI12 bus driver
 *
 * @param[in] reqSlotNum Requested slot support number
 *
 * @return SDI12RetCode_OK if Success
 *         SDI12RetCode_ERROR if fail
 */
SDI12RetCode sdi12Init(unsigned int reqSlotNum);

/**
 * @brief Deinit SDI12 bus driver
 *
 * @param[in] pUnassignCfg pointer to SDI12 gpio port-pin mapping for NULL assignments
 *
 * @return SDI12RetCode_OK if Success
 *         SDI12RetCode_ERROR if Failure
 */
SDI12RetCode sdi12DeInit(const Sdi12PinConfig *pUnassignCfg);

/**
 * @brief Enable power to all SDI12 slots
 *
 * @param[in] slotsToEnable Bitwise flag to specify which slots (0-3) will be enabled. ex: 1111 (15) to enable all slots
 *
 * @return SDI12RetCode_OK if Success
 *         SDI12RetCode_ERROR if Failure
 */
SDI12RetCode sdi12Enable(uint8_t slotsToEnable);

/**
 * @brief Disable power to specific SDI slots
 *
 * @param[in] slotsToDisable Bitwise flag to specify which slots (0-3) will be disabled. ex: 0010 (2) to disable slot 2
 *
 * @return SDI12RetCode_OK if Success
 *         SDI12RetCode_ERROR if Failure
 */
SDI12RetCode sdi12DisableSlots(uint8_t slotsToDisable);

/**
 * @brief Disable power to all SDI12 slots and disable main power bus
 *
 * @return SDI12RetCode_OK if Success
 *         SDI12RetCode_ERROR if Failure
 */
SDI12RetCode sdi12DisableAll();

/**
 * @brief Function to check if addressed sensor is active
 *
 * @param[in] deviceAddress Address on sdi bus to which communication is desired
 *
 * @return SDI12RetCode_OK in case sensor with address is active
 *         SDI12RetCode_ADDRESS_INVALID in case provided address is invalid/out-of-range
 *         SDI12RetCode_ERROR in case sensor is not active
 */
SDI12RetCode sdi12IsAckActive(char deviceAddress);

/**
 * @brief Function queries address of sensor present on SDI12 bus
 *        NOTE: This function must be used when only one sensor is present on bus
 *              or only single sensor is powered on and able to communicate on bus.
 *
 * @param[in] slotNumber HW slot number in which sensor is connected
 * @param[out] SdiAddrFromResp gets filled by Address received in response for query command
 *
 * @return SDI12RetCode_OK if queried address is valid
 *         SDI12RetCode_ADDRESS_INVALID if wrong address is read
 *         SDI12RetCode_ERROR if failed
 */
int sdi12QuerySensorAddress(uint8_t slotNumber, char *sdiAddrFromResp);

/**
 * @brief Function to get identification information from sensor
 *
 * @param[in] deviceAddr address on SDI bus to which communication is intended
 * @param[out] pointer to structure where identification information gets filled in
 * @return SDI12RetCode_OK if Success
 *         SDI12RetCode_ERROR if Fail
 */
SDI12RetCode sdi12Identification(char deviceAddr, Sdi12Identification *identificationInfo);

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
 * @brief Function to issue start measurement command to sensor and processes response
 *
 * @param[in] measIndex Index of measurement - 0: no index, 1-9 - index number
 * @param[in] wantConcurrent boolean to indicate mode for measurement
 * @param[in] wantCRC boolean type to indicate CRC requested or not
 * @param[in] sdiAddr sdi12 address for communication
 * @param[out] respStartMeas: pointer to struct holding response of start measurement command
 *
 * @return SDI12RetCode_OK on Success
 *         SDI12RetCode_ERROR on Failure
 *         SDI12RetCode_RX_ERROR in case of RX error
 */
/* @[declare_sdi12_startMeasurement] */
SDI12RetCode sdi12StartMeasurement(uint8_t measIndex,
                                    bool wantConcurrent, bool wantCRC,
                                    char sdiAddr,
                                    Sdi12StartMeasVal *respStartMeas);

/* @[declare_sdi12_startMeasurement] */

/**
 * @brief Function to read measured data values
 *
 * @param[in] measIndex Index of additional measurement data to be read
 * @param[in] sdiAddr intended address for communication
 * @param[out] measuredDataResp fills with the response values
 *
 * @return SDI12RetCode_OK if success
 *         SDI12RetCode_ERROR if failed
 *         SDI12RetCode_RX_ERROR in case of RX error
 */
/* @[declare_sdi12_getMeasuredData] */
SDI12RetCode sdi12GetMeasuredData(uint8_t measIndex,
                                   char sdiAddr,
                                   Sdi12MeasDataVal *measuredDataResp);

/* @[declare_sdi12_getMeasuredData] */

#endif /* SDI12_DRIVER_H */
