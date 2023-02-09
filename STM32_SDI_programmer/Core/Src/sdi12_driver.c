#include "sdi12_driver.h"
#include "sdi12Bus_communication.h"
#include "sdi_logging.h"

//Delay defines
#define STD_WAIT_SENRESP_MS            (150)  /**< @brief SDI12 wait time for response as per standard*/
#define SDI_STANDARD_WAIT_MS           (750)  /**< @brief SDI12 standard wait time for Address change and Identification command response as per standard */

//Address related defines
#define ADDRESS_CHAR_IN_RESP           0   /**< @brief  character in response */
#define ALLOWED_ADDRESS_MIN            '0' /**< @brief lower limit of valid address range */
#define ALLOWED_ADDRESS_MAX            '3' /**< @brief upper limit of valid address range */
#define MIN_RESP_LEN_IDENTITY          20

extern UART_HandleTypeDef huart1;
char address_buf;

Sdi12Receive *Sdi12Response;
Sdi12Handle *sensorHandle = {0};          /*SDI12 sensor handle*/


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
//static SDI12RetCode sdi12QueryAddress();


SDI12RetCode sdi12Init(Sdi12Handle *sdi12SensorHandle, Sdi12Receive *sdi12recBuf)
{
    NULL_PTR_CHECK(sdi12SensorHandle, SDI12RetCode_INVALID);
    NULL_PTR_CHECK(sdi12recBuf, SDI12RetCode_INVALID);

    sensorHandle = sdi12SensorHandle;
    Sdi12Response = sdi12recBuf;

    return SDI12RetCode_OK;
}

SDI12RetCode sdi12QueryAddress()
{
    //?! : a<CR><LF>

    SDI12RetCode retStat = 0;

    retStat = sdi12_BusCommunication(sensorHandle, Sdi12Response->recBuf,
                                     &Sdi12Response->recSize, STD_WAIT_SENRESP_MS,
                                     SDI12_QUERY_ADDR);

    if (retStat == SDI12RetCode_OK)
    {
        if (Sdi12Response->recBuf[ADDRESS_CHAR_IN_RESP] >= ALLOWED_ADDRESS_MIN &&
            Sdi12Response->recBuf[ADDRESS_CHAR_IN_RESP] <= ALLOWED_ADDRESS_MAX)
        {
            sensorHandle->sdi12IdNewOrQuery = Sdi12Response->recBuf[ADDRESS_CHAR_IN_RESP];
            address_buf = Sdi12Response->recBuf[ADDRESS_CHAR_IN_RESP];

            char ch = Sdi12Response->recBuf[ADDRESS_CHAR_IN_RESP] - '0'; //Limitation: for now applicable only to range 0-9
        }
        else
        {
            retStat = SDI12RetCode_ADDRESS_INVALID;
        }
    }

    //xSemaphoreGive(sdi12CommMutex);

    return retStat;
}

SDI12RetCode sdi12ChangeAddress(char existingAddr, char desiredAddr)
{
    //aAb! : b<CR><LF>


    SDI12RetCode retStat;


    uint8_t addrDuplicationExists = 0;

    // if desired address is out of acceptable range
    if (desiredAddr > ALLOWED_ADDRESS_MAX)
    {
        return (SDI12RetCode_ADDRESS_INVALID);
    }

    char ch = desiredAddr - '0';

    //fill up required fields for communication
    sensorHandle->sdi12IdNewOrQuery = desiredAddr;
    sensorHandle->sdi12Address      = existingAddr;

    retStat = sdi12_BusCommunication(sensorHandle, Sdi12Response->recBuf,
                                     &Sdi12Response->recSize, SDI_STANDARD_WAIT_MS,
                                     SDI12_CHANGE_ADDR);

    if (retStat == SDI12RetCode_OK)
    {
        //check if address field of response is same as requested desired address
        if (Sdi12Response->recBuf[ADDRESS_CHAR_IN_RESP] ==
            sensorHandle->sdi12IdNewOrQuery)
        {
            sensorHandle->sdi12Address         =
                Sdi12Response->recBuf[ADDRESS_CHAR_IN_RESP];
            retStat =
                (addrDuplicationExists ==
                 1) ? SDI12RetCode_ADDRESS_DUPLICATE : SDI12RetCode_OK;
        }
        else
        {
            retStat = SDI12RetCode_ERROR;
        }
    }

    return retStat;
}
