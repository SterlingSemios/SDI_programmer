#include "sdi12_driver.h"
#include "sdi12Bus_communication.h"
#include "sdi_logging.h"
#include "led_driver.h"

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

int* buttonFlag;

SDI12RetCode sdi12Init(Sdi12Handle *sdi12SensorHandle, Sdi12Receive *sdi12recBuf, int *flag)
{
    NULL_PTR_CHECK(sdi12SensorHandle, SDI12RetCode_INVALID);
    NULL_PTR_CHECK(sdi12recBuf, SDI12RetCode_INVALID);

    sensorHandle = sdi12SensorHandle;
    Sdi12Response = sdi12recBuf;
    buttonFlag = flag;

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

        }
        else
        {
            retStat = SDI12RetCode_ADDRESS_INVALID;
        }
    }

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

void triggerAddressChange(addrChangeType changeAddress)
{
    int maxAddress = 3;
    char addressArr[]=  {'0', '1', '2', '3'};
    int newAddress = 0;

    for(int address = 0; address <= maxAddress; address++)
    {
        if(strncmp(&sensorHandle->sdi12IdNewOrQuery, &addressArr[address], 1) == 0)
        {
            if(changeAddress == UP)
            {
                if(address != 3)
                {
                    newAddress = address + 1;
                }
                else
                {
                    newAddress = 0;
                }
            }
            else if(changeAddress == DOWN)
            {
                if(address != 0)
                {
                    newAddress = address - 1;
                }
                else
                {
                    newAddress = 3;
                }
            }
        }
    }

    *buttonFlag = newAddress;
}
