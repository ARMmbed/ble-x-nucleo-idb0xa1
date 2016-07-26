/* mbed Microcontroller Library
* Copyright (c) 2006-2013 ARM Limited
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*/


/**
  ******************************************************************************
  * @file    BlueNRGGap.cpp
  * @author  STMicroelectronics
  * @brief   Implementation of BLE_API Gap Class
  ******************************************************************************
  * @copy
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2013 STMicroelectronics</center></h2>
  */

/** @defgroup BlueNRGGap
 *  @brief BlueNRG BLE_API GAP Adaptation
 *  @{
 */

#include "BlueNRGDevice.h"
#ifdef YOTTA_CFG_MBED_OS
    #include "mbed-drivers/mbed.h"
#else
    #include "mbed.h"
#endif 
#include "Payload.h"
#include "Utils.h"
#include "debug.h"

/*
 * Utility to process GAP specific events (e.g., Advertising timeout)
 */
void BlueNRGGap::Process(void)
{
    if(AdvToFlag) {
        stopAdvertising();
    }

}

/**************************************************************************/
/*!
    @brief  Sets the advertising parameters and payload for the device.
            Note: Some data types give error when their adv data is updated using aci_gap_update_adv_data() API

    @params[in] advData
                The primary advertising data payload
    @params[in] scanResponse
                The optional Scan Response payload if the advertising
                type is set to \ref GapAdvertisingParams::ADV_SCANNABLE_UNDIRECTED
                in \ref GapAdveritinngParams

    @returns    \ref ble_error_t

    @retval     BLE_ERROR_NONE
                Everything executed properly

    @retval     BLE_ERROR_BUFFER_OVERFLOW
                The proposed action would cause a buffer overflow.  All
                advertising payloads must be <= 31 bytes, for example.

    @retval     BLE_ERROR_NOT_IMPLEMENTED
                A feature was requested that is not yet supported in the
                nRF51 firmware or hardware.

    @retval     BLE_ERROR_PARAM_OUT_OF_RANGE
                One of the proposed values is outside the valid range.

    @section EXAMPLE

    @code

    @endcode
*/
/**************************************************************************/
ble_error_t BlueNRGGap::setAdvertisingData(const GapAdvertisingData &advData, const GapAdvertisingData &scanResponse)
{
    PRINTF("BlueNRGGap::setAdvertisingData\n\r");
    /* Make sure we don't exceed the advertising payload length */
    if (advData.getPayloadLen() > GAP_ADVERTISING_DATA_MAX_PAYLOAD) {
        PRINTF("Exceeded the advertising payload length\n\r");
        return BLE_ERROR_BUFFER_OVERFLOW;
    }

    /* Make sure we have a payload! */
    if (advData.getPayloadLen() != 0) {
        PayloadPtr loadPtr(advData.getPayload(), advData.getPayloadLen());

        /* Align the GAP Service Appearance Char value coherently
           This setting is duplicate (see below GapAdvertisingData::APPEARANCE)
           since BLE API has an overloaded function for appearance
        */
        STORE_LE_16(deviceAppearance, advData.getAppearance());
        setAppearance((GapAdvertisingData::Appearance)(deviceAppearance[1]<<8|deviceAppearance[0]));


        for(uint8_t index=0; index<loadPtr.getPayloadUnitCount(); index++) {
            loadPtr.getUnitAtIndex(index);

            PRINTF("adData[%d].length=%d\n\r", index,(uint8_t)(*loadPtr.getUnitAtIndex(index).getLenPtr()));
            PRINTF("adData[%d].AdType=0x%x\n\r", index,(uint8_t)(*loadPtr.getUnitAtIndex(index).getAdTypePtr()));

            switch(*loadPtr.getUnitAtIndex(index).getAdTypePtr()) {
            /**< TX Power Level (in dBm) */
            case GapAdvertisingData::TX_POWER_LEVEL:
                {
                PRINTF("Advertising type: TX_POWER_LEVEL\n\r");
                int8_t enHighPower = 0;
                int8_t paLevel = 0;

                int8_t dbm = *loadPtr.getUnitAtIndex(index).getDataPtr();
                tBleStatus ret = getHighPowerAndPALevelValue(dbm, enHighPower, paLevel);
#ifdef DEBUG
                PRINTF("dbm=%d, ret=%d\n\r", dbm, ret);
                PRINTF("enHighPower=%d, paLevel=%d\n\r", enHighPower, paLevel);
#endif
                if(ret == BLE_STATUS_SUCCESS) {
                  aci_hal_set_tx_power_level(enHighPower, paLevel);
                }
                break;
                }
            /**< Appearance */
            case GapAdvertisingData::APPEARANCE:
                {
                PRINTF("Advertising type: APPEARANCE\n\r");

                GapAdvertisingData::Appearance appearanceP;
                memcpy(deviceAppearance, loadPtr.getUnitAtIndex(index).getDataPtr(), 2);

                PRINTF("input: deviceAppearance= 0x%x 0x%x\n\r", deviceAppearance[1], deviceAppearance[0]);

                appearanceP = (GapAdvertisingData::Appearance)(deviceAppearance[1]<<8|deviceAppearance[0]);
                /* Align the GAP Service Appearance Char value coherently */
                setAppearance(appearanceP);
                break;
                }

            } // end switch

        } //end for

    }

    _advData = advData;
    _scanResponse = scanResponse;

    return BLE_ERROR_NONE;
}

/*
 * Utility to set ADV timeout flag
 */
void BlueNRGGap::setAdvToFlag(void) {
    AdvToFlag = true;
}

/*
 * ADV timeout callback
 */
#ifdef AST_FOR_MBED_OS
static void advTimeoutCB(void)
{
    Gap::GapState_t state;

    state = BlueNRGGap::getInstance().getState();
    if (state.advertising == 1) {

        BlueNRGGap::getInstance().stopAdvertising();

    }
}
#else
static void advTimeoutCB(void)
{
    Gap::GapState_t state;

    state = BlueNRGGap::getInstance().getState();
    if (state.advertising == 1) {

        BlueNRGGap::getInstance().setAdvToFlag();

        Timeout t = BlueNRGGap::getInstance().getAdvTimeout();
        t.detach(); /* disable the callback from the timeout */

    }
}
#endif /* AST_FOR_MBED_OS */

/**************************************************************************/
/*!
    @brief  Starts the BLE HW, initialising any services that were
            added before this function was called.

    @param[in]  params
                Basic advertising details, including the advertising
                delay, timeout and how the device should be advertised

    @note   All services must be added before calling this function!

    @returns    ble_error_t

    @retval     BLE_ERROR_NONE
                Everything executed properly

    @section EXAMPLE

    @code

    @endcode
*/
/**************************************************************************/

ble_error_t BlueNRGGap::startAdvertising(const GapAdvertisingParams &params)
{
    tBleStatus ret;
    int err;

    /* Make sure we support the advertising type */
    if (params.getAdvertisingType() == GapAdvertisingParams::ADV_CONNECTABLE_DIRECTED) {
        /* ToDo: This requires a proper security implementation, etc. */
        return BLE_ERROR_NOT_IMPLEMENTED;
    }

    /* Check interval range */
    if (params.getAdvertisingType() == GapAdvertisingParams::ADV_NON_CONNECTABLE_UNDIRECTED) {
        /* Min delay is slightly longer for unconnectable devices */
        if ((params.getIntervalInADVUnits() < GapAdvertisingParams::GAP_ADV_PARAMS_INTERVAL_MIN_NONCON) ||
                (params.getIntervalInADVUnits() > GapAdvertisingParams::GAP_ADV_PARAMS_INTERVAL_MAX)) {
            return BLE_ERROR_PARAM_OUT_OF_RANGE;
        }
    } else {
        if ((params.getIntervalInADVUnits() < GapAdvertisingParams::GAP_ADV_PARAMS_INTERVAL_MIN) ||
                (params.getIntervalInADVUnits() > GapAdvertisingParams::GAP_ADV_PARAMS_INTERVAL_MAX)) {
            return BLE_ERROR_PARAM_OUT_OF_RANGE;
        }
    }

    /* Check timeout is zero for Connectable Directed */
    if ((params.getAdvertisingType() == GapAdvertisingParams::ADV_CONNECTABLE_DIRECTED) && (params.getTimeout() != 0)) {
        /* Timeout must be 0 with this type, although we'll never get here */
        /* since this isn't implemented yet anyway */
        return BLE_ERROR_PARAM_OUT_OF_RANGE;
    }

    /* Check timeout for other advertising types */
    if ((params.getAdvertisingType() != GapAdvertisingParams::ADV_CONNECTABLE_DIRECTED) &&
            (params.getTimeout() > GapAdvertisingParams::GAP_ADV_PARAMS_TIMEOUT_MAX)) {
        return BLE_ERROR_PARAM_OUT_OF_RANGE;
    }

    /*
     * Advertising filter policy setting
     * FIXME: the Security Manager should be implemented
     */
    AdvertisingPolicyMode_t mode = getAdvertisingPolicyMode();
    if(mode != ADV_POLICY_IGNORE_WHITELIST) {
        ret = aci_gap_configure_whitelist();
        if(ret != BLE_STATUS_SUCCESS) {
          PRINTF("aci_gap_configure_whitelist ret=0x%x\n\r", ret);
          return BLE_ERROR_OPERATION_NOT_PERMITTED;
        }
    }

    uint8_t advFilterPolicy = NO_WHITE_LIST_USE;
    switch(mode) {
        case ADV_POLICY_FILTER_SCAN_REQS:
            advFilterPolicy = WHITE_LIST_FOR_ONLY_SCAN;
            break;
        case ADV_POLICY_FILTER_CONN_REQS:
            advFilterPolicy = WHITE_LIST_FOR_ONLY_CONN;
            break;
        case ADV_POLICY_FILTER_ALL_REQS:
            advFilterPolicy = WHITE_LIST_FOR_ALL;
            break;
        default:
            advFilterPolicy = NO_WHITE_LIST_USE;
            break;
    }

    /* Check the ADV type before setting scan response data */
    if (params.getAdvertisingType() == GapAdvertisingParams::ADV_CONNECTABLE_UNDIRECTED ||
        params.getAdvertisingType() == GapAdvertisingParams::ADV_SCANNABLE_UNDIRECTED) {

        /* set scan response data */
        PRINTF(" setting scan response data (_scanResponseLen=%u)\n", _scanResponse.getPayloadLen());
        ret = hci_le_set_scan_resp_data(_scanResponse.getPayloadLen(), _scanResponse.getPayload());

        if(BLE_STATUS_SUCCESS!=ret) {
            PRINTF(" error while setting scan response data (ret=0x%x)\n", ret);
            switch (ret) {
              case BLE_STATUS_TIMEOUT:
                return BLE_STACK_BUSY;
              default:
                return BLE_ERROR_UNSPECIFIED;
            }
        }
    } else {
        hci_le_set_scan_resp_data(0, NULL);
    }

    setAdvParameters();
    PRINTF("advInterval=%d advType=%d\n\r", advInterval, params.getAdvertisingType());

    err = hci_le_set_advertising_data(_advData.getPayloadLen(), _advData.getPayload());

    if (err) {
        PRINTF("error while setting the payload\r\n");
        return BLE_ERROR_UNSPECIFIED;
    }

    tBDAddr dummy_addr = { 0 };
    uint16_t advIntervalMin = advInterval == GapAdvertisingParams::GAP_ADV_PARAMS_INTERVAL_MAX ? advInterval - 1 : advInterval;
    uint16_t advIntervalMax = advIntervalMin + 1;

    err = hci_le_set_advertising_parameters(
        advIntervalMin,
        advIntervalMax,
        params.getAdvertisingType(),
    	addr_type,
        0x00,
        dummy_addr,
        /* all channels */ 7,
    	advFilterPolicy
    );

    if (err) {
        PRINTF("impossible to set advertising parameters\n\r");
        PRINTF("advInterval min: %u, advInterval max: %u\n\r", advInterval, advInterval + 1);
        PRINTF("advType: %u, advFilterPolicy: %u\n\r", params.getAdvertisingType(), advFilterPolicy);
        return BLE_ERROR_INVALID_PARAM;
    }

    err = hci_le_set_advertise_enable(0x01);
    if (err) {
        PRINTF("impossible to start advertising\n\r");
        return BLE_ERROR_UNSPECIFIED;
    }

    state.advertising = 1;

    AdvToFlag = false;
    if(params.getTimeout() != 0) {
        PRINTF("!!! attaching to!!!\n");
#ifdef AST_FOR_MBED_OS
        minar::Scheduler::postCallback(advTimeoutCB).delay(minar::milliseconds(params.getTimeout() * 1000));
#else
        advTimeout.attach(advTimeoutCB, params.getTimeout() * 1000);
#endif
    }

    return BLE_ERROR_NONE;

}


/**************************************************************************/
/*!
    @brief  Stops the BLE HW and disconnects from any devices

    @returns    ble_error_t

    @retval     BLE_ERROR_NONE
                Everything executed properly

    @section EXAMPLE

    @code

    @endcode
*/
/**************************************************************************/
ble_error_t BlueNRGGap::stopAdvertising(void)
{

    if(state.advertising == 1) {

        int err = hci_le_set_advertise_enable(0);
        if (err) {
            return BLE_ERROR_OPERATION_NOT_PERMITTED;
        }

        PRINTF("Advertisement stopped!!\n\r") ;
        //Set GapState_t::advertising state
        state.advertising = 0;
    }

    return BLE_ERROR_NONE;
}

/**************************************************************************/
/*!
    @brief  Disconnects if we are connected to a central device

    @param[in]  reason
                Disconnection Reason

    @returns    ble_error_t

    @retval     BLE_ERROR_NONE
                Everything executed properly

    @section EXAMPLE

    @code

    @endcode
*/
/**************************************************************************/
ble_error_t BlueNRGGap::disconnect(Handle_t connectionHandle, Gap::DisconnectionReason_t reason)
{
    tBleStatus ret;

    ret = aci_gap_terminate(connectionHandle, reason);

    if (BLE_STATUS_SUCCESS != ret){
        PRINTF("Error in GAP termination (ret=0x%x)!!\n\r", ret) ;
        switch (ret) {
          case ERR_COMMAND_DISALLOWED:
            return BLE_ERROR_OPERATION_NOT_PERMITTED;
          case BLE_STATUS_TIMEOUT:
            return BLE_STACK_BUSY;
          default:
            return BLE_ERROR_UNSPECIFIED;
        }
    }

    return BLE_ERROR_NONE;
}

/**************************************************************************/
/*!
    @brief  Disconnects if we are connected to a central device

    @param[in]  reason
                Disconnection Reason

    @returns    ble_error_t

    @retval     BLE_ERROR_NONE
                Everything executed properly

    @section EXAMPLE

    @code

    @endcode
*/
/**************************************************************************/
ble_error_t BlueNRGGap::disconnect(Gap::DisconnectionReason_t reason)
{
    return disconnect(m_connectionHandle, reason);
}

/**************************************************************************/
/*!
    @brief  Sets the 16-bit connection handle

    @param[in]  conn_handle
                Connection Handle which is set in the Gap Instance

    @returns    void
*/
/**************************************************************************/
void BlueNRGGap::setConnectionHandle(uint16_t conn_handle)
{
    m_connectionHandle = conn_handle;
}

/**************************************************************************/
/*!
    @brief  Gets the 16-bit connection handle

    @param[in]  void

    @returns    uint16_t
                Connection Handle of the Gap Instance
*/
/**************************************************************************/
uint16_t BlueNRGGap::getConnectionHandle(void)
{
    return m_connectionHandle;
}

/**************************************************************************/
/*!
    @brief      Sets the BLE device address. SetAddress will reset the BLE
                device and re-initialize BTLE. Will not start advertising.

    @param[in]  type
                Type of Address

    @param[in]  address[6]
                Value of the Address to be set

    @returns    ble_error_t

    @section EXAMPLE

    @code

    @endcode
*/
/**************************************************************************/
ble_error_t BlueNRGGap::setAddress(AddressType_t type, const BLEProtocol::AddressBytes_t address)
{
    if (type > BLEProtocol::AddressType::RANDOM_PRIVATE_NON_RESOLVABLE) {
        return BLE_ERROR_PARAM_OUT_OF_RANGE;
    }

    if(type == BLEProtocol::AddressType::PUBLIC){
        tBleStatus ret = aci_hal_write_config_data(
            CONFIG_DATA_PUBADDR_OFFSET,
            CONFIG_DATA_PUBADDR_LEN,
            address
        );
        if(ret != BLE_STATUS_SUCCESS) {
            return BLE_ERROR_OPERATION_NOT_PERMITTED;
        }
    } else if  (type == BLEProtocol::AddressType::RANDOM_STATIC) {
        // ensure that the random static address is well formed
        if ((address[5] & 0xC0) != 0xC0) {
            return BLE_ERROR_PARAM_OUT_OF_RANGE;
        }

        // thanks to const correctness of the API ...
        tBDAddr random_address = { 0 };
        memcpy(random_address, address, sizeof(random_address));
        int err = hci_le_set_random_address(random_address);
        if (err) {
            return BLE_ERROR_OPERATION_NOT_PERMITTED;
        }

        // It is not possible to get the bluetooth address when it is set
        // store it locally in class data member
        memcpy(bdaddr, address, sizeof(bdaddr));
    } else {
        // FIXME random addresses are not supported yet
        // BLEProtocol::AddressType::RANDOM_PRIVATE_NON_RESOLVABLE
        // BLEProtocol::AddressType::RANDOM_PRIVATE_RESOLVABLE
        return BLE_ERROR_NOT_IMPLEMENTED;
    }

    // if we're here then the address was correctly set
    // commit it inside the addr_type
    addr_type = type;
    isSetAddress = true;
    return BLE_ERROR_NONE;
}

/**************************************************************************/
/*!
    @brief      Returns boolean if the address of the device has been set
                or not

    @returns    bool

    @section EXAMPLE

    @code

    @endcode
*/
/**************************************************************************/
bool BlueNRGGap::getIsSetAddress()
{
    return isSetAddress;
}

/**************************************************************************/
/*!
    @brief      Returns the address of the device if set

    @returns    Pointer to the address if Address is set else NULL

    @section EXAMPLE

    @code

    @endcode
*/
/**************************************************************************/
ble_error_t BlueNRGGap::getAddress(BLEProtocol::AddressType_t *typeP, BLEProtocol::AddressBytes_t address)
{
    uint8_t bdaddr[BDADDR_SIZE];
    uint8_t data_len_out;

    // precondition, check that pointers in input are valid
    if (typeP == NULL || address == NULL) {
        return BLE_ERROR_INVALID_PARAM;
    }

    if (addr_type == BLEProtocol::AddressType::PUBLIC) {
        tBleStatus ret = aci_hal_read_config_data(CONFIG_DATA_PUBADDR_OFFSET, BDADDR_SIZE, &data_len_out, bdaddr);
        if(ret != BLE_STATUS_SUCCESS || data_len_out != BDADDR_SIZE) {
            return BLE_ERROR_UNSPECIFIED;
        }
    } else if (addr_type == BLEProtocol::AddressType::RANDOM_STATIC) {
        // FIXME hci_read_bd_addr and
        // aci_hal_read_config_data CONFIG_DATA_RANDOM_ADDRESS_IDB05A1
        // does not work, use the address stored in class data member
        memcpy(bdaddr, this->bdaddr, sizeof(bdaddr));
    } else {
        // FIXME: should be implemented with privacy features
        // BLEProtocol::AddressType::RANDOM_PRIVATE_NON_RESOLVABLE
        // BLEProtocol::AddressType::RANDOM_PRIVATE_RESOLVABLE
        return BLE_ERROR_NOT_IMPLEMENTED;
    }

    *typeP = addr_type;
    memcpy(address, bdaddr, BDADDR_SIZE);

    return BLE_ERROR_NONE;
}

/**************************************************************************/
/*!
    @brief      obtains preferred connection params

    @returns    ble_error_t

    @section EXAMPLE

    @code

    @endcode
*/
/**************************************************************************/
ble_error_t BlueNRGGap::getPreferredConnectionParams(ConnectionParams_t *params)
{
    static const size_t parameter_size = 2;

    if (!g_preferred_connection_parameters_char_handle) {
        return BLE_ERROR_OPERATION_NOT_PERMITTED;
    }

    // Peripheral preferred connection parameters are an array of 4 uint16_t
    uint8_t parameters_packed[parameter_size * 4];
    uint16_t bytes_read = 0;

    tBleStatus err = aci_gatt_read_handle_value(
        g_preferred_connection_parameters_char_handle + BlueNRGGattServer::CHAR_VALUE_HANDLE,
        sizeof(parameters_packed),
        &bytes_read,
        parameters_packed
    );

    PRINTF("getPreferredConnectionParams err=0x%02x (bytes_read=%u)\n\r", err, bytes_read);

    // check that the read succeed and the result have the expected length
    if (err || bytes_read != sizeof(parameters_packed)) {
        return BLE_ERROR_UNSPECIFIED;
    }

    // memcpy field by field
    memcpy(&params->minConnectionInterval, parameters_packed, parameter_size);
    memcpy(&params->maxConnectionInterval, &parameters_packed[parameter_size], parameter_size);
    memcpy(&params->slaveLatency, &parameters_packed[2 * parameter_size], parameter_size);
    memcpy(&params->connectionSupervisionTimeout, &parameters_packed[3 * parameter_size], parameter_size);

    return BLE_ERROR_NONE;
}


/**************************************************************************/
/*!
    @brief      sets preferred connection params

    @returns    ble_error_t

    @section EXAMPLE

    @code

    @endcode
*/
/**************************************************************************/
ble_error_t BlueNRGGap::setPreferredConnectionParams(const ConnectionParams_t *params)
{
    static const size_t parameter_size = 2;
    uint8_t parameters_packed[parameter_size * 4];

    // ensure that parameters are correct
    // see BLUETOOTH SPECIFICATION Version 4.2 [Vol 3, Part C]
    // section 12.3 PERIPHERAL PREFERRED CONNECTION PARAMETERS CHARACTERISTIC
    if (((0x0006 > params->minConnectionInterval) || (params->minConnectionInterval > 0x0C80)) &&
        params->minConnectionInterval != 0xFFFF) {
        return BLE_ERROR_PARAM_OUT_OF_RANGE;
    }

    if (((params->minConnectionInterval > params->maxConnectionInterval) || (params->maxConnectionInterval > 0x0C80)) &&
        params->maxConnectionInterval != 0xFFFF) {
        return BLE_ERROR_PARAM_OUT_OF_RANGE;
    }

    if (params->slaveLatency > 0x01F3) {
        return BLE_ERROR_PARAM_OUT_OF_RANGE;
    }

    if (((0x000A > params->connectionSupervisionTimeout) || (params->connectionSupervisionTimeout > 0x0C80)) &&
        params->connectionSupervisionTimeout != 0xFFFF) {
        return BLE_ERROR_PARAM_OUT_OF_RANGE;
    }

    // copy the parameters inside the byte array
    memcpy(parameters_packed, &params->minConnectionInterval, parameter_size);
    memcpy(&parameters_packed[parameter_size], &params->maxConnectionInterval, parameter_size);
    memcpy(&parameters_packed[2 * parameter_size], &params->slaveLatency, parameter_size);
    memcpy(&parameters_packed[3 * parameter_size], &params->connectionSupervisionTimeout, parameter_size);

    tBleStatus err = aci_gatt_update_char_value(
        g_gap_service_handle,
        g_preferred_connection_parameters_char_handle,
        /* offset */ 0,
        sizeof(parameters_packed),
        parameters_packed
    );

    if (err) {
        PRINTF("setPreferredConnectionParams failed (err=0x%x)!!\n\r", err) ;
        switch (err) {
          case BLE_STATUS_INVALID_HANDLE:
          case BLE_STATUS_INVALID_PARAMETER:
            return BLE_ERROR_INVALID_PARAM;
          case BLE_STATUS_INSUFFICIENT_RESOURCES:
            return BLE_ERROR_NO_MEM;
          case BLE_STATUS_TIMEOUT:
            return BLE_STACK_BUSY;
          default:
            return BLE_ERROR_UNSPECIFIED;
        }
    }

    return BLE_ERROR_NONE;
}

/**************************************************************************/
/*!
    @brief      updates preferred connection params

    @returns    ble_error_t

    @section EXAMPLE

    @code

    @endcode
*/
/**************************************************************************/
ble_error_t BlueNRGGap::updateConnectionParams(Handle_t handle, const ConnectionParams_t *params)
{
    tBleStatus ret = BLE_STATUS_SUCCESS;

    if(gapRole == Gap::CENTRAL) {
        ret = aci_gap_start_connection_update(handle,
                                              params->minConnectionInterval,
                                              params->maxConnectionInterval,
                                              params->slaveLatency,
                                              params->connectionSupervisionTimeout,
                                              CONN_L1, CONN_L2);
    } else {
        ret = aci_l2cap_connection_parameter_update_request(handle,
                                                            params->minConnectionInterval,
                                                            params->maxConnectionInterval,
                                                            params->slaveLatency,
                                                            params->connectionSupervisionTimeout);
    }

    if (BLE_STATUS_SUCCESS != ret){
        PRINTF("updateConnectionParams failed (ret=0x%x)!!\n\r", ret) ;
        switch (ret) {
          case ERR_INVALID_HCI_CMD_PARAMS:
          case BLE_STATUS_INVALID_PARAMETER:
            return BLE_ERROR_INVALID_PARAM;
          case ERR_COMMAND_DISALLOWED:
          case BLE_STATUS_NOT_ALLOWED:
            return BLE_ERROR_OPERATION_NOT_PERMITTED;
          default:
            return BLE_ERROR_UNSPECIFIED;
        }
    }

    return BLE_ERROR_NONE;
}

/**************************************************************************/
/*!
    @brief  Sets the Device Name Characteristic

    @param[in]  deviceName
                pointer to device name to be set

    @returns    ble_error_t

    @retval     BLE_ERROR_NONE
                Everything executed properly

    @section EXAMPLE

    @code

    @endcode
*/
/**************************************************************************/
ble_error_t BlueNRGGap::setDeviceName(const uint8_t *deviceName)
{
    tBleStatus ret;
    uint8_t nameLen = 0;

    nameLen = strlen((const char*)deviceName);
    PRINTF("DeviceName Size=%d\n\r", nameLen);

    ret = aci_gatt_update_char_value(g_gap_service_handle,
                                     g_device_name_char_handle,
                                     0,
                                     nameLen,
                                     deviceName);

    if (BLE_STATUS_SUCCESS != ret){
        PRINTF("device set name failed (ret=0x%x)!!\n\r", ret) ;
        switch (ret) {
          case BLE_STATUS_INVALID_HANDLE:
          case BLE_STATUS_INVALID_PARAMETER:
            return BLE_ERROR_INVALID_PARAM;
          case BLE_STATUS_INSUFFICIENT_RESOURCES:
            return BLE_ERROR_NO_MEM;
          case BLE_STATUS_TIMEOUT:
            return BLE_STACK_BUSY;
          default:
            return BLE_ERROR_UNSPECIFIED;
        }
    }

    return BLE_ERROR_NONE;
}

/**************************************************************************/
/*!
    @brief  Gets the Device Name Characteristic

    @param[in]  deviceName
                pointer to device name

    @param[in]  lengthP
                pointer to device name length

    @returns    ble_error_t

    @retval     BLE_ERROR_NONE
                Everything executed properly

    @section EXAMPLE

    @code

    @endcode
*/
/**************************************************************************/
ble_error_t BlueNRGGap::getDeviceName(uint8_t *deviceName, unsigned *lengthP)
{
    tBleStatus ret;

    ret = aci_gatt_read_handle_value(g_device_name_char_handle+BlueNRGGattServer::CHAR_VALUE_HANDLE,
                                     *lengthP,
                                     (uint16_t *)lengthP,
                                     deviceName);
    PRINTF("getDeviceName ret=0x%02x (lengthP=%d)\n\r", ret, *lengthP);
    if (ret == BLE_STATUS_SUCCESS) {
        return BLE_ERROR_NONE;
    } else {
        return BLE_ERROR_PARAM_OUT_OF_RANGE;
    }
}

/**************************************************************************/
/*!
    @brief  Sets the Device Appearance Characteristic

    @param[in]  appearance
                device appearance

    @returns    ble_error_t

    @retval     BLE_ERROR_NONE
                Everything executed properly

    @section EXAMPLE

    @code

    @endcode
*/
/**************************************************************************/
ble_error_t BlueNRGGap::setAppearance(GapAdvertisingData::Appearance appearance)
{
    tBleStatus ret;
    uint8_t deviceAppearance[2];

    STORE_LE_16(deviceAppearance, appearance);
    PRINTF("setAppearance= 0x%x 0x%x\n\r", deviceAppearance[1], deviceAppearance[0]);

    ret = aci_gatt_update_char_value(g_gap_service_handle,
                                     g_appearance_char_handle,
                                     0, 2, (uint8_t *)deviceAppearance);
    if (BLE_STATUS_SUCCESS == ret){
        return BLE_ERROR_NONE;
    }

    PRINTF("setAppearance failed (ret=0x%x)!!\n\r", ret);
    switch (ret) {
      case BLE_STATUS_INVALID_HANDLE:
      case BLE_STATUS_INVALID_PARAMETER:
        return BLE_ERROR_INVALID_PARAM;
      case BLE_STATUS_INSUFFICIENT_RESOURCES:
        return BLE_ERROR_NO_MEM;
      case BLE_STATUS_TIMEOUT:
        return BLE_STACK_BUSY;
      default:
        return BLE_ERROR_UNSPECIFIED;
    }
}

/**************************************************************************/
/*!
    @brief  Gets the Device Appearance Characteristic

    @param[in]  appearance
                pointer to device appearance value

    @returns    ble_error_t

    @retval     BLE_ERROR_NONE
                Everything executed properly

    @section EXAMPLE

    @code

    @endcode
*/
/**************************************************************************/
ble_error_t BlueNRGGap::getAppearance(GapAdvertisingData::Appearance *appearanceP)
{
    tBleStatus ret;
    uint16_t lengthP = 2;

    ret = aci_gatt_read_handle_value(g_appearance_char_handle+BlueNRGGattServer::CHAR_VALUE_HANDLE,
                                     lengthP,
                                     &lengthP,
                                     (uint8_t*)appearanceP);
    PRINTF("getAppearance ret=0x%02x (lengthP=%d)\n\r", ret, lengthP);
    if (ret == BLE_STATUS_SUCCESS) {
        return BLE_ERROR_NONE;
    } else {
        return BLE_ERROR_PARAM_OUT_OF_RANGE;
    }

}

GapScanningParams* BlueNRGGap::getScanningParams(void)
{
  return &_scanningParams;
}

static void makeConnection(void)
{
  BlueNRGGap::getInstance().createConnection();
}

void BlueNRGGap::Discovery_CB(Reason_t reason,
                              uint8_t adv_type,
                              uint8_t addr_type,
                              uint8_t *addr,
                              uint8_t *data_length,
                              uint8_t *data,
                              uint8_t *RSSI)
{
  switch (reason) {
  case DEVICE_FOUND:
    {
      GapAdvertisingParams::AdvertisingType_t type;
      bool isScanResponse = false;

      /*
       * Whitelisting (scan policy):
       * SCAN_POLICY_FILTER_ALL_ADV (ADV packets only from devs in the White List) &&
       * Private Random Address
       * => scan_results = FALSE
       * FIXME: the Security Manager should be implemented
       */
      ScanningPolicyMode_t mode = getScanningPolicyMode();
      PRINTF("mode=%u addr_type=%u\n\r", mode, addr_type);
      if(mode == Gap::SCAN_POLICY_FILTER_ALL_ADV ||
         (addr_type == RESOLVABLE_PRIVATE_ADDR ||
          addr_type == NON_RESOLVABLE_PRIVATE_ADDR)) {
        return;
      }

      switch(adv_type) {
      case ADV_IND:
        type = GapAdvertisingParams::ADV_CONNECTABLE_UNDIRECTED;
        break;
      case ADV_DIRECT_IND:
        type = GapAdvertisingParams::ADV_CONNECTABLE_DIRECTED;
        break;
      case ADV_SCAN_IND:
      case SCAN_RSP:
        type = GapAdvertisingParams::ADV_SCANNABLE_UNDIRECTED;
        isScanResponse = true;
        break;
      case ADV_NONCONN_IND:
        type = GapAdvertisingParams::ADV_NON_CONNECTABLE_UNDIRECTED;
        break;
      default:
        type = GapAdvertisingParams::ADV_CONNECTABLE_UNDIRECTED;
      }

      PRINTF("data_length=%d adv peerAddr[%02x %02x %02x %02x %02x %02x] \r\n",
             *data_length, addr[5], addr[4], addr[3], addr[2], addr[1], addr[0]);
      if(!_connecting) {
        processAdvertisementReport(addr, *RSSI, isScanResponse, type, *data_length, data);
      }
      PRINTF("!!!After processAdvertisementReport\n\r");
    }
    break;

  case DISCOVERY_COMPLETE:
    // The discovery is complete. If this is due to a stop scanning (i.e., the device
    // we are interested in has been found) and a connection has been requested
    // then we start the device connection.
    PRINTF("DISCOVERY_COMPLETE\n\r");
    _scanning = false;

    // Since the DISCOVERY_COMPLETE event can be received during the scanning interval,
    // we need to delay the starting of connection
    uint16_t delay = 2*(_scanningParams.getInterval());

#ifdef AST_FOR_MBED_OS
    if(_connecting) {
      minar::Scheduler::postCallback(makeConnection).delay(minar::milliseconds(delay));
    }
#else
    Clock_Wait(delay);
    if(_connecting) {
      makeConnection();
    }
#endif /* AST_FOR_MBED_OS */

    break;
  }
}

ble_error_t BlueNRGGap::startRadioScan(const GapScanningParams &scanningParams)
{

  tBleStatus ret = BLE_STATUS_SUCCESS;

  // Stop ADV before scanning
  /*
  if (state.advertising == 1) {
    stopAdvertising();
  }
  */

  /*
   * Whitelisting (scan policy):
   * SCAN_POLICY_FILTER_ALL_ADV (ADV packets only from devs in the White List) &&
   * White List is empty
   * => scan operation = FAILURE
   * FIXME: the Security Manager should be implemented
   */
  ScanningPolicyMode_t mode = getScanningPolicyMode();
  uint8_t whiteListSize = whitelistAddresses.size;
  if(whiteListSize == 0 && mode == Gap::SCAN_POLICY_FILTER_ALL_ADV) {
    return BLE_ERROR_OPERATION_NOT_PERMITTED;
  }

  ret = btleStartRadioScan(scanningParams.getActiveScanning(),
                           scanningParams.getInterval(),
                           scanningParams.getWindow(),
                           addr_type);

  PRINTF("Scanning...\n\r");
  PRINTF("scanningParams.getInterval()=%u[msec]\r\n",(scanningParams.getInterval()*625)/1000);
  PRINTF("scanningParams.getWindow()=%u[msec]\r\n",(scanningParams.getWindow()*625)/1000);
  //PRINTF("_advParams.getInterval()=%u\r\n",_advParams.getInterval());
  //PRINTF("CONN_P1=%u\r\n",(unsigned)CONN_P1);
  //PRINTF("CONN_P2=%u\r\n",(unsigned)CONN_P2);
  if (BLE_STATUS_SUCCESS == ret){
    PRINTF("Observation Procedure Started\n");
    _scanning = true;
    return BLE_ERROR_NONE;
  }

  // Observer role is not supported by X-NUCLEO-IDB04A1, return BLE_ERROR_NOT_IMPLEMENTED
  switch (ret) {
    case BLE_STATUS_INVALID_CID:
      PRINTF("Observation Procedure not implemented!!!\n\r");
      return BLE_ERROR_NOT_IMPLEMENTED;
    default:
      PRINTF("Observation Procedure failed (0x%02X)\n\r", ret);
      return BLE_ERROR_UNSPECIFIED;
  }

}

ble_error_t BlueNRGGap::stopScan() {
  tBleStatus ret = BLE_STATUS_SUCCESS;

  PRINTF("stopScan\n\r");
  ret = aci_gap_terminate_gap_procedure(GAP_OBSERVATION_PROC);

  if (ret != BLE_STATUS_SUCCESS) {
    PRINTF("GAP Terminate Gap Procedure failed(ret=0x%x)\n", ret);
    return BLE_ERROR_UNSPECIFIED;
  } else {
    PRINTF("Discovery Procedure Terminated\n");
    return BLE_ERROR_NONE;
  }
}

/**************************************************************************/
/*!
    @brief  set Tx power level
    @param[in] txPower Transmission Power level
    @returns    ble_error_t
*/
/**************************************************************************/
ble_error_t BlueNRGGap::setTxPower(int8_t txPower)
{
    tBleStatus ret;

    int8_t enHighPower = 0;
    int8_t paLevel = 0;

    ret = getHighPowerAndPALevelValue(txPower, enHighPower, paLevel);
    if(ret!=BLE_STATUS_SUCCESS) {
        return BLE_ERROR_PARAM_OUT_OF_RANGE;
    }

    PRINTF("enHighPower=%d, paLevel=%d\n\r", enHighPower, paLevel);
    ret = aci_hal_set_tx_power_level(enHighPower, paLevel);
    if(ret!=BLE_STATUS_SUCCESS) {
      return BLE_ERROR_PARAM_OUT_OF_RANGE;
    }

    return BLE_ERROR_NONE;
}

/**************************************************************************/
/*!
    @brief  get permitted Tx power values
    @param[in] values pointer to pointer to permitted power values
    @param[in] num number of values
*/
/**************************************************************************/
void BlueNRGGap::getPermittedTxPowerValues(const int8_t **valueArrayPP, size_t *countP) {
    static const int8_t permittedTxValues[] = {
        -18, -15, -14, -12, -11, -9, -8, -6, -5, -2, 0, 2, 4, 5, 8
    };

    *valueArrayPP = permittedTxValues;
    *countP = sizeof(permittedTxValues) / sizeof(int8_t);
}

/**************************************************************************/
/*!
    @brief  Set advertising parameters according to the current state
            Parameters value is set taking into account guidelines of the BlueNRG
            time slots allocation
*/
/**************************************************************************/
void BlueNRGGap::setAdvParameters(void)
{
  uint32_t advIntMS;

  if(state.connected == 1) {
    advIntMS = (conn_min_interval*1.25)-GUARD_INT;
    advInterval = _advParams.MSEC_TO_ADVERTISEMENT_DURATION_UNITS(advIntMS);

    PRINTF("conn_min_interval is equal to %u\r\n", conn_min_interval);
  } else {
    advInterval = _advParams.getIntervalInADVUnits();
  }
}

/**************************************************************************/
/*!
    @brief  Set connection parameters according to the current state (ADV and/or SCAN)
            Parameters value is set taking into account guidelines of the BlueNRG
            time slots allocation
*/
/**************************************************************************/
void BlueNRGGap::setConnectionParameters(void)
{
  if (state.advertising == 1) {

    if (_scanningParams.getInterval() < advInterval) {
      PRINTF("state.adv=1 scanInterval<advInterval\r\n");
      scanInterval = advInterval;
      scanWindow = advInterval;
    } else {
      PRINTF("state.adv=1 scanInterval>=advInterval\r\n");
      scanInterval = _scanningParams.getInterval();
      scanWindow = _scanningParams.getWindow();
    }

    if(advInterval>(MAX_INT_CONN-(GUARD_INT/1.25))) { //(4000-GUARD_INT)ms
        conn_min_interval = MAX_INT_CONN;
        conn_max_interval = MAX_INT_CONN;
    } else {
        conn_min_interval = (_advParams.ADVERTISEMENT_DURATION_UNITS_TO_MS(advInterval)+GUARD_INT)/1.25;
        conn_max_interval = (_advParams.ADVERTISEMENT_DURATION_UNITS_TO_MS(advInterval)+GUARD_INT)/1.25;
    }

  } else {

    PRINTF("state.adv = 0\r\n");

    scanInterval = _scanningParams.getInterval();
    scanWindow = _scanningParams.getWindow();
    if(SCAN_DURATION_UNITS_TO_MSEC(scanInterval)>(MAX_INT_CONN*1.25) ||
       SCAN_DURATION_UNITS_TO_MSEC(scanInterval)<(MIN_INT_CONN*1.25)) { //(4000)ms || (7.5)ms
        conn_min_interval = DEF_INT_CONN;
        conn_max_interval = DEF_INT_CONN;
    } else {
        conn_min_interval = SCAN_DURATION_UNITS_TO_MSEC(scanInterval)/1.25;
        conn_max_interval = SCAN_DURATION_UNITS_TO_MSEC(scanInterval)/1.25;
    }
  }
  PRINTF("scanInterval=%u[msec]\r\n",SCAN_DURATION_UNITS_TO_MSEC(scanInterval));
  PRINTF("scanWindow()=%u[msec]\r\n",SCAN_DURATION_UNITS_TO_MSEC(scanWindow));
  PRINTF("conn_min_interval=%u[msec]\r\n",(unsigned)(conn_min_interval*1.25));
  PRINTF("conn_max_interval=%u[msec]\r\n",(unsigned)(conn_max_interval*1.25));

}

ble_error_t BlueNRGGap::createConnection ()
{
  tBleStatus ret;

  /*
     Before creating connection, set parameters according
     to previous or current procedure (ADV and/or SCAN)
   */
  setConnectionParameters();

  /*
    Scan_Interval, Scan_Window, Peer_Address_Type, Peer_Address, Own_Address_Type, Conn_Interval_Min,
    Conn_Interval_Max, Conn_Latency, Supervision_Timeout, Conn_Len_Min, Conn_Len_Max
  */
  ret = aci_gap_create_connection(scanInterval,
				  scanWindow,
				  _peerAddrType,
				  (unsigned char*)_peerAddr,
				  addr_type,
				  conn_min_interval, conn_max_interval, 0,
				  SUPERV_TIMEOUT, CONN_L1, CONN_L1);

  //_connecting = false;

  if (ret != BLE_STATUS_SUCCESS) {
    PRINTF("Error while starting connection (ret=0x%02X).\n\r", ret);
    return BLE_ERROR_UNSPECIFIED;
  } else {
    PRINTF("Connection started.\n");
    _connecting = false;
    return BLE_ERROR_NONE;
  }
}

ble_error_t BlueNRGGap::connect (const Gap::Address_t peerAddr,
                                 Gap::AddressType_t peerAddrType,
                                 const ConnectionParams_t *connectionParams,
                                 const GapScanningParams *scanParams)
{
  /* avoid compiler warnings about unused variables */
  (void)connectionParams;

  setScanParams(scanParams->getInterval(),
                scanParams->getWindow(),
                scanParams->getTimeout(),
                scanParams->getActiveScanning()
               );

  // Save the peer address
  for(int i=0; i<BDADDR_SIZE; i++) {
    _peerAddr[i] = peerAddr[i];
  }
  _peerAddrType = peerAddrType;

  _connecting = true;

  if(_scanning) {
    stopScan();
  } else {
    PRINTF("Calling createConnection from connect()\n\r");
    return createConnection();
  }

  return BLE_ERROR_NONE;
}

/**************************************************************************/
/*!
    @brief  Set the advertising policy filter mode that will be used in
            the next call to startAdvertising().

    @returns    \ref ble_errror_t

    @retval     BLE_ERROR_NONE
                Everything executed properly.

                BLE_ERROR_NOT_IMPLEMENTED
                This feature is currently note implemented.
*/
/**************************************************************************/
ble_error_t BlueNRGGap::setAdvertisingPolicyMode(Gap::AdvertisingPolicyMode_t mode)
{
   advertisingPolicyMode = mode;

    return BLE_ERROR_NONE;
}

/**************************************************************************/
/*!
    @brief  Set the scanning policy filter mode that will be used in
            the next call to startAdvertising().

    @returns    \ref ble_errror_t

    @retval     BLE_ERROR_NONE
                Everything executed properly.

                BLE_ERROR_NOT_IMPLEMENTED
                This feature is currently note implemented.
*/
/**************************************************************************/
ble_error_t BlueNRGGap::setScanningPolicyMode(Gap::ScanningPolicyMode_t mode)
{
    scanningPolicyMode = mode;

    return BLE_ERROR_NONE;
}

/**************************************************************************/
/*!
    @brief  Get the current advertising policy filter mode.

    @returns    The advertising policy filter mode.
*/
/**************************************************************************/
Gap::AdvertisingPolicyMode_t BlueNRGGap::getAdvertisingPolicyMode(void) const
{
    return advertisingPolicyMode;
}

/**************************************************************************/
/*!
    @brief  Get the current scanning policy filter mode.

    @returns    The scanning policy filter mode.

*/
/**************************************************************************/
Gap::ScanningPolicyMode_t BlueNRGGap::getScanningPolicyMode(void) const
{
    return scanningPolicyMode;
}

/**************************************************************************/
/*!
    @brief  Clear BlueNRGGap's state.

    @returns    ble_error_t

    @retval     BLE_ERROR_NONE
                Everything executed properly
*/
/**************************************************************************/
ble_error_t BlueNRGGap::reset(void)
{
    /* Clear all state that is from the parent, including private members */
    if (Gap::reset() != BLE_ERROR_NONE) {
        return BLE_ERROR_INVALID_STATE;
    }

    /* Clear derived class members */
    m_connectionHandle = BLE_CONN_HANDLE_INVALID;

    /* Set the whitelist policy filter modes to IGNORE_WHITELIST */
    advertisingPolicyMode = Gap::ADV_POLICY_IGNORE_WHITELIST;
    scanningPolicyMode    = Gap::SCAN_POLICY_IGNORE_WHITELIST;

    return BLE_ERROR_NONE;
}

void BlueNRGGap::setConnectionInterval(uint16_t interval) {
    conn_min_interval = interval;
    conn_max_interval = interval;
}

void BlueNRGGap::setGapRole(Role_t role)
{
    gapRole = role;
}

