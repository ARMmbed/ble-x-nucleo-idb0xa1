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
  * @file    BlueNRGGattServer.cpp 
  * @author  STMicroelectronics
  * @brief   Implementation of BlueNRG BLE_API GattServer Class
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
 
/** @defgroup BlueNRGGATTSERVER
 *  @brief BlueNRG BLE_API GattServer Adaptation
 *  @{
 */
 
#include "BlueNRGGattServer.h"
#include "mbed-drivers/mbed.h"
#include "BlueNRGGap.h"
#include "Utils.h"
#include "debug.h"

/**************************************************************************/
/*!
    @brief  Adds a new service to the GATT table on the peripheral

    @params[in] service
                Pointer to instance of the Gatt Server to add
                
    @returns    ble_error_t

    @retval     BLE_ERROR_NONE
                Everything executed properly

    @section EXAMPLE

    @code

    @endcode
*/
/**************************************************************************/
ble_error_t BlueNRGGattServer::addService(GattService &service)
{
    /* ToDo: Make sure we don't overflow the array, etc. */
    /* ToDo: Make sure this service UUID doesn't already exist (?) */
    /* ToDo: Basic validation */
    
    tBleStatus ret;
    uint8_t type;
    uint16_t short_uuid;
    uint8_t primary_short_uuid[2];
    uint8_t primary_base_uuid[16];
    uint8_t char_base_uuid[16];
    const uint8_t *base_uuid;
    const uint8_t *base_char_uuid;
    
    uint8_t charsCount = 0;
    uint8_t maxAttrRecords = 0;

    type = (service.getUUID()).shortOrLong();
    PRINTF("AddService(): Type:%d\n\r", type);
    
    /* Add the service to the BlueNRG */
    short_uuid = (service.getUUID()).getShortUUID();
    STORE_LE_16(primary_short_uuid, short_uuid);
    
    if(type==UUID::UUID_TYPE_LONG) {
        base_uuid = (service.getUUID()).getBaseUUID();   
        
        COPY_UUID_128(primary_base_uuid, base_uuid[15],base_uuid[14],primary_short_uuid[1],primary_short_uuid[0],base_uuid[11],base_uuid[10],base_uuid[9],base_uuid[8],base_uuid[7],base_uuid[6],base_uuid[5],base_uuid[4],base_uuid[3],base_uuid[2],base_uuid[1],base_uuid[0]);
    }

    charsCount = service.getCharacteristicCount();
    //1(service record)+2records*char+1record*char_desc
    maxAttrRecords = 1+3*charsCount;

    if(type==UUID::UUID_TYPE_SHORT) {
        ret = aci_gatt_add_serv(UUID_TYPE_16,
                                primary_short_uuid,
                                PRIMARY_SERVICE,
                                maxAttrRecords/*7*/,
                                &servHandle);
        PRINTF("aci_gatt_add_serv UUID_TYPE_LONG ret=%d\n\r", ret);
    }
    else if(type==UUID::UUID_TYPE_LONG) {
        ret = aci_gatt_add_serv(UUID_TYPE_128,
                                primary_base_uuid,
                                PRIMARY_SERVICE,
                                maxAttrRecords/*7*/,
                                &servHandle);
        PRINTF("aci_gatt_add_serv UUID_TYPE_LONG ret=%d\n\r", ret);
    }
    
    service.setHandle(servHandle);
    //serviceHandleVector.push_back(servHandle);
    PRINTF("added servHandle handle =%u\n\r", servHandle);
    uint16_t bleCharacteristic;
    
    //iterate to include all characteristics
    for (uint8_t i = 0; i < charsCount; i++) {
        GattCharacteristic *p_char = service.getCharacteristic(i);
        uint16_t char_uuid = (p_char->getValueAttribute().getUUID()).getShortUUID();

        uint8_t int_8_uuid[2];
        STORE_LE_16(int_8_uuid, char_uuid);

        type = (p_char->getValueAttribute().getUUID()).shortOrLong();

        if(type==UUID::UUID_TYPE_LONG) {
            base_char_uuid = (p_char->getValueAttribute().getUUID()).getBaseUUID();
#ifdef DEBUG
            for(uint8_t j=0; j<16; j++) {
                PRINTF("base_char_uuid[%d] 0x%02x ", j, base_char_uuid[j]);
            }
            PRINTF("\n\r");
#endif
            COPY_UUID_128(char_base_uuid,base_char_uuid[15],base_char_uuid[14],int_8_uuid[1],int_8_uuid[0],base_char_uuid[11],base_char_uuid[10],base_char_uuid[9],base_char_uuid[8],base_char_uuid[7],base_char_uuid[6],base_char_uuid[5],base_char_uuid[4],base_char_uuid[3],base_char_uuid[2],base_char_uuid[1],base_char_uuid[0]);
        }
        
        PRINTF("Char Properties 0x%x\n\r", p_char->getProperties());
        /*
        * Gatt_Evt_Mask -> HardCoded (0)
        * Encryption_Key_Size -> Hardcoded (16)
        * isVariable (variable length value field) -> Hardcoded (1)
        */
        uint8_t Gatt_Evt_Mask = 0x0;

        if((p_char->getProperties() &
                    (GattCharacteristic::BLE_GATT_CHAR_PROPERTIES_WRITE_WITHOUT_RESPONSE|
                        GattCharacteristic::BLE_GATT_CHAR_PROPERTIES_WRITE))) {
            PRINTF("Setting up Gatt GATT_NOTIFY_ATTRIBUTE_WRITE Mask\n\r");
            Gatt_Evt_Mask = Gatt_Evt_Mask | GATT_NOTIFY_ATTRIBUTE_WRITE;
        }
        if((p_char->getProperties() &
                    (GattCharacteristic::BLE_GATT_CHAR_PROPERTIES_READ|
                        GattCharacteristic::BLE_GATT_CHAR_PROPERTIES_NOTIFY| GattCharacteristic::BLE_GATT_CHAR_PROPERTIES_INDICATE))) {
            PRINTF("Setting up Gatt GATT_NOTIFY_READ_REQ_AND_WAIT_FOR_APPL_RESP Mask\n\r");
            Gatt_Evt_Mask = Gatt_Evt_Mask | GATT_NOTIFY_READ_REQ_AND_WAIT_FOR_APPL_RESP; 
        }    //This will support also GATT_SERVER_ATTR_READ_WRITE since it will be covered by previous if() check.

        if(type==UUID::UUID_TYPE_SHORT) {
            ret =  aci_gatt_add_char(service.getHandle(),
                                     UUID_TYPE_16,
                                     int_8_uuid,
                                     p_char->getValueAttribute().getMaxLength() /*2*/ /*Value Length*/,
                                     p_char->getProperties(),
                                     ATTR_PERMISSION_NONE,
                                     Gatt_Evt_Mask /*Gatt_Evt_Mask*/,
                                     16 /*Encryption_Key_Size*/,
                                     1 /*isVariable*/,
                                     &bleCharacteristic);
            
            PRINTF("aci_gatt_add_char UUID_TYPE_16 props=%d MaxLength=%d ret=%d\n\r",
                    p_char->getProperties(), p_char->getValueAttribute().getMaxLength(), ret);

        } else if(type==UUID::UUID_TYPE_LONG) {
            ret =  aci_gatt_add_char(service.getHandle(),
                                     UUID_TYPE_128,
                                     char_base_uuid,
                                     p_char->getValueAttribute().getMaxLength() /*2*/ /*Value Length*/,
                                     p_char->getProperties(),
                                     ATTR_PERMISSION_NONE,
                                     Gatt_Evt_Mask /*Gatt_Evt_Mask*/,
                                     16 /*Encryption_Key_Size*/,
                                     1 /*isVariable*/,
                                     &bleCharacteristic);
            
            PRINTF("aci_gatt_add_char UUID_TYPE_128 props=%d MaxLength=%d ret=%d\n\r",
                    p_char->getProperties(), p_char->getValueAttribute().getMaxLength(), ret);
        }
        
        bleCharHandleMap.insert(std::pair<uint16_t, uint16_t>(bleCharacteristic, servHandle));
        
        p_characteristics[characteristicCount++] = p_char;
        /* Set the characteristic value handle */
        p_char->getValueAttribute().setHandle(bleCharacteristic+BlueNRGGattServer::CHAR_VALUE_HANDLE);
        PRINTF("added bleCharacteristic (value handle =%u)\n\r", p_char->getValueAttribute().getHandle());

        if ((p_char->getValueAttribute().getValuePtr() != NULL) && (p_char->getValueAttribute().getLength() > 0)) {
            write(p_char->getValueAttribute().getHandle(),
                  p_char->getValueAttribute().getValuePtr(),
                  p_char->getValueAttribute().getLength(), false /* localOnly */);
        }

        // add descriptors now
        uint16_t descHandle = 0;
        PRINTF("p_char->getDescriptorCount()=%d\n\r", p_char->getDescriptorCount());
        
        for(uint8_t descIndex=0; descIndex<p_char->getDescriptorCount(); descIndex++) {
            GattAttribute *descriptor = p_char->getDescriptor(descIndex);
            uint16_t shortUUID = descriptor->getUUID().getShortUUID();
            const uint8_t uuidArray[] = {(uint8_t)((shortUUID>>8)&0xFF), (uint8_t)((shortUUID&0xFF))};
            ret = aci_gatt_add_char_desc(service.getHandle(),
                                         bleCharacteristic,
                                         CHAR_DESC_TYPE_16_BIT,
                                         uuidArray,
                                         descriptor->getMaxLength(),
                                         descriptor->getLength(),
                                         descriptor->getValuePtr(),
                                         CHAR_DESC_SECURITY_PERMISSION,
                                         CHAR_DESC_ACCESS_PERMISSION,
                                         GATT_NOTIFY_ATTRIBUTE_WRITE,
                                         MIN_ENCRY_KEY_SIZE,
                                         CHAR_ATTRIBUTE_LEN_IS_FIXED,
                                         &descHandle);
            PRINTF("Adding Descriptor descriptor handle=%d ret=%d\n\r", descHandle, ret);
            if(ret==(tBleStatus)0) {
                PRINTF("Descriptor added successfully, descriptor handle=%d\n\r", descHandle);
                descriptor->setHandle(descHandle);
            }
        }

    }    
    
    serviceCount++;
    
    //FIXME: There is no GattService pointer array in GattServer. 
    //        There should be one? (Only the user is aware of GattServices!) Report to forum.
    
    return BLE_ERROR_NONE;
}

/**************************************************************************/
/*!
    @brief  Reads the value of a characteristic, based on the service
            and characteristic index fields

    @param[in]  charHandle
                The handle of the GattCharacteristic to read from
    @param[in]  buffer
                Buffer to hold the the characteristic's value
                (raw byte array in LSB format)
    @param[in]  lengthP
                The number of bytes read into the buffer

    @returns    ble_error_t

    @retval     BLE_ERROR_NONE
                Everything executed properly

    @section EXAMPLE

    @code

    @endcode
*/
/**************************************************************************/
ble_error_t BlueNRGGattServer::read(GattAttribute::Handle_t attributeHandle, uint8_t buffer[], uint16_t *lengthP)
{
  tBleStatus ret;
  uint16_t charHandle = attributeHandle-BlueNRGGattServer::CHAR_VALUE_HANDLE;

  ret = aci_gatt_read_handle_value(charHandle, *lengthP, lengthP, buffer);

  if(ret == BLE_STATUS_SUCCESS) {
    return BLE_ERROR_NONE;
  }
  switch (ret) {
  case ERR_INVALID_HCI_CMD_PARAMS:
    return BLE_ERROR_INVALID_PARAM;
  default:
    return BLE_ERROR_UNSPECIFIED;
  }
}

/**************************************************************************/
/*!
    @brief  Updates the value of a characteristic, based on the service
            and characteristic index fields

    @param[in]  charHandle
                The handle of the GattCharacteristic to write to
    @param[in]  buffer
                Data to use when updating the characteristic's value
                (raw byte array in LSB format)
    @param[in]  len
                The number of bytes in buffer

    @returns    ble_error_t

    @retval     BLE_ERROR_NONE
                Everything executed properly

    @section EXAMPLE

    @code

    @endcode
*/
/**************************************************************************/
ble_error_t BlueNRGGattServer::read(Gap::Handle_t connectionHandle,
                                    GattAttribute::Handle_t attributeHandle,
                                    uint8_t buffer[],
                                    uint16_t *lengthP) {

  /* avoid compiler warnings about unused variables */
  (void)connectionHandle;
  (void)attributeHandle;
  (void)buffer;
  (void)lengthP;

  return BLE_ERROR_NONE;
}

ble_error_t BlueNRGGattServer::write(Gap::Handle_t connectionHandle,
                                     GattAttribute::Handle_t,
                                     const uint8_t[],
                                     uint16_t, bool localOnly) {
  /* avoid compiler warnings about unused variables */
  (void)connectionHandle;
  (void)localOnly;

  return BLE_ERROR_NONE;
}
    
ble_error_t BlueNRGGattServer::write(GattAttribute::Handle_t attributeHandle, const uint8_t buffer[], uint16_t len, bool localOnly)
{
    /* avoid compiler warnings about unused variables */
    (void)localOnly;

    tBleStatus ret;

    uint16_t charHandle = attributeHandle-BlueNRGGattServer::CHAR_VALUE_HANDLE;

    PRINTF("updating bleCharacteristic valueHandle=%u,\
            corresponding serviceHandle=%u len=%d\n\r",
            attributeHandle, bleCharHandleMap.find(charHandle)->second, len);

    ret = aci_gatt_update_char_value(bleCharHandleMap.find(charHandle)->second, charHandle, 0, len, buffer);

    if (ret != BLE_STATUS_SUCCESS){
      PRINTF("Error while updating characteristic (ret=0x%x).\n\r", ret);
      switch (ret) {
        case BLE_STATUS_INVALID_HANDLE:
        case BLE_STATUS_INVALID_PARAMETER:
          return BLE_ERROR_INVALID_PARAM;
        default:
          return BLE_STACK_BUSY;
      }
    }

    //Generate Data Sent Event Here? (GattServerEvents::GATT_EVENT_DATA_SENT)  //FIXME: Is this correct?
    //Check if characteristic property is NOTIFY|INDICATE, if yes generate event
    GattCharacteristic *p_char = BlueNRGGattServer::getInstance().getCharacteristicFromHandle(charHandle);
    if(p_char->getProperties() &  (GattCharacteristic::BLE_GATT_CHAR_PROPERTIES_NOTIFY
                | GattCharacteristic::BLE_GATT_CHAR_PROPERTIES_INDICATE)) {
        PRINTF("Generate event after updating\n\r");
        BlueNRGGattServer::getInstance().handleEvent(GattServerEvents::GATT_EVENT_DATA_SENT, charHandle);
    }

    return BLE_ERROR_NONE;
}

/**************************************************************************/
/*!
    @brief  Reads a value according to the handle provided

    @param[in]  attributeHandle
                The handle of the attribute to read from

    @returns    ble_error_t

    @retval     BLE_ERROR_NONE
                Everything executed properly

    @section EXAMPLE

    @code

    @endcode
*/
/**************************************************************************/
ble_error_t BlueNRGGattServer::Read_Request_CB(uint16_t attributeHandle)
{
    uint16_t gapConnectionHandle = BlueNRGGap::getInstance().getConnectionHandle();
    
    GattReadCallbackParams readParams;
    readParams.handle = attributeHandle;

    //PRINTF("readParams.handle = %d\n\r", readParams.handle);
    HCIDataReadEvent(&readParams);
    
    //EXIT:
    if(gapConnectionHandle != 0){
        //PRINTF("Calling aci_gatt_allow_read\n\r");
        aci_gatt_allow_read(gapConnectionHandle);
    }
    
    return BLE_ERROR_NONE;
}

/**************************************************************************/
/*!
    @brief  Returns the GattCharacteristic according to the handle provided

    @param[in]  attrHandle
                The handle of the attribute

    @returns    ble_error_t

    @retval     BLE_ERROR_NONE
                Everything executed properly

    @section EXAMPLE

    @code

    @endcode
*/
/**************************************************************************/
GattCharacteristic* BlueNRGGattServer::getCharacteristicFromHandle(uint16_t attrHandle)
{
    GattCharacteristic *p_char = NULL;
    int i;
    uint16_t handle, handle_1;

    PRINTF("BlueNRGGattServer::getCharacteristicFromHandle()>>Attr Handle received %d\n\r",attrHandle);
    for(i=0; i<characteristicCount; i++)
    {
        handle = p_characteristics[i]->getValueAttribute().getHandle()-BlueNRGGattServer::CHAR_VALUE_HANDLE;
        PRINTF("handle(%d)=%d\n\r", i, handle);
        if(i==characteristicCount-1)//Last Characteristic check
        {
            if(attrHandle>=handle)
            {
                p_char = p_characteristics[i];
                PRINTF("Found Characteristic Properties 0x%x (handle=%d)\n\r",p_char->getProperties(), handle);
                break;
            }            
        }
        else {
            handle_1 = p_characteristics[i+1]->getValueAttribute().getHandle()-BlueNRGGattServer::CHAR_VALUE_HANDLE;
            //Testing if attribute handle is between two Characteristic Handles
            if(attrHandle>=handle && attrHandle<handle_1)
            {
                p_char = p_characteristics[i];
                PRINTF("Found Characteristic Properties 0x%x (handle=%d handle_1=%d)\n\r",p_char->getProperties(), handle, handle_1);
                break;
            } else continue;
        }
    }

    return p_char;
}

void BlueNRGGattServer::HCIDataWrittenEvent(const GattWriteCallbackParams *params) {
    this->handleDataWrittenEvent(params);
}
    
void BlueNRGGattServer::HCIDataReadEvent(const GattReadCallbackParams *params) {
    PRINTF("Called HCIDataReadEvent\n\r");
    this->handleDataReadEvent(params);
}

void BlueNRGGattServer::HCIEvent(GattServerEvents::gattEvent_e type, uint16_t charHandle) {
    this->handleEvent(type, charHandle);
}

void BlueNRGGattServer::HCIDataSentEvent(unsigned count) {
    this->handleDataSentEvent(count);
}

    
ble_error_t BlueNRGGattServer::initializeGATTDatabase(void)   {
    // <TODO>    
    return (ble_error_t)0;       
}

/**************************************************************************/
/*!
    @brief  Clear BlueNRGGattServer's state.

    @returns    ble_error_t

    @retval     BLE_ERROR_NONE
                Everything executed properly
*/
/**************************************************************************/
ble_error_t BlueNRGGattServer::reset(void)
{
    /* Clear all state that is from the parent, including private members */
    if (GattServer::reset() != BLE_ERROR_NONE) {
        return BLE_ERROR_INVALID_STATE;
    }

    /* Clear class members */
    memset(p_characteristics,        0, sizeof(p_characteristics));
    memset(bleCharacteristicHandles, 0, sizeof(bleCharacteristicHandles));
    serviceCount = 0;
    characteristicCount = 0;

    return BLE_ERROR_NONE;
}
