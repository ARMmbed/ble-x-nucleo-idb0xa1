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
 
/** @defgroup BlueNRGGATTClient
 *  @brief BlueNRG BLE_API GattClient Adaptation
 *  @{
 */
 
#include "BlueNRGGattClient.h"
#include "mbed-drivers/mbed.h"
#include "BlueNRGGap.h"
#include "Utils.h"
#include "debug.h"

static uint8_t props_mask[] = {
  0x01,
  0x02,
  0x04,
  0x08,
  0x10,
  0x20,
  0x40,
  0x80
  };

void BlueNRGGattClient::gattProcedureCompleteCB(Gap::Handle_t connectionHandle, uint8_t error_code)
{
  if(error_code != BLE_STATUS_SUCCESS) {
    return;
  }
    
  // Service Discovery complete
  if(_currentState != GATT_IDLE && 
     _currentState != GATT_DISCOVERY_TERMINATED &&
       _currentState != GATT_WRITE_CHAR &&
         _currentState != GATT_READ_CHAR) {
             
    findServiceChars(connectionHandle);
  }
  
  if(_currentState == GATT_WRITE_CHAR) {
    BlueNRGGattClient::getInstance().processWriteResponse(&writeCBParams);
    _currentState = GATT_IDLE;
  }
}
                                                
void BlueNRGGattClient::primaryServicesCB(Gap::Handle_t connectionHandle,
                                          uint8_t event_data_length,
                                          uint8_t attribute_data_length,
                                          uint8_t *attribute_data_list)
{
  GattAttribute::Handle_t startHandle, endHandle;
  UUID uuid;
  uint8_t i, offset, numAttr;
  /* avoid compiler warnings about unused variables */
  (void)connectionHandle;

  numAttr = (event_data_length - 1) / attribute_data_length;

  offset = 0;
  for (i=0; i<numAttr; i++) {
    startHandle = attribute_data_list[offset];
    endHandle = attribute_data_list[offset+2];

    // UUID Type
    if (attribute_data_length == 6) {
      
      PRINTF("UUID_TYPE_16\n\r");
      uuid = attribute_data_list[offset+5]<<8|attribute_data_list[offset+4];
      PRINTF("S UUID-%X attrs[%u %u]\r\n", uuid.getShortUUID(), startHandle, endHandle);
      
    } else {
      
      PRINTF("UUID_TYPE_128\n\r");
      uuid.setupLong(attribute_data_list+offset+4);
      
      PRINTF("S UUID-");
#ifdef DEBUG
      const uint8_t *longUUIDBytes = uuid.getBaseUUID();
      for (unsigned j = 0; j < UUID::LENGTH_OF_LONG_UUID; j++) {
        PRINTF("%02x", longUUIDBytes[j]);
      }
#endif
      PRINTF(" attrs[%u %u]\r\n", startHandle, endHandle);
      
    }
    
    PRINTF("Setup serviceIndex = %d\n\r", _numServices);
    discoveredService[_numServices].setup(uuid, startHandle, endHandle);
    
    if(serviceDiscoveryCallback) {
      if(_matchingServiceUUID == BLE_UUID_UNKNOWN || _matchingServiceUUID == discoveredService[_numServices].getUUID()) {
        serviceDiscoveryCallback(&discoveredService[_numServices]);
      }
    }
    _numServices++;

    offset += attribute_data_length;
  }

  PRINTF("!!!Service Discovery complete (numAttr=%u)!!!\n\r", numAttr);

}

void BlueNRGGattClient::primaryServiceCB(Gap::Handle_t connectionHandle,
                                         uint8_t event_data_length,
                                         uint8_t *handles_info_list)
{
  GattAttribute::Handle_t startHandle, endHandle;
  UUID uuid;
  uint8_t i, offset, numHandlePairs;
  /* avoid compiler warnings about unused variables */
  (void)connectionHandle;

  numHandlePairs = (event_data_length - 1) / 2;

  offset = 0;
  for (i=0; i<numHandlePairs; i++) {
    startHandle = handles_info_list[offset];
    endHandle = handles_info_list[offset+2];

    PRINTF("primaryServiceCB attrs[%u %u]\r\n", startHandle, endHandle);
    
    discoveredService[i].setup(_matchingServiceUUID, startHandle, endHandle);
    
    if(serviceDiscoveryCallback) {
      serviceDiscoveryCallback(&discoveredService[_numServices]);
    }
    _numServices++;
    
    offset += 4;
  }
}

void BlueNRGGattClient::serviceCharsCB(Gap::Handle_t connectionHandle,
                                       uint8_t event_data_length,
                                       uint8_t handle_value_pair_length,
                                       uint8_t *handle_value_pair)
{
  // Charac Handle (2), Charac Properties(1), Charac Value Handle(2), Charac UUID(2/16)

  GattAttribute::Handle_t declHandle, valueHandle;
  UUID uuid;
  uint8_t i, numChar, offset;

  numChar = (event_data_length - 1) / handle_value_pair_length;

  offset = 0;
  for (i=0; i<numChar; i++) {
    // UUID Type
    if (handle_value_pair_length == 7) {
      PRINTF("Char UUID_TYPE_16\n\r");
      uuid = handle_value_pair[offset+6]<<8|handle_value_pair[offset+5];
      PRINTF("C UUID-%X\r\n", uuid.getShortUUID());
    } else {
      PRINTF("Char UUID_TYPE_128\n\r");
      uuid.setupLong(handle_value_pair+offset+5);
      PRINTF("C UUID-");
#ifdef DEBUG
      const uint8_t *longUUIDBytes = uuid.getBaseUUID();
      for (unsigned i = 0; i < UUID::LENGTH_OF_LONG_UUID; i++) {
        PRINTF("%02X", longUUIDBytes[i]);
      }
      PRINTF("\r\n");
#endif
    }
    
    // Properties
    DiscoveredCharacteristic::Properties_t p;
    
    p._broadcast = (props_mask[0] & handle_value_pair[offset+2]);
    p._read = (props_mask[1] & handle_value_pair[offset+2])>>1;
    p._writeWoResp = (props_mask[2] & handle_value_pair[offset+2])>>2;
    p._write = (props_mask[3] & handle_value_pair[offset+2])>>3;
    p._notify = (props_mask[4] & handle_value_pair[offset+2])>>4;
    p._indicate = (props_mask[5] & handle_value_pair[offset+2])>>5;
    p._authSignedWrite = (props_mask[6] & handle_value_pair[offset+2])>>6;
    PRINTF("p._broadcast=%d\n\r", p._broadcast);
    PRINTF("p._read=%d\n\r", p._read);
    PRINTF("p._writeWoResp=%d\n\r", p._writeWoResp);
    PRINTF("p._write=%d\n\r", p._write);
    PRINTF("p._notify=%d\n\r", p._notify);
    PRINTF("p._indicate=%d\n\r", p._indicate);
    PRINTF("p._authSignedWrite=%d\n\r", p._authSignedWrite);

    /*
    uint8_t props = handle_value_pair[offset+2];
    PRINTF("CHAR PROPS: %d\n\r", props);
    */

    // Handles
    declHandle = handle_value_pair[offset];
    valueHandle = handle_value_pair[offset+3];

    discoveredChar[_numChars].setup(this,
                                    connectionHandle,
                                    uuid,
                                    p,
                                    declHandle,
                                    valueHandle);

    if(characteristicDiscoveryCallback) {
      characteristicDiscoveryCallback(&discoveredChar[_numChars]);
    }
    _numChars++;

    offset += handle_value_pair_length;
  }
}

void BlueNRGGattClient::serviceCharByUUIDCB(Gap::Handle_t connectionHandle,
                                            uint8_t event_data_length,
                                            uint16_t attr_handle,
                                            uint8_t *attr_value)
{
  // Charac Properties(1), Charac Value Handle(2), Charac UUID(2/16)
  GattAttribute::Handle_t declHandle, valueHandle;
  UUID uuid;
  
  PRINTF("serviceCharByUUIDCB\n\r");
  
  // UUID Type
  if (event_data_length == 7) {
    PRINTF("Char UUID_TYPE_16\n\r");
    uuid = attr_value[4]<<8|attr_value[3];
    PRINTF("C UUID-%X\r\n", uuid.getShortUUID());
  } else {
    PRINTF("Char UUID_TYPE_128\n\r");
    uuid.setupLong(attr_value+3);
    PRINTF("C UUID-");
#ifdef DEBUG
    const uint8_t *longUUIDBytes = uuid.getBaseUUID();
    for (unsigned i = 0; i < UUID::LENGTH_OF_LONG_UUID; i++) {
      PRINTF("%02X", longUUIDBytes[i]);
    }
    PRINTF("\r\n");
#endif
  }

  // Properties
  DiscoveredCharacteristic::Properties_t p;
  
  p._broadcast = (props_mask[0] & attr_value[0]);
  p._read = (props_mask[1] & attr_value[0])>>1;
  p._writeWoResp = (props_mask[2] & attr_value[0])>>2;
  p._write = (props_mask[3] & attr_value[0])>>3;
  p._notify = (props_mask[4] & attr_value[0])>>4;
  p._indicate = (props_mask[5] & attr_value[0])>>5;
  p._authSignedWrite = (props_mask[6] & attr_value[0])>>6;
  PRINTF("p._broadcast=%d\n\r", p._broadcast);
  PRINTF("p._read=%d\n\r", p._read);
  PRINTF("p._writeWoResp=%d\n\r", p._writeWoResp);
  PRINTF("p._write=%d\n\r", p._write);
  PRINTF("p._notify=%d\n\r", p._notify);
  PRINTF("p._indicate=%d\n\r", p._indicate);
  PRINTF("p._authSignedWrite=%d\n\r", p._authSignedWrite);

  /*
  uint8_t props = attr_value[0];
  PRINTF("CHAR PROPS: %d\n\r", props);
  */

  // Handles
  declHandle = attr_handle;
  valueHandle = attr_value[1];

  discoveredChar[_numChars].setup(this,
                                  connectionHandle,
                                  uuid,
                                  p,
                                  declHandle,
                                  valueHandle);
  
  if(characteristicDiscoveryCallback) {
    characteristicDiscoveryCallback(&discoveredChar[_numChars]);
  }
  _numChars++;
}

ble_error_t BlueNRGGattClient::findServiceChars(Gap::Handle_t connectionHandle)
{
  PRINTF("findServiceChars\n\r");
  
  tBleStatus ret;
  uint8_t uuid_type = UUID_TYPE_16;
  uint8_t short_uuid[2];
  uint8_t *uuid = NULL;
  
  DiscoveredService *service;
  
  // We finished chars discovery for all services
  if(_servIndex >= _numServices) {
    PRINTF("!!!We finished chars discovery for all services!!!\n\r");
    _currentState = GATT_CHARS_DISCOVERY_COMPLETE;
    
    terminateServiceDiscovery();
    
    return BLE_ERROR_NONE;
  }
    
  service = &discoveredService[_servIndex];
  /*
  if (service->getUUID().shortOrLong() == UUID::UUID_TYPE_SHORT) {
    PRINTF("S UUID-%X\r\n", service->getUUID().getShortUUID());
  } else {
    PRINTF("S UUID-");
    const uint8_t *longUUIDBytes = service->getUUID().getBaseUUID();
    for (unsigned i = 0; i < UUID::LENGTH_OF_LONG_UUID; i++) {
      PRINTF("%02X", longUUIDBytes[i]);
    }
    PRINTF("\r\n");
  }
  */
  
  PRINTF("findServiceChars (_servIndex=%d)\n\r", _servIndex);
  //ret = aci_gatt_disc_all_charac_of_serv(connectionHandle, service->getStartHandle(), service->getEndHandle());

  if(_matchingCharacteristicUUIDIn == BLE_UUID_UNKNOWN) {
    PRINTF("findServiceChars (BLE_UUID_UNKNOWN)\n\r");
    ret = aci_gatt_disc_all_charac_of_serv(connectionHandle, service->getStartHandle(), service->getEndHandle());
  } else {
    
    uint8_t type = _matchingCharacteristicUUIDIn.shortOrLong();
    
    if(type == UUID::UUID_TYPE_SHORT) {
      STORE_LE_16(short_uuid, _matchingCharacteristicUUIDIn.getShortUUID());
      
      uuid_type = UUID_TYPE_16;
      uuid = short_uuid;
      
      PRINTF("findServiceChars C UUID-");
      for(unsigned i = 0; i < 2; i++) {
        PRINTF("%02X", short_uuid[i]);
      }
      PRINTF("\n\r");

    } else if(type==UUID::UUID_TYPE_LONG) {
      
      uuid_type = UUID_TYPE_128;
      uuid = (unsigned char*)_matchingCharacteristicUUIDIn.getBaseUUID();
      
      PRINTF("(findServiceChars) C UUID-");
      for (unsigned i = 0; i < UUID::LENGTH_OF_LONG_UUID; i++) {
        PRINTF("%02X", uuid[i]);
      }
      PRINTF("\r\n");
    }

    ret = aci_gatt_disc_charac_by_uuid(connectionHandle,
                                       service->getStartHandle(),
                                       service->getEndHandle(),
                                       uuid_type,
                                       uuid);
  }
  
  if(ret == BLE_STATUS_SUCCESS) {
    _servIndex++;
  }
  
  PRINTF("findServiceChars ret=%d\n\r", ret);
  
  return BLE_ERROR_NONE;
}

ble_error_t BlueNRGGattClient::launchServiceDiscovery(Gap::Handle_t                               connectionHandle,
                                                      ServiceDiscovery::ServiceCallback_t         sc,
                                                      ServiceDiscovery::CharacteristicCallback_t  cc,
                                                      const UUID                                 &matchingServiceUUID,
                                                      const UUID                                 &matchingCharacteristicUUIDIn)
{
  PRINTF("launchServiceDiscovery\n\r");
  
  tBleStatus ret;
  uint8_t uuid_type = UUID_TYPE_16;
  uint8_t short_uuid[2];
  uint8_t *uuid = NULL;
  unsigned j;
  
  _connectionHandle = connectionHandle;
  serviceDiscoveryCallback = sc;
  characteristicDiscoveryCallback = cc;
  _matchingServiceUUID = matchingServiceUUID;
  _matchingCharacteristicUUIDIn = matchingCharacteristicUUIDIn;

  //reset services
  _numServices = 0;
  _numChars = 0;
  _servIndex = 0;
  for(j = 0; j < BLE_TOTAL_DISCOVERED_SERVICES; j++) {
    discoveredService[j].setup(BLE_UUID_UNKNOWN, GattAttribute::INVALID_HANDLE, GattAttribute::INVALID_HANDLE);
  }
  
  if(matchingServiceUUID == BLE_UUID_UNKNOWN) {
    
    // Wildcard: search for all services
    ret = aci_gatt_disc_all_prim_services((uint16_t)connectionHandle);
    
  } else {
  
    uint8_t type = matchingServiceUUID.shortOrLong();
    //PRINTF("AddService(): Type:%d\n\r", type);
    
    if(type == UUID::UUID_TYPE_SHORT) {
      STORE_LE_16(short_uuid, matchingServiceUUID.getShortUUID());
      
      PRINTF("launchServiceDiscovery short_uuid=0x");
      for(j = 0; j < 2; j++) {
        PRINTF("%02X", short_uuid[j]);
      }
      PRINTF("\n\r");
      
      
      uuid_type = UUID_TYPE_16;
      uuid = short_uuid;
      
    } else if(type==UUID::UUID_TYPE_LONG) {

      uuid_type = UUID_TYPE_128;
      uuid = (unsigned char*)matchingServiceUUID.getBaseUUID();
      
      /*
      PRINTF("launchServiceDiscovery base_uuid=0x");
      for(j = 0; j < 16; j++) {
        PRINTF("%02X", uuid[j]);
      }
      PRINTF("\n\r");
      */
    }
    
    // search for specific service by UUID
    ret = aci_gatt_disc_prim_service_by_uuid((uint16_t)connectionHandle, uuid_type, uuid);
    //ret = aci_gatt_disc_all_prim_services((uint16_t)connectionHandle);
  }
    
  if(ret == BLE_STATUS_SUCCESS) {
    _currentState = GATT_SERVICE_DISCOVERY;
  }
  
  PRINTF("launchServiceDiscovery ret=%d\n\r", ret);
  
  return BLE_ERROR_NONE;
}

ble_error_t BlueNRGGattClient::discoverServices(Gap::Handle_t                        connectionHandle,
                                                ServiceDiscovery::ServiceCallback_t  callback,
                                                const UUID                          &matchingServiceUUID)
{
  /* avoid compiler warnings about unused variables */
  (void)connectionHandle;
  (void)callback;
  (void)matchingServiceUUID;

  return BLE_ERROR_NOT_IMPLEMENTED;
}

ble_error_t BlueNRGGattClient::discoverServices(Gap::Handle_t                        connectionHandle,
                                                ServiceDiscovery::ServiceCallback_t  callback,
                                                GattAttribute::Handle_t              startHandle,
                                                GattAttribute::Handle_t              endHandle)
{
  /* avoid compiler warnings about unused variables */
  (void)connectionHandle;
  (void)callback;
  (void)startHandle;
  (void)endHandle;

  return BLE_ERROR_NOT_IMPLEMENTED;
}

bool BlueNRGGattClient::isServiceDiscoveryActive(void) const
{
  if(_currentState == GATT_IDLE ||
     _currentState == GATT_DISCOVERY_TERMINATED ||
       _currentState == GATT_READ_CHAR ||
         _currentState == GATT_WRITE_CHAR ) {
    return false;
  }
  
  return true;
}

void BlueNRGGattClient::terminateServiceDiscovery(void)
{
  _currentState = GATT_DISCOVERY_TERMINATED;
  
  if (terminationCallback) {
    terminationCallback(_connectionHandle);
  }
}

void BlueNRGGattClient::charReadCB(Gap::Handle_t connHandle,
                                   uint8_t event_data_length,
                                   uint8_t* attribute_value)
{
  readCBParams.connHandle = connHandle;
  readCBParams.offset = 0;
  readCBParams.len = event_data_length;
  readCBParams.data = attribute_value;

  BlueNRGGattClient::getInstance().processReadResponse(&readCBParams);
}

ble_error_t BlueNRGGattClient::read(Gap::Handle_t connHandle, GattAttribute::Handle_t attributeHandle, uint16_t offset) const
{
  /* avoid compiler warnings about unused variables */
  (void)offset;

  tBleStatus ret;
  
  BlueNRGGattClient *gattc = const_cast<BlueNRGGattClient*>(this);
  
  gattc->_currentState = GATT_READ_CHAR;
  
  // Save the attribute_handle not provided by evt_att_read_resp    
  gattc->readCBParams.handle = attributeHandle;
  
  // We need to wait for a while before starting a read due to
  // BlueNRG process queue handling
  Clock_Wait(100); //(?)

  ret = aci_gatt_read_charac_val(connHandle, attributeHandle);
  
  if(ret == BLE_STATUS_SUCCESS) {
    return BLE_ERROR_NONE;
  }
  switch (ret) {
  case BLE_STATUS_BUSY:
    return BLE_STACK_BUSY;
  default:
    return BLE_ERROR_INVALID_STATE;
  }
}

void BlueNRGGattClient::charWritePrepareCB(Gap::Handle_t connHandle,
                                           uint8_t event_data_length,
                                           uint16_t attribute_handle,
                                           uint16_t offset,
                                           uint8_t *part_attr_value)
{
  /* avoid compiler warnings about unused variables */
  (void)connHandle;

  // Update the write response params
  writeCBParams.handle = attribute_handle;
  writeCBParams.offset = offset;
  writeCBParams.len = event_data_length-4; //(?)
  writeCBParams.data = part_attr_value;

  BlueNRGGattClient::getInstance().processWriteResponse(&writeCBParams);
}

void BlueNRGGattClient::charWriteExecCB(Gap::Handle_t connHandle,
                                        uint8_t event_data_length)
{
  /* avoid compiler warnings about unused variables */
  (void)event_data_length;

  writeCBParams.connHandle = connHandle;
  
  BlueNRGGattClient::getInstance().processWriteResponse(&writeCBParams);
}

ble_error_t BlueNRGGattClient::write(GattClient::WriteOp_t    cmd,
                                     Gap::Handle_t            connHandle,
                                     GattAttribute::Handle_t  attributeHandle,
                                     size_t                   length,
                                     const uint8_t           *value) const
{
  /* avoid compiler warnings about unused variables */
  (void)cmd;

  tBleStatus ret;
  
  BlueNRGGattClient *gattc = const_cast<BlueNRGGattClient*>(this);
    
  gattc->_currentState = GATT_WRITE_CHAR;
  
  // We save the write response params (used by the callback) because
  // when the aci_gatt_write_charac_value() is used the only event received is the EVT_BLUE_GATT_PROCEDURE_COMPLETE
  gattc->writeCBParams.connHandle = connHandle;
  gattc->writeCBParams.writeOp = GattWriteCallbackParams::OP_WRITE_CMD;
  gattc->writeCBParams.handle = attributeHandle;
  gattc->writeCBParams.offset = 0;
  gattc->writeCBParams.len = length;
  gattc->writeCBParams.data = value;
  
  ret = aci_gatt_write_charac_value(connHandle, attributeHandle, length, const_cast<uint8_t *>(value));
  //ret = aci_gatt_write_charac_reliable(connHandle, attributeHandle, 0, length, const_cast<uint8_t *>(value));
  
  if (ret == BLE_STATUS_SUCCESS) {
    return BLE_ERROR_NONE;
  }
  switch (ret) {
  case BLE_STATUS_BUSY:
    return BLE_STACK_BUSY;
  default:
    return BLE_ERROR_INVALID_STATE;
  }

}
