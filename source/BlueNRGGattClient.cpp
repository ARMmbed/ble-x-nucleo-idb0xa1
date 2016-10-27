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
#ifdef YOTTA_CFG_MBED_OS
    #include "mbed-drivers/mbed.h"
#else
    #include "mbed.h"
#endif 
#include "BlueNRGGap.h"
#include "ble_utils.h"
#include "ble_debug.h"

#include <new>
#include <assert.h>

ble_error_t BlueNRGGattClient::createGattConnectionClient(Gap::Handle_t connectionHandle)
{
  if(MAX_ACTIVE_CONNECTIONS <= _numConnections) {
    return BLE_ERROR_OPERATION_NOT_PERMITTED;
  }

  for(uint8_t i = 0; i < MAX_ACTIVE_CONNECTIONS; i++) {

    if(_connectionPool[i] == NULL) {
      BlueNRGGattConnectionClient *gattConnectionClient = new(std::nothrow) BlueNRGGattConnectionClient(this, connectionHandle);

      if (gattConnectionClient == NULL) {
        return BLE_ERROR_NO_MEM;
      }

      _connectionPool[i] = gattConnectionClient;
      _numConnections++;

      break;
    }
  }
  PRINTF("createGattConnectionClient: succesfully added new gattConnectionClient (_numConnections=%d)\r\n", _numConnections);

  return BLE_ERROR_NONE;
}

ble_error_t BlueNRGGattClient::removeGattConnectionClient(Gap::Handle_t connectionHandle)
{
  BlueNRGGattConnectionClient *gattConnectionClient = getGattConnectionClient(connectionHandle);

  if(gattConnectionClient != NULL) {
    delete gattConnectionClient;
    gattConnectionClient = NULL;

    _numConnections--;

  } else {
    return BLE_ERROR_INTERNAL_STACK_FAILURE;
  }

  PRINTF("removeGattConnectionClient: succesfully removed gattConnectionClient (_numConnections=%d)\r\n", _numConnections);

  return BLE_ERROR_NONE;
}

BlueNRGGattConnectionClient * BlueNRGGattClient::getGattConnectionClient(Gap::Handle_t connectionHandle) {
  PRINTF("getGattConnectionClient\r\n");

  for (uint8_t i = 0; i < MAX_ACTIVE_CONNECTIONS; i++) {
    PRINTF("getGattConnectionClient: _connectionPool[i]->_connectionHandle=%d\r\n",_connectionPool[i]->_connectionHandle);

    if(_connectionPool[i]->_connectionHandle == connectionHandle) {
      PRINTF("getGattConnectionClient: Found gattConnectionClient\r\n");
      return _connectionPool[i];
    }
  }

  return NULL;
}

void BlueNRGGattClient::gattProcedureCompleteCB(Gap::Handle_t connectionHandle, uint8_t error_code) {

  BlueNRGGattConnectionClient *gattConnectionClient = getGattConnectionClient(connectionHandle);

  assert(gattConnectionClient != NULL);

  gattConnectionClient->gattProcedureCompleteCB(error_code);
}

void BlueNRGGattClient::primaryServicesCB(Gap::Handle_t connectionHandle,
                                          uint8_t event_data_length,
                                          uint8_t attribute_data_length,
                                          uint8_t *attribute_data_list) {

  BlueNRGGattConnectionClient *gattConnectionClient = getGattConnectionClient(connectionHandle);

  assert(gattConnectionClient != NULL);

  gattConnectionClient->primaryServicesCB(event_data_length,
                                          attribute_data_length,
                                          attribute_data_list);
}
    
void BlueNRGGattClient::primaryServiceCB(Gap::Handle_t connectionHandle,
                                         uint8_t event_data_length,
                                         uint8_t *handles_info_list) {

  BlueNRGGattConnectionClient *gattConnectionClient = getGattConnectionClient(connectionHandle);

  assert(gattConnectionClient != NULL);

  gattConnectionClient->primaryServiceCB(event_data_length,
                                         handles_info_list);
}
    
ble_error_t BlueNRGGattClient::findServiceChars(Gap::Handle_t connectionHandle) {

  BlueNRGGattConnectionClient *gattConnectionClient = getGattConnectionClient(connectionHandle);

  if(gattConnectionClient != NULL) {
    return gattConnectionClient->findServiceChars();
  } else {
    return BLE_ERROR_INTERNAL_STACK_FAILURE;
  }
}
    
void BlueNRGGattClient::serviceCharsCB(Gap::Handle_t connectionHandle,
                                       uint8_t event_data_length,
                                       uint8_t handle_value_pair_length,
                                       uint8_t *handle_value_pair) {

  BlueNRGGattConnectionClient *gattConnectionClient = getGattConnectionClient(connectionHandle);

  assert(gattConnectionClient != NULL);

  gattConnectionClient->serviceCharsCB(event_data_length,
                                       handle_value_pair_length,
                                       handle_value_pair);
}
    
void BlueNRGGattClient::serviceCharByUUIDCB(Gap::Handle_t connectionHandle,
                                            uint8_t event_data_length,
                                            uint16_t attr_handle,
                                            uint8_t *attr_value) {

  BlueNRGGattConnectionClient *gattConnectionClient = getGattConnectionClient(connectionHandle);

  assert(gattConnectionClient != NULL);

  gattConnectionClient->serviceCharByUUIDCB(event_data_length,
                                            attr_handle,
                                            attr_value);
}

void BlueNRGGattClient::discAllCharacDescCB(Gap::Handle_t connHandle,
                                            uint8_t event_data_length,
                                            uint8_t format,
                                            uint8_t *handle_uuid_pair) {

  BlueNRGGattConnectionClient *gattConnectionClient = getGattConnectionClient(connHandle);

  assert(gattConnectionClient != NULL);

  gattConnectionClient->discAllCharacDescCB(event_data_length,
                                            format,
                                            handle_uuid_pair);
}

void BlueNRGGattClient::charReadCB(Gap::Handle_t connHandle,
                                   uint8_t event_data_length,
                                   uint8_t* attribute_value) {

  BlueNRGGattConnectionClient *gattConnectionClient = getGattConnectionClient(connHandle);

  assert(gattConnectionClient != NULL);

  gattConnectionClient->charReadCB(event_data_length,
                                   attribute_value);
}

void BlueNRGGattClient::charWritePrepareCB(Gap::Handle_t connHandle,
                                           uint8_t event_data_length,
                                           uint16_t attribute_handle,
                                           uint16_t offset,
                                           uint8_t *part_attr_value) {

  BlueNRGGattConnectionClient *gattConnectionClient = getGattConnectionClient(connHandle);

  assert(gattConnectionClient != NULL);

  gattConnectionClient->charWritePrepareCB(event_data_length,
                                           attribute_handle,
                                           offset,
                                           part_attr_value);
}
    
void BlueNRGGattClient::charWriteExecCB(Gap::Handle_t connHandle,
                                        uint8_t event_data_length) {

  BlueNRGGattConnectionClient *gattConnectionClient = getGattConnectionClient(connHandle);

  assert(gattConnectionClient != NULL);

  gattConnectionClient->charWriteExecCB(event_data_length);
}

ble_error_t BlueNRGGattClient::launchServiceDiscovery(Gap::Handle_t                               connectionHandle,
                                                      ServiceDiscovery::ServiceCallback_t         sc,
                                                      ServiceDiscovery::CharacteristicCallback_t  cc,
                                                      const UUID                                 &matchingServiceUUID,
                                                      const UUID                                 &matchingCharacteristicUUIDIn)
{
  PRINTF("BlueNRGGattClient launchServiceDiscovery\n\r");

  BlueNRGGattConnectionClient *gattConnectionClient = getGattConnectionClient(connectionHandle);

  if(gattConnectionClient != NULL) {
    gattConnectionClient->onServiceDiscoveryTermination(terminationCallback);

    return gattConnectionClient->launchServiceDiscovery(sc, cc, matchingServiceUUID, matchingCharacteristicUUIDIn);

  } else {
    return BLE_ERROR_INTERNAL_STACK_FAILURE;
  }
}

ble_error_t BlueNRGGattClient::discoverServices(Gap::Handle_t                        connectionHandle,
                                                ServiceDiscovery::ServiceCallback_t  callback,
                                                const UUID                          &matchingServiceUUID)
{
  BlueNRGGattConnectionClient *gattConnectionClient = getGattConnectionClient(connectionHandle);

  if(gattConnectionClient != NULL) {

    return gattConnectionClient->discoverServices(callback, matchingServiceUUID);

  } else {
    return BLE_ERROR_INTERNAL_STACK_FAILURE;
  }
}

ble_error_t BlueNRGGattClient::discoverServices(Gap::Handle_t                        connectionHandle,
                                                ServiceDiscovery::ServiceCallback_t  callback,
                                                GattAttribute::Handle_t              startHandle,
                                                GattAttribute::Handle_t              endHandle)
{
  BlueNRGGattConnectionClient *gattConnectionClient = getGattConnectionClient(connectionHandle);

  if(gattConnectionClient != NULL) {

    return gattConnectionClient->discoverServices(callback, startHandle, endHandle);

  } else {
    return BLE_ERROR_INTERNAL_STACK_FAILURE;
  }
}

bool BlueNRGGattClient::isServiceDiscoveryActive(void) const
{
  bool isSDActive = false;

  for (uint8_t i = 0; i < _numConnections; i++) {
    isSDActive |= _connectionPool[i]->isServiceDiscoveryActive();
  }

  return isSDActive;
}

void BlueNRGGattClient::terminateServiceDiscovery(void)
{
  for (uint8_t i = 0; i < _numConnections; i++) {
    _connectionPool[i]->terminateServiceDiscovery();
  }
}

ble_error_t BlueNRGGattClient::read(Gap::Handle_t connHandle, GattAttribute::Handle_t attributeHandle, uint16_t offset) const
{
  BlueNRGGattConnectionClient *gattConnectionClient = const_cast<BlueNRGGattClient*>(this)->getGattConnectionClient(connHandle);

  if(gattConnectionClient != NULL) {

    return gattConnectionClient->read(attributeHandle, offset);

  } else {
    return BLE_ERROR_INTERNAL_STACK_FAILURE;
  }
}

ble_error_t BlueNRGGattClient::write(GattClient::WriteOp_t    cmd,
                                     Gap::Handle_t            connHandle,
                                     GattAttribute::Handle_t  attributeHandle,
                                     size_t                   length,
                                     const uint8_t           *value) const
{
  BlueNRGGattConnectionClient *gattConnectionClient = const_cast<BlueNRGGattClient*>(this)->getGattConnectionClient(connHandle);

  if(gattConnectionClient != NULL) {

    return gattConnectionClient->write(cmd, attributeHandle, length, value);

  } else {
    return BLE_ERROR_INTERNAL_STACK_FAILURE;
  }
}

ble_error_t BlueNRGGattClient::discoverCharacteristicDescriptors(
        const DiscoveredCharacteristic& characteristic,
        const CharacteristicDescriptorDiscovery::DiscoveryCallback_t& discoveryCallback,
        const CharacteristicDescriptorDiscovery::TerminationCallback_t& terminationCallback)
{
  BlueNRGGattConnectionClient *gattConnectionClient = getGattConnectionClient(characteristic.getConnectionHandle());

  if(gattConnectionClient != NULL) {

    return gattConnectionClient->discoverCharacteristicDescriptors(characteristic, discoveryCallback, terminationCallback);

  } else {
    return BLE_ERROR_INTERNAL_STACK_FAILURE;
  }
}

/**************************************************************************/
/*!
    @brief  Clear BlueNRGGattClient's state.

    @returns    ble_error_t

    @retval     BLE_ERROR_NONE
                Everything executed properly
*/
/**************************************************************************/
ble_error_t BlueNRGGattClient::reset(void)
{
  for (uint8_t i = 0; i < MAX_ACTIVE_CONNECTIONS; i++) {
    if(_connectionPool[i] != NULL) {
      _connectionPool[i]->reset();

      delete _connectionPool[i];
      _connectionPool[i] = NULL;

      _numConnections--;
    }
  }

  return BLE_ERROR_NONE;
}

