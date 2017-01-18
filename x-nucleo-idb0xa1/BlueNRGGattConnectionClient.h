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
  * @file    BlueNRGGattConnectionClient.cpp 
  * @author  STMicroelectronics
  * @brief   Header file for BlueNRGGattConnectionClient Class
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
  * <h2><center>&copy; COPYRIGHT 2015 STMicroelectronics</center></h2>
  */
 
#ifndef __BLUENRG_GATT_CONNECTION_CLIENT_H__
#define __BLUENRG_GATT_CONNECTION_CLIENT_H__

#ifdef YOTTA_CFG_MBED_OS
    #include "mbed-drivers/mbed.h"
#else
    #include "mbed.h"
#endif 
#include "ble/blecommon.h"
#include "btle.h"
#include "ble/GattClient.h"
#include "ble/DiscoveredService.h"
#include "ble/CharacteristicDescriptorDiscovery.h"
#include "BlueNRGDiscoveredCharacteristic.h"

using namespace std;

#define BLE_TOTAL_DISCOVERED_SERVICES 10
#define BLE_TOTAL_DISCOVERED_CHARS 10

class BlueNRGGattConnectionClient
{
public:
    
    enum {
      GATT_IDLE,
      GATT_SERVICE_DISCOVERY,
      GATT_CHAR_DESC_DISCOVERY,
      //GATT_CHARS_DISCOVERY_COMPLETE,
      //GATT_DISCOVERY_TERMINATED,
      GATT_READ_CHAR,
      GATT_WRITE_CHAR
    };
    
    /* Functions that must be implemented from GattClient */
    ble_error_t launchServiceDiscovery(ServiceDiscovery::ServiceCallback_t         sc                           = NULL,
                                       ServiceDiscovery::CharacteristicCallback_t  cc                           = NULL,
                                       const UUID                                 &matchingServiceUUID          = UUID::ShortUUIDBytes_t(BLE_UUID_UNKNOWN),
                                       const UUID                                 &matchingCharacteristicUUIDIn = UUID::ShortUUIDBytes_t(BLE_UUID_UNKNOWN));

    ble_error_t discoverServices(ServiceDiscovery::ServiceCallback_t  callback,
                                         const UUID                          &matchingServiceUUID = UUID::ShortUUIDBytes_t(BLE_UUID_UNKNOWN));

    ble_error_t discoverServices(ServiceDiscovery::ServiceCallback_t  callback,
                                         GattAttribute::Handle_t              startHandle,
                                         GattAttribute::Handle_t              endHandle);

    bool isServiceDiscoveryActive(void) const;
    void terminateServiceDiscovery(void);
    void onServiceDiscoveryTermination(ServiceDiscovery::TerminationCallback_t callback) {
      terminationCallback = callback;
    }
    ble_error_t read(GattAttribute::Handle_t attributeHandle, uint16_t offset) const;
    ble_error_t write(GattClient::WriteOp_t    cmd,
                              GattAttribute::Handle_t  attributeHandle,
                              size_t                   length,
                              const uint8_t           *value) const;
    ble_error_t discoverCharacteristicDescriptors(
        const DiscoveredCharacteristic& characteristic,
        const CharacteristicDescriptorDiscovery::DiscoveryCallback_t& discoveryCallback,
        const CharacteristicDescriptorDiscovery::TerminationCallback_t& terminationCallback);

    ble_error_t reset(void);

    void gattProcedureCompleteCB(uint8_t error_code);

    void primaryServicesCB(uint8_t event_data_length,
                           uint8_t attribute_data_length,
                           uint8_t *attribute_data_list);
    
    void primaryServiceCB(uint8_t event_data_length,
                          uint8_t *handles_info_list);
    
    ble_error_t findServiceChars(void);
    
    void serviceCharsCB(uint8_t event_data_length,
                        uint8_t handle_value_pair_length,
                        uint8_t *handle_value_pair);
    
    void serviceCharByUUIDCB(uint8_t event_data_length,
                             uint16_t attr_handle,
                             uint8_t *attr_value);

    void discAllCharacDescCB(uint8_t event_data_length,
                             uint8_t format,
                             uint8_t *handle_uuid_pair);

    void charReadCB(uint8_t event_data_length,
                    uint8_t* attribute_value);

    void charWritePrepareCB(uint8_t event_data_length,
                            uint16_t attribute_handle,
                            uint16_t offset,
                            uint8_t *part_attr_value);
    
    void charWriteExecCB(uint8_t event_data_length);

protected:

    BlueNRGGattConnectionClient(BlueNRGGattClient *gattClient, Gap::Handle_t connectionHandle) {
     _gattClient = gattClient;
     _connectionHandle = connectionHandle;

     _currentState = GATT_IDLE;
     _matchingServiceUUID = BLE_UUID_UNKNOWN;
     _matchingCharacteristicUUIDIn = BLE_UUID_UNKNOWN;

//     printf("BlueNRGGattConnectionClient construtor: connHandle=%d\n\r", connectionHandle);

    }

    ServiceDiscovery::ServiceCallback_t  serviceDiscoveryCallback;
    ServiceDiscovery::CharacteristicCallback_t characteristicDiscoveryCallback;
    ServiceDiscovery::TerminationCallback_t terminationCallback;
    CharacteristicDescriptorDiscovery::DiscoveryCallback_t charDescDiscoveryCallback;
    CharacteristicDescriptorDiscovery::TerminationCallback_t charDescTerminationCallback;

private:

  BlueNRGGattConnectionClient(BlueNRGGattConnectionClient const &);
  void operator=(BlueNRGGattConnectionClient const &);
  ~BlueNRGGattConnectionClient() {};

  BlueNRGGattClient *_gattClient;

  Gap::Handle_t _connectionHandle;
  DiscoveredService discoveredService[BLE_TOTAL_DISCOVERED_SERVICES];
  BlueNRGDiscoveredCharacteristic discoveredChar[BLE_TOTAL_DISCOVERED_CHARS];
  
  GattReadCallbackParams readCBParams;
  GattWriteCallbackParams writeCBParams;

  // The char for which the descriptor discovery has been launched  
  DiscoveredCharacteristic _characteristic;

  UUID _matchingServiceUUID;
  UUID _matchingCharacteristicUUIDIn;
  uint8_t _currentState;
  uint8_t _numServices, _servIndex;
  uint8_t _numChars;
  uint8_t _numCharDesc;
  
  friend class BlueNRGGattClient;
};

#endif /* __BLUENRG_GATT_CONNECTION_CLIENT_H__ */
