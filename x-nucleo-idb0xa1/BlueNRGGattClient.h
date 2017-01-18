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
  * @file    BlueNRGGattClient.cpp 
  * @author  STMicroelectronics
  * @brief   Header file for BLE_API GattClient Class
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
 
#ifndef __BLUENRG_GATT_CLIENT_H__
#define __BLUENRG_GATT_CLIENT_H__

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
#include "BlueNRGGattConnectionClient.h"

using namespace std;

#define MAX_ACTIVE_CONNECTIONS 7

class BlueNRGGattClient : public GattClient
{
public:
    static BlueNRGGattClient &getInstance() {
        static BlueNRGGattClient m_instance;
        return m_instance;
    }

    ble_error_t createGattConnectionClient(Gap::Handle_t connectionHandle);
    ble_error_t removeGattConnectionClient(Gap::Handle_t connectionHandle, uint8_t reason);
    
    /* Functions that must be implemented from GattClient */
    virtual ble_error_t launchServiceDiscovery(Gap::Handle_t                               connectionHandle,
                                               ServiceDiscovery::ServiceCallback_t         sc                           = NULL,
                                               ServiceDiscovery::CharacteristicCallback_t  cc                           = NULL,
                                               const UUID                                 &matchingServiceUUID          = UUID::ShortUUIDBytes_t(BLE_UUID_UNKNOWN),
                                               const UUID                                 &matchingCharacteristicUUIDIn = UUID::ShortUUIDBytes_t(BLE_UUID_UNKNOWN));

    virtual ble_error_t discoverServices(Gap::Handle_t                        connectionHandle,
                                         ServiceDiscovery::ServiceCallback_t  callback,
                                         const UUID                          &matchingServiceUUID = UUID::ShortUUIDBytes_t(BLE_UUID_UNKNOWN));

    virtual ble_error_t discoverServices(Gap::Handle_t                        connectionHandle,
                                         ServiceDiscovery::ServiceCallback_t  callback,
                                         GattAttribute::Handle_t              startHandle,
                                         GattAttribute::Handle_t              endHandle);

    virtual bool isServiceDiscoveryActive(void) const;
    virtual void terminateServiceDiscovery(void);
    virtual void onServiceDiscoveryTermination(ServiceDiscovery::TerminationCallback_t callback) {
      terminationCallback = callback;
    }
    virtual ble_error_t read(Gap::Handle_t connHandle, GattAttribute::Handle_t attributeHandle, uint16_t offset) const;
    virtual ble_error_t write(GattClient::WriteOp_t    cmd,
                              Gap::Handle_t            connHandle,
                              GattAttribute::Handle_t  attributeHandle,
                              size_t                   length,
                              const uint8_t           *value) const;
    virtual ble_error_t discoverCharacteristicDescriptors(
        const DiscoveredCharacteristic& characteristic,
        const CharacteristicDescriptorDiscovery::DiscoveryCallback_t& discoveryCallback,
        const CharacteristicDescriptorDiscovery::TerminationCallback_t& terminationCallback);

    virtual ble_error_t reset(void);

    void gattProcedureCompleteCB(Gap::Handle_t connectionHandle, uint8_t error_code);

    void primaryServicesCB(Gap::Handle_t connectionHandle,
                           uint8_t event_data_length,
                           uint8_t attribute_data_length,
                           uint8_t *attribute_data_list);
    
    void primaryServiceCB(Gap::Handle_t connectionHandle,
                          uint8_t event_data_length,
                          uint8_t *handles_info_list);
    
    ble_error_t findServiceChars(Gap::Handle_t connectionHandle);
    
    void serviceCharsCB(Gap::Handle_t connectionHandle,
                        uint8_t event_data_length,
                        uint8_t handle_value_pair_length,
                        uint8_t *handle_value_pair);
    
    void serviceCharByUUIDCB(Gap::Handle_t connectionHandle,
                             uint8_t event_data_length,
                             uint16_t attr_handle,
                             uint8_t *attr_value);

    void discAllCharacDescCB(Gap::Handle_t connHandle,
                             uint8_t event_data_length,
                             uint8_t format,
                             uint8_t *handle_uuid_pair);

    void charReadCB(Gap::Handle_t connHandle,
                    uint8_t event_data_length,
                    uint8_t* attribute_value);

    void charWritePrepareCB(Gap::Handle_t connHandle,
                            uint8_t event_data_length,
                            uint16_t attribute_handle,
                            uint16_t offset,
                            uint8_t *part_attr_value);
    
    void charWriteExecCB(Gap::Handle_t connHandle,
                         uint8_t event_data_length);


protected:

    BlueNRGGattClient(): _connectionPool() {};
/*
      memset(_connectionPool, 0, sizeof(_connectionPool));
    }
*/
    ServiceDiscovery::TerminationCallback_t terminationCallback;

private:

  BlueNRGGattClient(BlueNRGGattClient const &);
  void operator=(BlueNRGGattClient const &);

  BlueNRGGattConnectionClient *_connectionPool[MAX_ACTIVE_CONNECTIONS];
  uint8_t _numConnections;

  BlueNRGGattConnectionClient * getGattConnectionClient(Gap::Handle_t connectionHandle);

};

#endif /* __BLUENRG_GATT_CLIENT_H__ */
