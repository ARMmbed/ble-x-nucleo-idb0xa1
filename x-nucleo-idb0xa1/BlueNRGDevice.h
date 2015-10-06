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
  * @file    BlueNRGDevice.h 
  * @author  STMicroelectronics
  * @brief   Header file for BLEDeviceInstanceBase based BlueNRGDevice
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
  
#ifndef __BLUENRG_DEVICE_H__
#define __BLUENRG_DEVICE_H__

#define BLUENRG
#define DEBUG_BLUENRG_USER

#include "btle.h"

#include "mbed.h"
#include "blecommon.h"
#include "BLE.h"
#include "BlueNRGGap.h"
#include "BlueNRGGattServer.h"


class BlueNRGDevice : public BLEInstanceBase
{

public:
    BlueNRGDevice(PinName mosi, PinName miso, PinName sck, PinName cs, PinName rst, PinName irq);
    virtual ~BlueNRGDevice(void);

    virtual ble_error_t init(void);
    virtual ble_error_t shutdown(void);   
    virtual const char *getVersion(void);
    virtual Gap&        getGap();
    virtual const Gap&  getGap() const;
    virtual GattServer& getGattServer();
    virtual const GattServer& getGattServer() const;
    virtual void        waitForEvent(void);
    
    virtual GattClient& getGattClient() {
        return *gattClient;
    }

    virtual SecurityManager& getSecurityManager() {
        return *sm;
    }

    virtual const SecurityManager& getSecurityManager() const {
        return *sm;
    }
    ble_error_t reset(void);
    bool getIsInitialized(void);

    bool dataPresent();
    int32_t spiRead(uint8_t *buffer, uint8_t buff_size);
    int32_t spiWrite(uint8_t* data1, uint8_t* data2, uint8_t Nb_bytes1, uint8_t Nb_bytes2);
    void disable_irq();
    void enable_irq();
    
private:
    bool isInitialized;

    SPI         spi_;
    DigitalOut  nCS_;
    DigitalOut  rst_;
    InterruptIn irq_;

    //FIXME: TBI (by now just placeholders to let build
    /*** betzw: placeholders ***/
    GattClient *gattClient;
    SecurityManager *sm;
};

#endif
