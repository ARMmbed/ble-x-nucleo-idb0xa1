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

#include "BlueNRGDiscoveredCharacteristic.h"
#include "BlueNRGGattClient.h"

void BlueNRGDiscoveredCharacteristic::setup(BlueNRGGattClient *gattcIn,
                                            Gap::Handle_t     connectionHandleIn,
                                            DiscoveredCharacteristic::Properties_t    propsIn,
                                            GattAttribute::Handle_t  declHandleIn,
                                            GattAttribute::Handle_t  valueHandleIn,
                                            GattAttribute::Handle_t  lastHandleIn)
{
    gattc       = gattcIn;
    connHandle  = connectionHandleIn;
    declHandle  = declHandleIn;
    valueHandle = valueHandleIn;
    lastHandle  = lastHandleIn;

    props._broadcast       = propsIn.broadcast();
    props._read            = propsIn.read();
    props._writeWoResp     = propsIn.writeWoResp();
    props._write           = propsIn.write();
    props._notify          = propsIn.notify();
    props._indicate        = propsIn.indicate();
    props._authSignedWrite = propsIn.authSignedWrite();
}

void BlueNRGDiscoveredCharacteristic::setup(BlueNRGGattClient         *gattcIn,
                                            Gap::Handle_t            connectionHandleIn,
                                            UUID   uuidIn,
                                            DiscoveredCharacteristic::Properties_t    propsIn,
                                            GattAttribute::Handle_t  declHandleIn,
                                            GattAttribute::Handle_t  valueHandleIn,
                                            GattAttribute::Handle_t  lastHandleIn)
{
    gattc       = gattcIn;
    connHandle  = connectionHandleIn;
    uuid        = uuidIn;
    declHandle  = declHandleIn;
    valueHandle = valueHandleIn;
    lastHandle  = lastHandleIn;

    props._broadcast       = propsIn.broadcast();
    props._read            = propsIn.read();
    props._writeWoResp     = propsIn.writeWoResp();
    props._write           = propsIn.write();
    props._notify          = propsIn.notify();
    props._indicate        = propsIn.indicate();
    props._authSignedWrite = propsIn.authSignedWrite();
}

 void BlueNRGDiscoveredCharacteristic::setLastHandle(GattAttribute::Handle_t  lastHandleIn) {
     lastHandle = lastHandleIn;
 }
