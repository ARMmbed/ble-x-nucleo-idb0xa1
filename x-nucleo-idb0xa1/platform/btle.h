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


#ifndef _BTLE_H_
#define _BTLE_H_


#ifdef __cplusplus
extern "C" {
#endif

#include <stdio.h>
#include <string.h>

#include "hci.h"
#include "bluenrg_aci.h"
#include "hci_const.h"
#include "bluenrg_hal_aci.h"
#include "stm32_bluenrg_ble.h"
#include "bluenrg_gap.h"
#include "bluenrg_gatt_server.h"

extern uint16_t g_gap_service_handle;
extern uint16_t g_appearance_char_handle;
extern uint16_t g_device_name_char_handle;
extern uint16_t g_preferred_connection_parameters_char_handle;

void btleInit(void);
void SPI_Poll(void);
void User_Process(void);
void setConnectable(void);
void setVersionString(uint8_t hwVersion, uint16_t fwVersion);
const char* getVersionString(void);
tBleStatus btleStartRadioScan(uint8_t scan_type,
                              uint16_t scan_interval,
                              uint16_t scan_window,
                              uint8_t own_address_type);


extern int btle_handler_pending;
extern void btle_handler(void);

#ifdef __cplusplus
}
#endif

#endif
