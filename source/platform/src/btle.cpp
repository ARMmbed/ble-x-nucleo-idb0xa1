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
  * @file    btle.cpp 
  * @author  STMicroelectronics
  * @brief   Implementation BlueNRG Init and helper functions.
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


#include "btle.h"
#include "ble/Gap.h"
#include "ble/GapEvents.h"
#include "BlueNRGGap.h"
#include "BlueNRGGattServer.h"
#include "Utils.h"

#ifdef __cplusplus
extern "C" {
#endif


/* C File Includes ------------------------------------------------------------------*/
// ANDREA: Updated includes and changed some types (e.g., tHalUint8 --> uint8_t)
#include <stdio.h>
#include <string.h>
#include "hci.h"
#include "hci_const.h"
#include "bluenrg_aci.h"
#include "bluenrg_hal_aci.h"
#include "bluenrg_gap.h"
#include "bluenrg_utils.h"

#include "hal_types.h"
#include "hal.h"
#include "gp_timer.h"
#include "osal.h"
#include "sm.h"
#include "debug_platform.h"

#ifdef __cplusplus
}
#endif

#define IDB04A1 0
#define IDB05A1 1

// static void btle_handler(/*ble_evt_t * p_ble_evt*/);
void HCI_Input(tHciDataPacket * hciReadPacket);

//#define BDADDR_SIZE 6
//tHalUint8 bdaddr[BDADDR_SIZE]= {0xaa, 0x00, 0x00, 0xE1, 0x80, 0x02};

uint16_t g_gap_service_handle = 0;
uint16_t g_appearance_char_handle = 0;
uint16_t g_device_name_char_handle = 0;

/* Private variables ---------------------------------------------------------*/
volatile uint8_t set_connectable = 1;
// ANDREA
uint8_t bnrg_expansion_board = IDB04A1; /* at startup, suppose the X-NUCLEO-IDB04A1 is used */

Gap::Address_t bleAddr;
Gap::AddressType_t addr_type = Gap::ADDR_TYPE_PUBLIC;

/**************************************************************************/
/*!
    @brief  Initialises BTLE and the underlying HW/Device
    @param  isSetAddress boolean if address has been set
    @param  mosi MOSI Pin
    @param  miso MISO Pin
    @param  sclk clock Pin
    @returns void
*/
/**************************************************************************/
void btle_init(bool isSetAddress)
{
    DEBUG("btle_init>>\n\r"); 
    
    int ret;
    uint8_t  hwVersion;
    uint16_t fwVersion;
    uint16_t service_handle, dev_name_char_handle, appearance_char_handle;

    /* Delay needed only to be able to acces the JTAG interface after reset
    if it will be disabled later. */
    Clock_Wait(500);

    /* Initialize the BlueNRG HCI */
    HCI_Init();

    /* Reset BlueNRG SPI interface */
    BlueNRG_RST();

    /* get the BlueNRG HW and FW versions */
    getBlueNRGVersion(&hwVersion, &fwVersion);

    /* 
     * Reset BlueNRG again otherwise we won't
     * be able to change its MAC address.
     * aci_hal_write_config_data() must be the first
     * command after reset otherwise it will fail.
     */
    BlueNRG_RST();

    if (hwVersion > 0x30) { /* X-NUCLEO-IDB05A1 expansion board is used */
        bnrg_expansion_board = IDB05A1;
    }

    /* The Nucleo board must be configured as SERVER */
    //check if isSetAddress is set than set address.
    // ANDREA
    if(isSetAddress)
    {
        BlueNRGGap::getInstance().getAddress(&addr_type, bleAddr);
        
        Gap::Address_t bdaddr;
        Osal_MemCpy(bdaddr, bleAddr, BDADDR_SIZE);

        ret = aci_hal_write_config_data(CONFIG_DATA_PUBADDR_OFFSET,
                                        CONFIG_DATA_PUBADDR_LEN,
                                        bdaddr);
    } else {
        
        const Gap::Address_t BLE_address_BE = {0xFD,0x00,0x25,0xAA,0x02,0x04};
        BlueNRGGap::getInstance().setAddress(Gap::ADDR_TYPE_PUBLIC, BLE_address_BE);
        
        ret = aci_hal_write_config_data(CONFIG_DATA_PUBADDR_OFFSET,
                                        CONFIG_DATA_PUBADDR_LEN,
                                        BLE_address_BE);
    }
    
    ret = aci_gatt_init();
    if(ret){
        PRINTF("GATT_Init failed.\n");
    }
    //GAP is always in PERIPHERAL _ROLE as mbed does not support Master role at the moment
    if (bnrg_expansion_board == IDB05A1) {
        ret = aci_gap_init_IDB05A1(GAP_PERIPHERAL_ROLE_IDB05A1, 0, 0x07, &service_handle, &dev_name_char_handle, &appearance_char_handle);
    } else {
        ret = aci_gap_init_IDB04A1(GAP_PERIPHERAL_ROLE_IDB04A1, &service_handle, &dev_name_char_handle, &appearance_char_handle);
    }
    
    if(ret != BLE_STATUS_SUCCESS){
        PRINTF("GAP_Init failed.\n");
    }

    //ANDREA -- FIXME: Security and passkey set by default    
    ret = aci_gap_set_auth_requirement(MITM_PROTECTION_REQUIRED,
                                       OOB_AUTH_DATA_ABSENT,
                                       NULL,
                                       7,
                                       16,
                                       USE_FIXED_PIN_FOR_PAIRING,
                                       123456,
                                       BONDING);
    if (ret == BLE_STATUS_SUCCESS) {
        DEBUG("Auth Req set successfully.\n");
    }
    
    aci_hal_set_tx_power_level(1,4);
    
    g_gap_service_handle = service_handle;
    g_appearance_char_handle = appearance_char_handle;
    g_device_name_char_handle = dev_name_char_handle; 
    //Device Name is set from Accumulate Adv Data Payload or through setDeviceName API  
    /*ret = aci_gatt_update_char_value(service_handle, dev_name_char_handle, 0,
                            strlen(name), (tHalUint8 *)name);*/
    
    return;
}

void User_Process()
{
    if(set_connectable){
        setConnectable();
        set_connectable = FALSE;
    }
}

void setConnectable(void)
{  
    tBleStatus ret;

    const char local_name[] = {AD_TYPE_COMPLETE_LOCAL_NAME,'B','l','u','e','N','R','G'};

    /* disable scan response */
    hci_le_set_scan_resp_data(0,NULL);

    //int t = BlueNRGGap::getInstance()::ADV_IND;// advType;
    
    ret = aci_gap_set_discoverable(BlueNRGGap::getInstance().ADV_IND, 0, 0, PUBLIC_ADDR, NO_WHITE_LIST_USE,
    sizeof(local_name), local_name, 0, NULL, 0, 0);
    if (ret != BLE_STATUS_SUCCESS) {
        DEBUG("Error while setting discoverable mode (%d)\n", ret);    
    }

}

/**************************************************************************/
/*!
    @brief  Not Used

    @param[in]  void
    
    @returns
*/
/**************************************************************************/
/*
static void btle_handler()
{

}
*/

/*!
    @brief  Not Used

    @param[in]  void
    
    @returns
*/
void SPI_Poll(void)
{
    //HAL_GPIO_EXTI_Callback_Poll(BNRG_SPI_EXTI_PIN);
    return;
}
   
void Attribute_Modified_CB(uint16_t attr_handle, uint8_t data_length, uint8_t *att_data, uint8_t offset)
{        
    //Extract the GattCharacteristic from p_characteristics[] and find the properties mask
    GattCharacteristic *p_char = BlueNRGGattServer::getInstance().getCharacteristicFromHandle(attr_handle);
    if(p_char!=NULL) {
        GattAttribute::Handle_t charHandle = p_char->getValueAttribute().getHandle();
        BlueNRGGattServer::HandleEnum_t currentHandle = BlueNRGGattServer::CHAR_HANDLE;
        DEBUG("CharHandle %d, length: %d, Data: %d\n\r", charHandle, data_length, (uint16_t)att_data[0]);
        DEBUG("getProperties 0x%x\n\r",p_char->getProperties());
        if(attr_handle == charHandle+CHAR_VALUE_OFFSET) {
            currentHandle = BlueNRGGattServer::CHAR_VALUE_HANDLE;
        }

        if(attr_handle == charHandle+CHAR_DESC_OFFSET) {
            currentHandle = BlueNRGGattServer::CHAR_DESC_HANDLE;
        }
        DEBUG("currentHandle %d\n\r", currentHandle);
        if((p_char->getProperties() & 
            (GattCharacteristic::BLE_GATT_CHAR_PROPERTIES_NOTIFY | GattCharacteristic::BLE_GATT_CHAR_PROPERTIES_INDICATE)) &&
            currentHandle == BlueNRGGattServer::CHAR_DESC_HANDLE) {

            DEBUG("*****NOTIFICATION CASE\n\r");
            //Now Check if data written in Enable or Disable
            if((uint16_t)att_data[0]==1) {
                //DEBUG("Notify ENABLED\n\r"); 
                BlueNRGGattServer::getInstance().HCIEvent(GattServerEvents::GATT_EVENT_UPDATES_ENABLED, p_char->getValueAttribute().getHandle());
            } else {
                //DEBUG("Notify DISABLED\n\r"); 
                BlueNRGGattServer::getInstance().HCIEvent(GattServerEvents::GATT_EVENT_UPDATES_DISABLED, p_char->getValueAttribute().getHandle());
            }
        }
                    
        //Check is attr handle property is WRITEABLE, if yes, generate GATT_EVENT_DATA_WRITTEN Event
        if((p_char->getProperties() &
            (GattCharacteristic::BLE_GATT_CHAR_PROPERTIES_WRITE_WITHOUT_RESPONSE | GattCharacteristic::BLE_GATT_CHAR_PROPERTIES_WRITE)) &&
            currentHandle == BlueNRGGattServer::CHAR_VALUE_HANDLE) {
                    
            DEBUG("*****WRITE CASE\n\r");
                   
            GattWriteCallbackParams writeParams;
            writeParams.handle = p_char->getValueAttribute().getHandle();
            writeParams.writeOp = GattWriteCallbackParams::OP_WRITE_REQ;//Where to find this property in BLUENRG?
            writeParams.len = data_length;
            writeParams.data = att_data;                                                                                    
            if (bnrg_expansion_board == IDB05A1) {
                writeParams.offset = offset;
            }
            BlueNRGGattServer::getInstance().HCIDataWrittenEvent(&writeParams);

            //BlueNRGGattServer::getInstance().handleEvent(GattServerEvents::GATT_EVENT_DATA_WRITTEN, evt->attr_handle);
            //Write the actual Data to the Attr Handle? (uint8_1[])att_data contains the data
            if ((p_char->getValueAttribute().getValuePtr() != NULL) && (p_char->getValueAttribute().getInitialLength() > 0)) {
                BlueNRGGattServer::getInstance().write(p_char->getValueAttribute().getHandle(), (uint8_t*)att_data, data_length, false);
            }
        } 
    }

}

#ifdef __cplusplus
extern "C" {
#endif

    /**************************************************************************/
    /*!
    @brief      Handle HCI Stack Event

    @param[in]  pckt
                Event Packet sent by the stack to be decoded
    
    @returns
    */
    /**************************************************************************/
    extern void HCI_Event_CB(void *pckt) {
        hci_uart_pckt *hci_pckt = (hci_uart_pckt*)pckt;
        hci_event_pckt *event_pckt = (hci_event_pckt*)hci_pckt->data;
        
        if(hci_pckt->type != HCI_EVENT_PKT)
        return;

        switch(event_pckt->evt){
            
        case EVT_DISCONN_COMPLETE:
            {
                DEBUG("EVT_DISCONN_COMPLETE\n");
                
                evt_disconn_complete *evt = (evt_disconn_complete*)event_pckt;
                
                BlueNRGGap::getInstance().processDisconnectionEvent(evt->handle, BlueNRGGap::REMOTE_USER_TERMINATED_CONNECTION);
            }
            break;
            
        case EVT_LE_META_EVENT:
            {
                DEBUG("EVT_LE_META_EVENT\n");
                
                evt_le_meta_event *evt = (evt_le_meta_event *)event_pckt->data;
                
                switch(evt->subevent){
                // ANDREA
                case EVT_LE_CONN_COMPLETE:
                    {                            
                        Gap::AddressType_t peerAddrType = Gap::ADDR_TYPE_PUBLIC;
                        DEBUG("EVT_LE_CONN_COMPLETE\n");
                        
                        evt_le_connection_complete *cc = (evt_le_connection_complete *)evt->data;
                        
                        BlueNRGGap::getInstance().setConnectionHandle(cc->handle);
                        BlueNRGGap::ConnectionParams_t connectionParams;
                        BlueNRGGap::getInstance().getPreferredConnectionParams(&connectionParams);
                        switch (cc->peer_bdaddr_type) {
                            case PUBLIC_ADDR:
                                peerAddrType = Gap::ADDR_TYPE_PUBLIC;
                                break;
                            case STATIC_RANDOM_ADDR:
                                peerAddrType = Gap::ADDR_TYPE_RANDOM_STATIC;
                                break;
                            case RESOLVABLE_PRIVATE_ADDR:
                                peerAddrType = Gap::ADDR_TYPE_RANDOM_PRIVATE_RESOLVABLE;
                                break;
                            case NON_RESOLVABLE_PRIVATE_ADDR:
                                peerAddrType = Gap::ADDR_TYPE_RANDOM_PRIVATE_NON_RESOLVABLE;
                                break;
                        }                                             
                        BlueNRGGap::getInstance().processConnectionEvent(cc->handle, Gap::PERIPHERAL, peerAddrType, cc->peer_bdaddr, addr_type, bleAddr, (const BlueNRGGap::ConnectionParams_t *)&connectionParams);                            
                    }
                    break;
                }
            }
            break;
            
        case EVT_VENDOR:
            {                
                evt_blue_aci *blue_evt = (evt_blue_aci*)event_pckt->data;
                //DEBUG("EVT_VENDOR %d\n", blue_evt->ecode);
                
                switch(blue_evt->ecode){
                           
                case EVT_BLUE_GATT_READ_PERMIT_REQ:
                    {
                        DEBUG("EVT_BLUE_GATT_READ_PERMIT_REQ_OK\n\r");
                        evt_gatt_read_permit_req *pr = (evt_gatt_read_permit_req*)blue_evt->data;
                        BlueNRGGattServer::getInstance().Read_Request_CB(pr->attr_handle-CHAR_VALUE_OFFSET);                                                
                    }
                    break;
                    
                case EVT_BLUE_GATT_ATTRIBUTE_MODIFIED:         
                    {
                        DEBUG("EVT_BLUE_GATT_ATTRIBUTE_MODIFIED\n\r");
                        /* this callback is invoked when a GATT attribute is modified
                            extract callback data and pass to suitable handler function */
                        if (bnrg_expansion_board == IDB05A1) {
                            evt_gatt_attr_modified_IDB05A1 *evt = (evt_gatt_attr_modified_IDB05A1*)blue_evt->data;
                            Attribute_Modified_CB(evt->attr_handle, evt->data_length, evt->att_data, evt->offset);
                        } else {
                            evt_gatt_attr_modified_IDB04A1 *evt = (evt_gatt_attr_modified_IDB04A1*)blue_evt->data;
                            Attribute_Modified_CB(evt->attr_handle, evt->data_length, evt->att_data, 0);
                        }                  
                    }
                    break;  
                    
                    //Any cases for Data Sent Notifications?
                case EVT_BLUE_GATT_NOTIFICATION:
                    //This is only relevant for Client Side Event
                    DEBUG("EVT_BLUE_GATT_NOTIFICATION");
                    break;
                case EVT_BLUE_GATT_INDICATION:
                    //This is only relevant for Client Side Event
                    DEBUG("EVT_BLUE_GATT_INDICATION");
                    break;   
                    
                case EVT_BLUE_GATT_PROCEDURE_COMPLETE:
                    DEBUG("EVT_BLUE_GATT_PROCEDURE_COMPLETE");
                    break;                                     
                }
            }
            break;
        }    
        
        return ;
    }


#ifdef __cplusplus
}
#endif
