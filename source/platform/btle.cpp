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
#include "BlueNRGGattClient.h"
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
#include "debug.h"

#ifdef __cplusplus
}
#endif

#define IDB04A1 0
#define IDB05A1 1

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
Gap::AddressType_t addr_type = BLEProtocol::AddressType::PUBLIC;

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
void btle_init(bool isSetAddress, uint8_t role)
{
    PRINTF("btle_init>>\n\r"); 
    
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
        
        const Gap::Address_t BLE_address_BE = {0xFD,0x66,0x05,0x13,0xBE,0xBA};
        BlueNRGGap::getInstance().setAddress(BLEProtocol::AddressType::PUBLIC, BLE_address_BE);
        
        ret = aci_hal_write_config_data(CONFIG_DATA_PUBADDR_OFFSET,
                                        CONFIG_DATA_PUBADDR_LEN,
                                        BLE_address_BE);
    }
    
    ret = aci_gatt_init();
    if(ret){
        PRINTF("GATT_Init failed.\n");
    }
    if (bnrg_expansion_board == IDB05A1) {
        ret = aci_gap_init_IDB05A1(GAP_PERIPHERAL_ROLE_IDB05A1|GAP_CENTRAL_ROLE_IDB05A1, 0, 0x07, &service_handle, &dev_name_char_handle, &appearance_char_handle);
    } else {
        ret = aci_gap_init_IDB04A1(role, &service_handle, &dev_name_char_handle, &appearance_char_handle);
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
        PRINTF("Auth Req set successfully.\n");
    }
    
    aci_hal_set_tx_power_level(1,4);
    
    g_gap_service_handle = service_handle;
    g_appearance_char_handle = appearance_char_handle;
    g_device_name_char_handle = dev_name_char_handle; 
    //Device Name is set from Accumulate Adv Data Payload or through setDeviceName API  
    /*ret = aci_gatt_update_char_value(service_handle, dev_name_char_handle, 0,
                            strlen(name), (tHalUint8 *)name);*/

    // Andrea: mbedOS
#ifdef AST_FOR_MBED_OS
    minar::Scheduler::postCallback(btle_handler);
#endif
    return;
}

/**************************************************************************/
/*!
    @brief  Andrea: mbedOS

    @param[in]  void
    
    @returns
*/
/**************************************************************************/
#ifdef AST_FOR_MBED_OS
int btle_handler_pending = 0;

void btle_handler(void)
{
    btle_handler_pending = 0;
    HCI_Process();
}
#endif


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
        PRINTF("CharHandle %d, length: %d, Data: %d\n\r", charHandle, data_length, (uint16_t)att_data[0]);
        PRINTF("getProperties 0x%x\n\r",p_char->getProperties());
        if(attr_handle == charHandle+CHAR_VALUE_OFFSET) {
            currentHandle = BlueNRGGattServer::CHAR_VALUE_HANDLE;
        }

        if(attr_handle == charHandle+CHAR_DESC_OFFSET) {
            currentHandle = BlueNRGGattServer::CHAR_DESC_HANDLE;
        }
        PRINTF("currentHandle %d\n\r", currentHandle);
        if((p_char->getProperties() & 
            (GattCharacteristic::BLE_GATT_CHAR_PROPERTIES_NOTIFY | GattCharacteristic::BLE_GATT_CHAR_PROPERTIES_INDICATE)) &&
            currentHandle == BlueNRGGattServer::CHAR_DESC_HANDLE) {

            PRINTF("*****NOTIFICATION CASE\n\r");
            //Now Check if data written in Enable or Disable
            if((uint16_t)att_data[0]==1) {
                //PRINTF("Notify ENABLED\n\r"); 
                BlueNRGGattServer::getInstance().HCIEvent(GattServerEvents::GATT_EVENT_UPDATES_ENABLED, p_char->getValueAttribute().getHandle());
            } else {
                //PRINTF("Notify DISABLED\n\r"); 
                BlueNRGGattServer::getInstance().HCIEvent(GattServerEvents::GATT_EVENT_UPDATES_DISABLED, p_char->getValueAttribute().getHandle());
            }
        }
                    
        //Check is attr handle property is WRITEABLE, if yes, generate GATT_EVENT_DATA_WRITTEN Event
        if((p_char->getProperties() &
            (GattCharacteristic::BLE_GATT_CHAR_PROPERTIES_WRITE_WITHOUT_RESPONSE | GattCharacteristic::BLE_GATT_CHAR_PROPERTIES_WRITE)) &&
            currentHandle == BlueNRGGattServer::CHAR_VALUE_HANDLE) {
                    
            PRINTF("*****WRITE CASE\n\r");
                   
            GattWriteCallbackParams writeParams;
            writeParams.handle = p_char->getValueAttribute().getHandle();
            writeParams.writeOp = GattWriteCallbackParams::OP_WRITE_REQ;//Where to find this property in BLUENRG?
            writeParams.len = data_length;
            writeParams.data = att_data;                                                                                    
            if (bnrg_expansion_board == IDB05A1) {
                writeParams.offset = offset;
            } else {
                writeParams.offset = 0;
            }
            BlueNRGGattServer::getInstance().HCIDataWrittenEvent(&writeParams);

            //BlueNRGGattServer::getInstance().handleEvent(GattServerEvents::GATT_EVENT_DATA_WRITTEN, evt->attr_handle);
            //Write the actual Data to the Attr Handle? (uint8_1[])att_data contains the data
            if ((p_char->getValueAttribute().getValuePtr() != NULL) && (p_char->getValueAttribute().getLength() > 0)) {
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
                PRINTF("EVT_DISCONN_COMPLETE\n");
                
                evt_disconn_complete *evt = (evt_disconn_complete*)event_pckt;
                
                BlueNRGGap::getInstance().processDisconnectionEvent(evt->handle, BlueNRGGap::REMOTE_USER_TERMINATED_CONNECTION);
            }
            break;
            
        case EVT_LE_META_EVENT:
            {
                PRINTF("EVT_LE_META_EVENT\n");
                
                evt_le_meta_event *evt = (evt_le_meta_event *)event_pckt->data;
                
                switch(evt->subevent){
                // ANDREA
                case EVT_LE_CONN_COMPLETE:
                    {                            
                        PRINTF("EVT_LE_CONN_COMPLETE\n");
                        Gap::AddressType_t peerAddrType = BLEProtocol::AddressType::PUBLIC;
                        Gap::Role_t role;
                        
                        evt_le_connection_complete *cc = (evt_le_connection_complete *)evt->data;
                        
                        BlueNRGGap::getInstance().setConnectionHandle(cc->handle);
                        BlueNRGGap::ConnectionParams_t connectionParams;
                        BlueNRGGap::getInstance().getPreferredConnectionParams(&connectionParams);
                        switch (cc->peer_bdaddr_type) {
                            case PUBLIC_ADDR:
                                peerAddrType = BLEProtocol::AddressType::PUBLIC;
                                break;
                            case STATIC_RANDOM_ADDR:
                                peerAddrType = BLEProtocol::AddressType::RANDOM_STATIC;
                                break;
                            case RESOLVABLE_PRIVATE_ADDR:
                                peerAddrType = BLEProtocol::AddressType::RANDOM_PRIVATE_RESOLVABLE;
                                break;
                            case NON_RESOLVABLE_PRIVATE_ADDR:
                                peerAddrType = BLEProtocol::AddressType::RANDOM_PRIVATE_NON_RESOLVABLE;
                                break;
                        }                                             
                        //PRINTF("EVT_LE_CONN_COMPLETE LL role=%d\n", cc->role);
                        switch (cc->role) {
			case 0: //master
                                role = Gap::CENTRAL;
                                break;
			case 1:
                                role = Gap::PERIPHERAL;
                                break;
			default:
                                role = Gap::PERIPHERAL;
				break;
                        }
                        //PRINTF("EVT_LE_CONN_COMPLETE GAP role=%d\n", role);
                        BlueNRGGap::getInstance().processConnectionEvent(cc->handle, role/*Gap::PERIPHERAL*/, peerAddrType, cc->peer_bdaddr, addr_type, bleAddr, (const BlueNRGGap::ConnectionParams_t *)&connectionParams);                            
                    }
                    break;
          
        case EVT_LE_ADVERTISING_REPORT:
          PRINTF("EVT_LE_ADVERTISING_REPORT\n\r");
          /* FIXME: comment this otherwise it will be obscure and error prone if BlueNRG FW will be updated */
          // This event is generated only by X-NUCLEO-IDB05A1 version but not by X-NUCLEO-IDB04A1 (which generates DEVICE_FOUND EVT)
          // Formally the structure related to both events are identical except that for the ADV REPORT
          // there is one more field (number of reports) which is not forwarded to upper layer.
          // Thus we need to move one byte over (((uint8_t*)evt->data)+1) before persing the ADV REPORT. 
          le_advertising_info *pr = (le_advertising_info*) (((uint8_t*)evt->data)+1);
          PRINTF("EVT_LE_ADVERTISING_REPORT evt_type=%d\n\r", pr->evt_type);
          
          BlueNRGGap::getInstance().Discovery_CB(BlueNRGGap::DEVICE_FOUND,
                                                 pr->evt_type,
                                                 &pr->bdaddr_type,
                                                 pr->bdaddr,
                                                 &pr->data_length,
                                                 &pr->data_RSSI[0],
                                                 &pr->data_RSSI[pr->data_length]);
                    break;
                }
            }
            break;
            
        case EVT_VENDOR:
            {                
                evt_blue_aci *blue_evt = (evt_blue_aci*)event_pckt->data;
                //PRINTF("EVT_VENDOR %d\n", blue_evt->ecode);
                
                switch(blue_evt->ecode){
                           
                case EVT_BLUE_GATT_READ_PERMIT_REQ:
                    {
                        PRINTF("EVT_BLUE_GATT_READ_PERMIT_REQ_OK\n\r");
                        evt_gatt_read_permit_req *pr = (evt_gatt_read_permit_req*)blue_evt->data;
                        BlueNRGGattServer::getInstance().Read_Request_CB(pr->attr_handle-CHAR_VALUE_OFFSET);                                                
                    }
                    break;
                    
                case EVT_BLUE_GATT_ATTRIBUTE_MODIFIED:         
                    {
                        PRINTF("EVT_BLUE_GATT_ATTRIBUTE_MODIFIED\n\r");
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
                    PRINTF("EVT_BLUE_GATT_NOTIFICATION");
                    break;
                case EVT_BLUE_GATT_INDICATION:
                    //This is only relevant for Client Side Event
                    PRINTF("EVT_BLUE_GATT_INDICATION");
                    break;   
                    
        case EVT_BLUE_ATT_READ_BY_GROUP_TYPE_RESP:
          {
            PRINTF("EVT_BLUE_ATT_READ_BY_GROUP_TYPE_RESP\n\r");
            evt_att_read_by_group_resp *pr = (evt_att_read_by_group_resp*)blue_evt->data;
            BlueNRGGattClient::getInstance().primaryServicesCB(pr->conn_handle,
                                                               pr->event_data_length,
                                                               pr->attribute_data_length,
                                                               pr->attribute_data_list);
          }
          break;
        case EVT_BLUE_ATT_READ_BY_TYPE_RESP:
          {
            PRINTF("EVT_BLUE_ATT_READ_BY_TYPE_RESP\n\r");
            evt_att_read_by_type_resp *pr = (evt_att_read_by_type_resp*)blue_evt->data;
            BlueNRGGattClient::getInstance().serviceCharsCB(pr->conn_handle,
                                                            pr->event_data_length,
                                                            pr->handle_value_pair_length,
                                                            pr->handle_value_pair);
          }
          break;
        case EVT_BLUE_ATT_READ_RESP:
          {
            PRINTF("EVT_BLUE_ATT_READ_RESP\n\r");
            evt_att_read_resp *pr = (evt_att_read_resp*)blue_evt->data;
            BlueNRGGattClient::getInstance().charReadCB(pr->conn_handle,
                                                        pr->event_data_length,
                                                        pr->attribute_value);
          }
          break;
        case EVT_BLUE_ATT_EXEC_WRITE_RESP:
          {
            PRINTF("EVT_BLUE_ATT_EXEC_WRITE_RESP\n\r");
            evt_att_prepare_write_resp *pr = (evt_att_prepare_write_resp*)blue_evt->data;
            BlueNRGGattClient::getInstance().charWriteExecCB(pr->conn_handle,
                                                             pr->event_data_length);
          }
          break;
        case EVT_BLUE_ATT_PREPARE_WRITE_RESP:
          {
            PRINTF("EVT_BLUE_ATT_PREPARE_WRITE_RESP\n\r");
            evt_att_prepare_write_resp *pr = (evt_att_prepare_write_resp*)blue_evt->data;
            BlueNRGGattClient::getInstance().charWritePrepareCB(pr->conn_handle,
                                                                pr->event_data_length,
                                                                pr->attribute_handle,
                                                                pr->offset,
                                                                pr->part_attr_value);
          }
          break;
        case EVT_BLUE_GATT_DISC_READ_CHAR_BY_UUID_RESP:
          {
            PRINTF("EVT_BLUE_GATT_DISC_READ_CHAR_BY_UUID_RESP\n\r");
            evt_gatt_disc_read_char_by_uuid_resp *pr = (evt_gatt_disc_read_char_by_uuid_resp*)blue_evt->data;
            BlueNRGGattClient::getInstance().serviceCharByUUIDCB(pr->conn_handle,
                                                                 pr->event_data_length,
                                                                 pr->attr_handle,
                                                                 pr->attr_value);
          }
          break;
        case EVT_BLUE_ATT_FIND_BY_TYPE_VAL_RESP:
          {
            PRINTF("EVT_BLUE_ATT_FIND_BY_TYPE_VAL_RESP\n\r");
            evt_att_find_by_type_val_resp *pr = (evt_att_find_by_type_val_resp*)blue_evt->data;
            BlueNRGGattClient::getInstance().primaryServiceCB(pr->conn_handle,
                                                              pr->event_data_length,
                                                              pr->handles_info_list);
          }
          break;
                case EVT_BLUE_GATT_PROCEDURE_COMPLETE:
          {
            //PRINTF("EVT_BLUE_GATT_PROCEDURE_COMPLETE\n\r");
            evt_gatt_procedure_complete *evt = (evt_gatt_procedure_complete*)blue_evt->data;
            PRINTF("EVT_BLUE_GATT_PROCEDURE_COMPLETE error_code=%d\n\r", evt->error_code);
            BlueNRGGattClient::getInstance().gattProcedureCompleteCB(evt->conn_handle, evt->error_code);
          }
          break;
          
        case EVT_BLUE_GAP_DEVICE_FOUND:
          {
            PRINTF("EVT_BLUE_GAP_DEVICE_FOUND\n\r");
            evt_gap_device_found *pr = (evt_gap_device_found*)blue_evt->data;
            PRINTF("EVT_BLUE_GAP_DEVICE_FOUND evt_type=%d\n\r", pr->evt_type);
            
            BlueNRGGap::getInstance().Discovery_CB(BlueNRGGap::DEVICE_FOUND,
                                                   pr->evt_type,
                                                   &pr->bdaddr_type,
                                                   pr->bdaddr,
                                                   &pr->data_length,
                                                   &pr->data_RSSI[0],
                                                   &pr->data_RSSI[pr->data_length]);
          }
          break;
          
        case EVT_BLUE_GAP_PROCEDURE_COMPLETE:
          {
            evt_gap_procedure_complete *pr = (evt_gap_procedure_complete*)blue_evt->data;
            //printf("EVT_BLUE_GAP_PROCEDURE_COMPLETE (code=0x%02X)\n\r", pr->procedure_code);
            
            switch(pr->procedure_code) {
            case GAP_LIMITED_DISCOVERY_PROC:
            case GAP_GENERAL_DISCOVERY_PROC:
              
              BlueNRGGap::getInstance().Discovery_CB(BlueNRGGap::DISCOVERY_COMPLETE, 0, NULL, NULL, NULL, NULL, NULL);
              break;
            }
          }
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
