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
#include "ble_utils.h"

#include "x_nucleo_idb0xa1_targets.h"

#ifdef __cplusplus
extern "C" {
#endif


/* C File Includes ------------------------------------------------------------------*/
#include <stdio.h>
#include <string.h>
#include "ble_hci.h"
#include "ble_hci_const.h"
#include "bluenrg_aci.h"
#include "bluenrg_hal_aci.h"
#include "bluenrg_gap.h"
#include "bluenrg_utils.h"

#include "ble_hal_types.h"
#include "ble_hal.h"
#include "ble_gp_timer.h"
#include "ble_osal.h"
#include "ble_sm.h"
#include "ble_debug.h"

#ifdef __cplusplus
}
#endif

#define IDB04A1 0
#define IDB05A1 1

/* See file 'x_nucleo_idb0xa1_targets.h' for details regarding the IDB0XA1 STACK_MODE */
#define STACK_MODE IDB0XA1_STACK_MODE

void HCI_Input(tHciDataPacket * hciReadPacket);

uint16_t g_gap_service_handle = 0;
uint16_t g_appearance_char_handle = 0;
uint16_t g_device_name_char_handle = 0;
uint16_t g_preferred_connection_parameters_char_handle = 0;

/* Private variables ---------------------------------------------------------*/
volatile uint8_t set_connectable = 1;

static char versionString[32];
uint8_t bnrg_expansion_board = IDB04A1; /* at startup, suppose the X-NUCLEO-IDB04A1 is used */

/**************************************************************************/
/*!
    @brief  Init the BTLE stack with the specified role
    @returns void
*/
/**************************************************************************/
void btleInit(void)
{
    PRINTF("btleInit>>\n\r");

    int ret;
    uint8_t  hwVersion;
    uint16_t fwVersion;
    uint16_t service_handle, dev_name_char_handle, appearance_char_handle;

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

    /* set BLE version string */
    setVersionString(hwVersion, fwVersion);

    if (bnrg_expansion_board == IDB05A1) {
        uint8_t stackMode = STACK_MODE;
        ret = aci_hal_write_config_data(CONFIG_DATA_ROLE,
                                        CONFIG_DATA_ROLE_LEN,
                                        &stackMode);
    }

    ret = aci_gatt_init();
    if(ret != BLE_STATUS_SUCCESS){
        PRINTF("GATT_Init failed.\n");
    }
    if (bnrg_expansion_board == IDB05A1) {
        ret = aci_gap_init_IDB05A1(GAP_PERIPHERAL_ROLE_IDB05A1|GAP_CENTRAL_ROLE_IDB05A1|GAP_OBSERVER_ROLE_IDB05A1,
                                   0,
                                   0x18,
                                   &service_handle,
                                   &dev_name_char_handle,
                                   &appearance_char_handle);
    } else {
        // IDB04A1 is configured as peripheral by default
        ret = aci_gap_init_IDB04A1(GAP_PERIPHERAL_ROLE_IDB04A1, &service_handle, &dev_name_char_handle, &appearance_char_handle);
    }

    // read the default static address and inject it into the GAP object
    {
        Gap::Address_t BLE_address_BE = { 0 };
        uint8_t data_len_out;
        aci_hal_read_config_data(CONFIG_DATA_RANDOM_ADDRESS_IDB05A1, BDADDR_SIZE, &data_len_out, BLE_address_BE);
        // FIXME error handling of this function
        BlueNRGGap::getInstance().setAddress(BLEProtocol::AddressType::RANDOM_STATIC, BLE_address_BE);
    }

    if(ret != BLE_STATUS_SUCCESS){
        PRINTF("GAP_Init failed.\n");
    }

    //FIXME: Security and passkey set by default
    ret = aci_gap_set_auth_requirement(MITM_PROTECTION_REQUIRED,
                                       OOB_AUTH_DATA_ABSENT,
                                       NULL,
                                       7,
                                       16,
                                       USE_FIXED_PIN_FOR_PAIRING,
                                       123456,
                                       BONDING);
    if (ret != BLE_STATUS_SUCCESS) {
        PRINTF("Auth Req set failed.\n");
    }

    aci_hal_set_tx_power_level(1,4);

    g_gap_service_handle = service_handle;
    g_appearance_char_handle = appearance_char_handle;
    g_device_name_char_handle = dev_name_char_handle;
    //Device Name is set from Accumulate Adv Data Payload or through setDeviceName API
    /*ret = aci_gatt_update_char_value(service_handle, dev_name_char_handle, 0,
                            strlen(name), (tHalUint8 *)name);*/

    signalEventsToProcess();
    // update the peripheral preferred conenction parameters handle
    // This value is hardcoded at the moment.
    g_preferred_connection_parameters_char_handle = 10;

    return;
}

/**************************************************************************/
/*!
    @brief  mbedOS

    @param[in]  void

    @returns
*/
/**************************************************************************/
int btle_handler_pending = 0;

void btle_handler(void)
{
    btle_handler_pending = 0;
    BlueNRGGap::getInstance().Process();
    HCI_Process();
}

/* set BLE Version string */
void setVersionString(uint8_t hwVersion, uint16_t fwVersion)
{
    if(bnrg_expansion_board == IDB04A1 || bnrg_expansion_board == IDB05A1) {
        snprintf(versionString, sizeof(versionString), "ST BLE4.1 HW v%u.%u FW v%u.%u",
                                                        hwVersion>>4, (hwVersion&0x0F),
                                                        fwVersion>>8, (fwVersion&0x00F0)>>4);
    } else {
        snprintf(versionString, sizeof(versionString), "ST (unknown spec)");
    }
}

/* get BLE Version string */
const char* getVersionString(void)
{
    return versionString;
}

tBleStatus btleStartRadioScan(uint8_t scan_type,
                              uint16_t scan_interval,
                              uint16_t scan_window,
                              uint8_t own_address_type)
{
  tBleStatus ret;

  // Observer role is not supported by X-NUCLEO-IDB04A1, return BLE_ERROR_NOT_IMPLEMENTED
  if(bnrg_expansion_board == IDB05A1) {
      PRINTF("scan_interval=%d scan_window=%d\n\r", scan_interval, scan_window);
      PRINTF("scan_type=%d own_address_type=%d\n\r", scan_type, own_address_type);
      ret = aci_gap_start_observation_procedure(scan_interval,
                                                scan_window,
                                                scan_type,
                                                own_address_type,
                                                0); // 1 to filter duplicates
  } else {
      ret = BLE_STATUS_INVALID_CID;
  }

  return ret;

}

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

void Attribute_Modified_CB(evt_blue_aci *blue_evt)
{
    uint16_t conn_handle;
    uint16_t attr_handle;
    uint8_t data_length;
    uint8_t *att_data;
    uint8_t offset;

    if (bnrg_expansion_board == IDB05A1) {
        evt_gatt_attr_modified_IDB05A1 *evt = (evt_gatt_attr_modified_IDB05A1*)blue_evt->data;
        conn_handle = evt->conn_handle;
        attr_handle = evt->attr_handle;
        data_length = evt->data_length;
        att_data = evt->att_data;
        offset = evt->offset;
    } else {
        evt_gatt_attr_modified_IDB04A1 *evt = (evt_gatt_attr_modified_IDB04A1*)blue_evt->data;
        conn_handle = evt->conn_handle;
        attr_handle = evt->attr_handle;
        data_length = evt->data_length;
        att_data = evt->att_data;
        offset = 0;
    }

    //Extract the GattCharacteristic from p_characteristics[] and find the properties mask
    GattCharacteristic *p_char = BlueNRGGattServer::getInstance().getCharacteristicFromHandle(attr_handle);
    if(p_char!=NULL) {
        GattAttribute::Handle_t charHandle = p_char->getValueAttribute().getHandle()-BlueNRGGattServer::CHAR_VALUE_HANDLE;
        BlueNRGGattServer::HandleEnum_t currentHandle = BlueNRGGattServer::CHAR_HANDLE;
        PRINTF("CharHandle %d, length: %d, Data: %d\n\r", charHandle, data_length, (uint16_t)att_data[0]);
        PRINTF("getProperties 0x%x\n\r",p_char->getProperties());

        if(attr_handle == charHandle+BlueNRGGattServer::CHAR_VALUE_HANDLE) {
            currentHandle = BlueNRGGattServer::CHAR_VALUE_HANDLE;
        }

        if(attr_handle == charHandle+BlueNRGGattServer::CHAR_DESC_HANDLE) {
            currentHandle = BlueNRGGattServer::CHAR_DESC_HANDLE;
        }
        PRINTF("currentHandle %d\n\r", currentHandle);
        if((p_char->getProperties() &
            (GattCharacteristic::BLE_GATT_CHAR_PROPERTIES_NOTIFY | GattCharacteristic::BLE_GATT_CHAR_PROPERTIES_INDICATE)) &&
            currentHandle == BlueNRGGattServer::CHAR_DESC_HANDLE) {

            GattAttribute::Handle_t charDescHandle = p_char->getValueAttribute().getHandle()+1;

            PRINTF("*****NOTIFICATION CASE\n\r");
            //Now Check if data written in Enable or Disable
            if((uint16_t)att_data[0]==1) {
                //PRINTF("Notify ENABLED\n\r");
                BlueNRGGattServer::getInstance().HCIEvent(GattServerEvents::GATT_EVENT_UPDATES_ENABLED, charDescHandle);
            } else {
                //PRINTF("Notify DISABLED\n\r");
                BlueNRGGattServer::getInstance().HCIEvent(GattServerEvents::GATT_EVENT_UPDATES_DISABLED, charDescHandle);
            }
            return;
        }

        //Check if attr handle property is WRITEABLE, in the case generate GATT_EVENT_DATA_WRITTEN Event
        if((p_char->getProperties() &
            (GattCharacteristic::BLE_GATT_CHAR_PROPERTIES_WRITE_WITHOUT_RESPONSE | GattCharacteristic::BLE_GATT_CHAR_PROPERTIES_WRITE)) &&
            currentHandle == BlueNRGGattServer::CHAR_VALUE_HANDLE) {

            PRINTF("*****WRITE CASE\n\r");

            GattWriteCallbackParams writeParams;
            writeParams.connHandle = conn_handle;
            writeParams.handle = p_char->getValueAttribute().getHandle();
            writeParams.writeOp = GattWriteCallbackParams::OP_WRITE_REQ;//Where to find this property in BLUENRG?
            writeParams.len = data_length;
            writeParams.data = att_data;
            writeParams.offset = offset;

            //BlueNRGGattServer::getInstance().handleEvent(GattServerEvents::GATT_EVENT_DATA_WRITTEN, attr_handle);
            //Write the actual Data to the Attr Handle? (uint8_1[])att_data contains the data
            if ((p_char->getValueAttribute().getValuePtr() != NULL) && (p_char->getValueAttribute().getLength() > 0)) {
                BlueNRGGattServer::getInstance().write(
                    p_char->getValueAttribute().getHandle(),
                    (uint8_t*)att_data,
                    data_length,
                    false
                );
            }

            BlueNRGGattServer::getInstance().HCIDataWrittenEvent(&writeParams);
        } else {
            PRINTF("*****WRITE DESCRIPTOR CASE\n\r");

            GattWriteCallbackParams writeParams;
            writeParams.connHandle = conn_handle;
            writeParams.handle = attr_handle;
            writeParams.writeOp = GattWriteCallbackParams::OP_WRITE_REQ;//Where to find this property in BLUENRG?
            writeParams.len = data_length;
            writeParams.data = att_data;
            writeParams.offset = offset;

            BlueNRGGattServer::getInstance().HCIDataWrittenEvent(&writeParams);
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

                evt_disconn_complete *evt = (evt_disconn_complete*)event_pckt->data;

                BlueNRGGap::getInstance().processDisconnectionEvent(evt->handle, (Gap::DisconnectionReason_t)evt->reason);
            }
            break;

        case EVT_LE_META_EVENT:
            {
                PRINTF("EVT_LE_META_EVENT\n");

                evt_le_meta_event *evt = (evt_le_meta_event *)event_pckt->data;

                switch(evt->subevent){

                case EVT_LE_CONN_COMPLETE:
                    {
                        PRINTF("EVT_LE_CONN_COMPLETE\n");
                        Gap::Address_t ownAddr;
                        Gap::AddressType_t ownAddrType;
                        BlueNRGGap::getInstance().getAddress(&ownAddrType, ownAddr);

                        Gap::AddressType_t peerAddrType = BLEProtocol::AddressType::RANDOM_STATIC;
                        Gap::Role_t role;

                        evt_le_connection_complete *cc = (evt_le_connection_complete *)evt->data;

                        BlueNRGGap::getInstance().setConnectionHandle(cc->handle);
                        BlueNRGGap::ConnectionParams_t connectionParams = {
                            /* minConnectionInterval = */ cc->interval,
                            /* maxConnectionInterval = */ cc->interval,
                            /* slaveLatency = */ cc->latency,
                            /* connectionSupervisionTimeout = */ cc->supervision_timeout
                        };

                        BlueNRGGap::getInstance().setConnectionInterval(cc->interval);

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

                        BlueNRGGap::getInstance().setGapRole(role);

                        BlueNRGGap::getInstance().processConnectionEvent(cc->handle,
                                                                         role,
                                                                         peerAddrType,
                                                                         cc->peer_bdaddr,
                                                                         ownAddrType,
                                                                         ownAddr,
                                                                         &connectionParams);
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
                                                 pr->bdaddr_type,
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

                case EVT_BLUE_GATT_WRITE_PERMIT_REQ:
                    {
                        PRINTF("EVT_BLUE_GATT_WRITE_PERMIT_REQ\r\n");
                        evt_gatt_write_permit_req* write_req = (evt_gatt_write_permit_req*)blue_evt->data;

                        // ask the local server if the write operation is authorized
                        uint8_t err_code = BlueNRGGattServer::getInstance().Write_Request_CB(
                            write_req->conn_handle,
                            write_req->attr_handle,
                            write_req->data_length,
                            write_req->data
                        );
                        uint8_t write_status = err_code == 0 ? 0 : 1;

                        // reply to the shield
                        aci_gatt_write_response(
                            write_req->conn_handle,
                            write_req->attr_handle,
                            write_status,
                            err_code,
                            write_req->data_length,
                            write_req->data
                        );
                    }
                    break;

                case EVT_BLUE_GATT_READ_PERMIT_REQ:
                    {
                        PRINTF("EVT_BLUE_GATT_READ_PERMIT_REQ_OK\n\r");
                        evt_gatt_read_permit_req *pr = (evt_gatt_read_permit_req*)blue_evt->data;
                        PRINTF("EVT_BLUE_GATT_READ_PERMIT_REQ_OK pr->attr_handle=%u\n\r", pr->attr_handle);
                        BlueNRGGattServer::getInstance().Read_Request_CB(pr->attr_handle);
                    }
                    break;

                case EVT_BLUE_GATT_ATTRIBUTE_MODIFIED:
                    {
                        PRINTF("EVT_BLUE_GATT_ATTRIBUTE_MODIFIED\n\r");
                        /* this callback is invoked when a GATT attribute is modified
                            extract callback data and pass to suitable handler function */
                        Attribute_Modified_CB(blue_evt);
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
        case EVT_BLUE_ATT_FIND_INFORMATION_RESP:
          {
            PRINTF("EVT_BLUE_ATT_FIND_INFORMATION_RESP\n\r");
            evt_att_find_information_resp *pr = (evt_att_find_information_resp*)blue_evt->data;
            BlueNRGGattClient::getInstance().discAllCharacDescCB(pr->conn_handle,
                                                                 pr->event_data_length,
                                                                 pr->format,
                                                                 pr->handle_uuid_pair);
          }
          break;
        case EVT_BLUE_GATT_PROCEDURE_COMPLETE:
          {
            evt_gatt_procedure_complete *evt = (evt_gatt_procedure_complete*)blue_evt->data;
            PRINTF("EVT_BLUE_GATT_PROCEDURE_COMPLETE error_code=%d\n\r", evt->error_code);
            BlueNRGGattClient::getInstance().gattProcedureCompleteCB(evt->conn_handle, evt->error_code);
          }
          break;

        case EVT_BLUE_L2CAP_CONN_UPD_REQ:
          {
            PRINTF("EVT_BLUE_L2CAP_CONN_UPD_REQ\r\n");
            evt_l2cap_conn_upd_req *evt = (evt_l2cap_conn_upd_req*)blue_evt->data;
            if(bnrg_expansion_board == IDB05A1) {
              // we assume the application accepts the request from the slave
              aci_l2cap_connection_parameter_update_response_IDB05A1(evt->conn_handle,
                                                                     evt->interval_min,
                                                                     evt->interval_max,
                                                                     evt->slave_latency,
                                                                     evt->timeout_mult,
                                                                     CONN_L1, CONN_L2,
                                                                     evt->identifier,
                                                                     0x0000);
            }
          }
          break;

        case EVT_BLUE_L2CAP_CONN_UPD_RESP:
          {
            PRINTF("EVT_BLUE_L2CAP_CONN_UPD_RESP\r\n");
          }
          break;

        case EVT_LE_CONN_UPDATE_COMPLETE:
          {
            PRINTF("EVT_LE_CONN_UPDATE_COMPLETE\r\n");
          }
          break;

        case EVT_BLUE_GAP_DEVICE_FOUND:
          {
            evt_gap_device_found *pr = (evt_gap_device_found*)blue_evt->data;
            PRINTF("EVT_BLUE_GAP_DEVICE_FOUND evt_type=%d\n\r", pr->evt_type);

            BlueNRGGap::getInstance().Discovery_CB(BlueNRGGap::DEVICE_FOUND,
                                                   pr->evt_type,
                                                   pr->bdaddr_type,
                                                   pr->bdaddr,
                                                   &pr->data_length,
                                                   &pr->data_RSSI[0],
                                                   &pr->data_RSSI[pr->data_length]);
          }
          break;

        case EVT_BLUE_GAP_PROCEDURE_COMPLETE:
          {
            evt_gap_procedure_complete *pr = (evt_gap_procedure_complete*)blue_evt->data;
            //PRINTF("EVT_BLUE_GAP_PROCEDURE_COMPLETE (code=0x%02X)\n\r", pr->procedure_code);

            switch(pr->procedure_code) {
            case GAP_OBSERVATION_PROC_IDB05A1:

              BlueNRGGap::getInstance().Discovery_CB(BlueNRGGap::DISCOVERY_COMPLETE, 0, 0, NULL, NULL, NULL, NULL);
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
