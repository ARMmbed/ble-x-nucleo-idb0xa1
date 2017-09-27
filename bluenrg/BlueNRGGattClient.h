/* mbed Microcontroller Library
 * Copyright (c) 2017-2017 ARM Limited
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

#ifndef ST_BLUENRGATTCLIENT_H_
#define ST_BLUENRGATTCLIENT_H_

extern "C" {
#include "bluenrg_gatt_aci.h"
#include "ble_hci.h"
#include "ble_hci_const.h"
#include "bluenrg_aci_const.h"
#include "ble_debug.h"
}

#include "ble/pal/AttClient.h"
#include "ble/pal/SimpleAttServerMessage.h"

#define COUNT_OF(x) ((sizeof(x)/sizeof(0[x])) / ((size_t)(!(sizeof(x) % sizeof(0[x])))))

namespace ble {
namespace pal {
namespace st {

/**
 * Implementation of ble::pal::AttClient for ST BlueNRG targets.
 */
class BlueNRGGattClient: public AttClient {
public:
    enum Code {
        GATT_IDLE = 0x01,
        GATT_READ_CHAR = 0x02,
        GATT_READ_LONG_CHAR = 0x03,
        GATT_READ_MULTIPLE_CHAR = 0x04,
        GATT_WRITE_CHAR = 0x05
    };

    /**
     * Access to the AttClient singleton.
     */
    //static BlueNRGGattClient& get_client() {
    static BlueNRGGattClient& getInstance() {
        static BlueNRGGattClient _client;
        return _client;
    }

    /**
     * @see ble::pal::AttClient::exchange_mtu_request
     */
    virtual ble_error_t exchange_mtu_request(connection_handle_t connection) {
        return _error_converter << aci_gatt_exchange_configuration(connection);
    }

    /**
     * @see ble::pal::AttClient::get_mtu_size
     */
    virtual ble_error_t get_mtu_size(
        connection_handle_t connection_handle,
        uint16_t& mtu_size
    ) {
        mtu_size = ATT_MTU;
        return BLE_ERROR_NONE;
    }

    /**
     * @see ble::pal::AttClient::find_information_request
     */
    virtual ble_error_t find_information_request(
        connection_handle_t connection_handle,
        attribute_handle_range_t discovery_range
    ) {
        return _error_converter << aci_att_find_information_req(
            connection_handle,
            discovery_range.begin,
            discovery_range.end
        );
    }

    /**
     * @see ble::pal::AttClient::find_by_type_value_request
     */
    virtual ble_error_t find_by_type_value_request(
        connection_handle_t connection_handle,
        attribute_handle_range_t discovery_range,
        uint16_t type,
        const ArrayView<const uint8_t>& value
    ) {
        return _error_converter << aci_att_find_by_type_value_req(
            connection_handle,
            discovery_range.begin,
            discovery_range.end,
            reinterpret_cast<uint8_t*>(&type),
            value.size(),
            const_cast<uint8_t*>(value.data())
        );
    }

    /**
     * @see ble::pal::AttClient::read_by_type_request
     */
    virtual ble_error_t read_by_type_request(
        connection_handle_t connection_handle,
        attribute_handle_range_t read_range,
        const UUID& type
    ) {
        return _error_converter << aci_att_read_by_type_req(
            connection_handle,
            read_range.begin,
            read_range.end,
            type.shortOrLong() == UUID::UUID_TYPE_SHORT ? UUID_TYPE_16 : UUID_TYPE_128,
            const_cast<uint8_t*>(type.getBaseUUID())
        );
    }

    /**
     * @see ble::pal::AttClient::read_request
     */
    virtual ble_error_t read_request(
        connection_handle_t connection_handle,
        attribute_handle_t attribute_handle
    ) {
        gattClientState = GATT_READ_CHAR;

        return _error_converter << aci_gatt_read_charac_val(
            connection_handle, attribute_handle
        );
    }

    /**
     * @see ble::pal::AttClient::read_blob_request
     */
    virtual ble_error_t read_blob_request(
        connection_handle_t connection_handle,
        attribute_handle_t attribute_handle,
        uint16_t offset
    ) {
        gattClientState = GATT_READ_LONG_CHAR;

        return _error_converter << aci_gatt_read_long_charac_val(
            connection_handle, attribute_handle, offset
        );
    }

    /**
     * @see ble::pal::AttClient::read_multiple_request
     */
    virtual ble_error_t read_multiple_request(
        connection_handle_t connection_handle,
        const ArrayView<const attribute_handle_t>& attribute_handles
    ) {
        gattClientState = GATT_READ_MULTIPLE_CHAR;

        return _error_converter << aci_gatt_read_multiple_charac_val(
            connection_handle,
            attribute_handles.size(),
            (uint8_t*) attribute_handles.data()
        );
    }

    /**
     * @see ble::pal::AttClient::read_by_group_type_request
     */
    virtual ble_error_t read_by_group_type_request(
        connection_handle_t connection_handle,
        attribute_handle_range_t read_range,
        const UUID& group_type
    ) {
        return _error_converter << aci_att_read_by_group_type_req(
            connection_handle,
            read_range.begin,
            read_range.end,
            group_type.shortOrLong() == UUID::UUID_TYPE_SHORT ? UUID_TYPE_16 : UUID_TYPE_128,
            const_cast<uint8_t*>(group_type.getBaseUUID())
        );
    }

    /**
     * @see ble::pal::AttClient::write_request
     */
    virtual ble_error_t write_request(
        connection_handle_t connection_handle,
        attribute_handle_t attribute_handle,
        const ArrayView<const uint8_t>& value
    ) {
        gattClientState = GATT_WRITE_CHAR;

        return _error_converter << aci_gatt_write_charac_value(
            connection_handle,
            attribute_handle,
            value.size(),
            const_cast<uint8_t*>(value.data())
        );
    }

    /**
     * @see ble::pal::AttClient::write_command
     */
    virtual ble_error_t write_command(
        connection_handle_t connection_handle,
        attribute_handle_t attribute_handle,
        const ArrayView<const uint8_t>& value
    ) {
        return _error_converter << aci_gatt_write_without_response(
            connection_handle,
            attribute_handle,
            value.size(),
            const_cast<uint8_t*>(value.data())
        );
    }

    /**
     * @see ble::pal::AttClient::signed_write_command
     */
    virtual ble_error_t signed_write_command(
        connection_handle_t connection_handle,
        attribute_handle_t attribute_handle,
        const ArrayView<const uint8_t>& value
    ) {
        // authentication signature calculated by the system ...
        return _error_converter << aci_gatt_signed_write_without_resp(
            connection_handle,
            attribute_handle,
            value.size(),
            const_cast<uint8_t*>(value.data())
        );
    }

    /**
     * @see ble::pal::AttClient::prepare_write_request
     */
    virtual ble_error_t prepare_write_request(
        connection_handle_t connection_handle,
        attribute_handle_t attribute_handle,
        uint16_t offset,
        const ArrayView<const uint8_t>& value
    ) {
        return _error_converter << aci_att_prepare_write_req(
            connection_handle,
            attribute_handle,
            offset,
            value.size(),
            const_cast<uint8_t*>(value.data())
        );
    }

    /**
     * @see ble::pal::AttClient::execute_write_request
     */
    virtual ble_error_t execute_write_request(
        connection_handle_t connection_handle,
        bool execute
    ) {
        return _error_converter << aci_att_execute_write_req(
            connection_handle, execute
        );
    }

    /**
     * @see ble::pal::AttClient::initialize
     */
    virtual ble_error_t initialize() {
        // TODO
        gattClientState = GATT_IDLE;
        return BLE_ERROR_NONE;
    }

    /**
     * @see ble::pal::AttClient::terminate
     */
    virtual ble_error_t terminate() {
        // TODO
        gattClientState = GATT_IDLE;
        return BLE_ERROR_NONE;
    }

    // callback handling aci events
    static void aci_event_cb(const evt_blue_aci* event) {

        // event handlers are stored in an array.
        static const event_handler_t handlers[] = {
            &event_handler<ExchangeMtuResponseConverter>,
            &event_handler<FindInformationResponseConverter>,
            &event_handler<FindByTypeValueResponseConverter>,
            &event_handler<ReadByTypeResponseConverter>,
            &event_handler<ReadResponseConverter>,
            &event_handler<ReadBlobResponseConverter>,
            &event_handler<ReadMultipleResponseConverter>,
            &event_handler<ReadBygroupTypeResponseConverter>,
            &event_handler<PrepareWriteResponseConverter>,
            &event_handler<ExecuteWriteResponseConverter>,
            &event_handler<HandleValueIndicationConverter>,
            &event_handler<HandleValueNotificationConverter>,
            &event_handler<ErrorResponseConverter>,
            &gatt_procedure_complete_event_handler,
            &timeout_event_handler
        };

        // dispatch the event sequentially to all event handlers.
        // if one of them is able to process the event, exit the function.
        for(size_t i = 0; i < COUNT_OF(handlers); ++i) {
            if (handlers[i](event)) {
                return;
            }
        }
    }

private:
    BlueNRGGattClient() { }

    uint8_t gattClientState;
    GattReadCallbackParams readCBParams;

    virtual ~BlueNRGGattClient() { }

    // type of an aci event handler
    // they return true if the event has been handled and false
    // otherwise.
    typedef bool (*event_handler_t)(const evt_blue_aci*);

    /**
     * Event handler base callback.
     * @tparam T structure describing a converter between aci event and
     * AttServerMessage. It should contain the field event_id corresponding to
     * the event to handle and a function named converter which return the
     * AttServerMessage corresponding to the event in input.
     * @param event
     * @return
     */
    template<typename T>
    static bool event_handler(const evt_blue_aci* event) {
        if (event->ecode == T::event_id) {
            generated_handler(event, T::converter);
            return true;
        }
        return false;
    }

    static bool gatt_procedure_complete_event_handler(const evt_blue_aci* event) {

        if (event->ecode == EVT_BLUE_GATT_PROCEDURE_COMPLETE) {
            const evt_gatt_procedure_complete* evt = (evt_gatt_procedure_complete*)event->data;

            if(evt->error_code != BLE_STATUS_SUCCESS) {
                return true;
            }

            switch (getInstance().gattClientState) {
                case GATT_READ_CHAR :
                    getInstance().gattClientState = GATT_IDLE;
                    getInstance().on_server_event(
                        getInstance().readCBParams.connHandle,
                        AttReadResponse(
                            make_ArrayView(
                                getInstance().readCBParams.data,
                                getInstance().readCBParams.len
                            )
                        )
                    );
                    break;
                case GATT_READ_LONG_CHAR :
                    getInstance().gattClientState = GATT_IDLE;
                    getInstance().on_server_event(
                        getInstance().readCBParams.connHandle,
                        AttReadBlobResponse(
                            make_ArrayView(
                                getInstance().readCBParams.data,
                                getInstance().readCBParams.len
                            )
                        )
                    );
                    break;
                case GATT_READ_MULTIPLE_CHAR :
                    getInstance().gattClientState = GATT_IDLE;
                    getInstance().on_server_event(
                        getInstance().readCBParams.connHandle,
                        AttReadMultipleResponse(
                            make_ArrayView(
                                getInstance().readCBParams.data,
                                getInstance().readCBParams.len
                            )
                        )
                    );
                    break;
                case GATT_WRITE_CHAR :
                    getInstance().gattClientState = GATT_IDLE;
                    getInstance().on_server_event(
                        evt->conn_handle,
                        AttWriteResponse()
                    );
                    break;
            }

            if(getInstance().readCBParams.data != NULL) {
                free((void*)(getInstance().readCBParams.data));
                getInstance().readCBParams.data = NULL;
            }

            return true;
        }
        return false;

    }

    static bool timeout_event_handler(const evt_blue_aci* event) {
        if (event->ecode == EVT_BLUE_GATT_PROCEDURE_TIMEOUT) {
            const evt_gatt_procedure_timeout* evt = (evt_gatt_procedure_timeout*)event->data;
            getInstance().on_transaction_timeout(evt->conn_handle);

            return true;
        }
        return false;
    }

    template<typename ResultType, typename EventType>
    static void generated_handler(const evt_blue_aci* event, ResultType (*converter)(const EventType*)) {
        const EventType* response = reinterpret_cast<const EventType*>(event->data);

        // We need to wait for EVT_BLUE_GATT_PROCEDURE_COMPLETE event before ending the READ process
        // Copy the blob data read, they will be forwarded to the on_server_event() CB
        // once the procedure has been completed
        switch(event->ecode) {
            case EVT_BLUE_ATT_READ_RESP :
            {
                evt_att_read_resp *rr = (evt_att_read_resp*)event->data;

                getInstance().readCBParams.connHandle = rr->conn_handle;
                getInstance().readCBParams.offset = 0;
                getInstance().readCBParams.len = rr->event_data_length;
                getInstance().readCBParams.data = static_cast<uint8_t*>(malloc(rr->event_data_length));
                memcpy((void*)(getInstance().readCBParams.data), rr->attribute_value, rr->event_data_length);
            }
            return;

            case EVT_BLUE_ATT_READ_BLOB_RESP :
            {                
                evt_att_read_blob_resp *rbr = (evt_att_read_blob_resp*)event->data;

                getInstance().readCBParams.connHandle = rbr->conn_handle;
                getInstance().readCBParams.offset = 0;
                getInstance().readCBParams.len = rbr->event_data_length;
                getInstance().readCBParams.data = static_cast<uint8_t*>(malloc(rbr->event_data_length));
                memcpy((void*)(getInstance().readCBParams.data), rbr->part_attribute_value, rbr->event_data_length);
            }
            return;

            case EVT_BLUE_ATT_READ_MULTIPLE_RESP :
            {
                evt_att_read_mult_resp *rmr = (evt_att_read_mult_resp*)event->data;

                getInstance().readCBParams.connHandle = rmr->conn_handle;
                getInstance().readCBParams.offset = 0;
                getInstance().readCBParams.len = rmr->event_data_length;
                getInstance().readCBParams.data = static_cast<uint8_t*>(malloc(rmr->event_data_length));
                memcpy((void*)(getInstance().readCBParams.data), rmr->set_of_values, rmr->event_data_length);
            }
            return;
        }

        getInstance().on_server_event(
            response->conn_handle,
            converter(response)
        );

    }

    /**
     * Traits to declare an EventID
     */
    template<uint16_t Id>
    struct EventId {
        static const uint16_t event_id = Id;
    };

    /**
     * evt_att_exchange_mtu_resp to AttExchangeMTUResponse converter.
     */
    struct ExchangeMtuResponseConverter : EventId<EVT_BLUE_ATT_EXCHANGE_MTU_RESP> {
        static AttExchangeMTUResponse converter(const evt_att_exchange_mtu_resp* resp) {
            return AttExchangeMTUResponse(resp->server_rx_mtu);
        }
    };

    /**
     * evt_att_find_information_resp to SimpleAttFindInformationResponse converter.
     */
    struct FindInformationResponseConverter : EventId<EVT_BLUE_ATT_FIND_INFORMATION_RESP> {
        static SimpleAttFindInformationResponse converter(const evt_att_find_information_resp* resp) {
            return SimpleAttFindInformationResponse(
                static_cast<SimpleAttFindInformationResponse::Format>(resp->format),
                make_ArrayView(
                    resp->handle_uuid_pair,
                    resp->event_data_length - 1
                )
            );
        }
    };

    /**
     * evt_att_find_by_type_val_resp to SimpleAttFindByTypeValueResponse converter.
     */
    struct FindByTypeValueResponseConverter : EventId<EVT_BLUE_ATT_FIND_BY_TYPE_VAL_RESP> {
        static SimpleAttFindByTypeValueResponse converter(const evt_att_find_by_type_val_resp* resp) {           
            return SimpleAttFindByTypeValueResponse(
                make_ArrayView(
                    resp->handles_info_list,
                    resp->event_data_length
                )
            );
        }
    };

    /**
     * evt_att_read_by_type_resp to SimpleAttReadByTypeResponse converter.
     */
    struct ReadByTypeResponseConverter : EventId<EVT_BLUE_ATT_READ_BY_TYPE_RESP> {
        static SimpleAttReadByTypeResponse converter(const evt_att_read_by_type_resp* resp) {
            return SimpleAttReadByTypeResponse(
                resp->handle_value_pair_length,
                make_ArrayView(
                    resp->handle_value_pair,
                    resp->event_data_length - 1
                )
            );
        }
    };

    /**
     * evt_att_read_resp to AttReadResponse converter.
     */
    struct ReadResponseConverter : EventId<EVT_BLUE_ATT_READ_RESP> {
        static AttReadResponse converter(const evt_att_read_resp* resp) {
            return AttReadResponse(
                make_ArrayView(
                    resp->attribute_value,
                    resp->event_data_length
                )
            );
        }
    };

    /**
     * evt_att_read_blob_resp to AttReadBlobResponse converter.
     */
    struct ReadBlobResponseConverter : EventId<EVT_BLUE_ATT_READ_BLOB_RESP> {
        static AttReadBlobResponse converter(const evt_att_read_blob_resp* resp) {
            return AttReadBlobResponse(
                make_ArrayView(
                    resp->part_attribute_value,
                    resp->event_data_length
                )
            );
        }
    };

    /**
     * evt_att_read_mult_resp to AttReadMultipleResponse converter.
     */
    struct ReadMultipleResponseConverter : EventId<EVT_BLUE_ATT_READ_MULTIPLE_RESP> {
        static AttReadMultipleResponse converter(const evt_att_read_mult_resp* resp) {
            return AttReadMultipleResponse(
                make_ArrayView(
                    resp->set_of_values,
                    resp->event_data_length
                )
            );
        }
    };

    /**
     * evt_att_read_by_group_resp to SimpleAttReadByGroupTypeResponse converter.
     */
    struct ReadBygroupTypeResponseConverter : EventId<EVT_BLUE_ATT_READ_BY_GROUP_TYPE_RESP> {
        static SimpleAttReadByGroupTypeResponse converter(const evt_att_read_by_group_resp* resp) {
            return SimpleAttReadByGroupTypeResponse(
                resp->attribute_data_length,
                make_ArrayView(
                    resp->attribute_data_list,
                    resp->event_data_length - 1
                )
            );
        }
    };

    /**
     * evt_att_prepare_write_resp to AttPrepareWriteResponse converter.
     */
    struct PrepareWriteResponseConverter : EventId<EVT_BLUE_ATT_PREPARE_WRITE_RESP> {
        static AttPrepareWriteResponse converter(const evt_att_prepare_write_resp* resp) {
            return AttPrepareWriteResponse(
                resp->attribute_handle,
                resp->offset,
                make_ArrayView(
                    resp->part_attr_value,
                    resp->event_data_length - 2
                )
            );
        }
    };

    /**
     * evt_att_exec_write_resp to AttExecuteWriteResponse converter.
     */
    struct ExecuteWriteResponseConverter : EventId<EVT_BLUE_ATT_EXEC_WRITE_RESP> {
        static AttExecuteWriteResponse converter(const evt_att_exec_write_resp* resp) {
            return AttExecuteWriteResponse();
        }
    };

    /**
     * evt_gatt_indication to AttHandleValueIndication converter.
     */
    struct HandleValueIndicationConverter : EventId<EVT_BLUE_GATT_INDICATION> {
        static AttHandleValueIndication converter(const evt_gatt_indication* resp) {
            return AttHandleValueIndication(
                resp->attr_handle,
                make_ArrayView(
                    resp->attr_value,
                    resp->event_data_length - 1
                )
            );
        }
    };

    /**
     * evt_gatt_attr_notification to AttHandleValueNotification converter.
     */
    struct HandleValueNotificationConverter : EventId<EVT_BLUE_GATT_NOTIFICATION> {
        static AttHandleValueNotification converter(const evt_gatt_attr_notification* resp) {
            return AttHandleValueNotification(
                resp->attr_handle,
                make_ArrayView(
                    resp->attr_value,
                    resp->event_data_length - 1
                )
            );
        }
    };

    /**
     * evt_gatt_error_resp to AttErrorResponse converter.
     */
    struct ErrorResponseConverter : EventId<EVT_BLUE_GATT_ERROR_RESP> {
        static AttErrorResponse converter(const evt_gatt_error_resp* resp) {
            return AttErrorResponse(
                static_cast<AttributeOpcode::Code>(resp->req_opcode),
                resp->attr_handle,
                resp->error_code
            );
        }
    };

    /**
     * Placeholder used to convert BlueNRG error codes into ble_error_t.
     * The operator << of this instance is overloaded and handle the conversion.
     * It become possible to write code like:
     * @code
     * ble_error_t some_function() {
     *   return _error_converter << aci_do_something();
     * }
     * @endcode
     */
    static struct {
        ble_error_t operator<<(tBleStatus status) {
            PRINTF("ERROR code=0x%x\n", status);
            switch (status) {
                case BLE_STATUS_SUCCESS: return BLE_ERROR_NONE;
                // TODO: map other kind of errors
                // FIXME: the current version of the BlueNRG FW (7.2c)
                // sometimes returns TIMEOUT (0xFF). Ignore the error by now...
                default:                 return BLE_ERROR_NONE;//return BLE_ERROR_UNSPECIFIED;
            }
        }
    } _error_converter;
};

} /* namespace st */
} /* namespace pal */
} /* namespace ble */

#endif /* ST_BLUENRGATTCLIENT_H_ */
