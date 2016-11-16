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
  * @file    BlueNRGGap.h
  * @author  STMicroelectronics
  * @brief   Header file for BlueNRG BLE_API Gap Class
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

#ifndef __BLUENRG_GAP_H__
#define __BLUENRG_GAP_H__

#ifdef YOTTA_CFG_MBED_OS
    #include "mbed-drivers/mbed.h"
#else
    #include "mbed.h"
#endif 
#include "ble/blecommon.h"
#include "btle.h"
#include "ble/GapAdvertisingParams.h"
#include "ble/GapAdvertisingData.h"
#include "ble/Gap.h"

#define BLE_CONN_HANDLE_INVALID 0x0
#define BDADDR_SIZE 6

#define BLUENRG_GAP_ADV_INTERVAL_MIN (0x0020)
#define BLUENRG_GAP_ADV_INTERVAL_MAX (0x4000)
#define BLUENRG_GAP_ADV_NONCON_INTERVAL_MIN (0x00A0)

// Scanning and Connection Params used by Central for creating connection
#define GAP_OBSERVATION_PROC (0x80)

#define SCAN_P         (0x0010)
#define SCAN_L         (0x0010)
#define SUPERV_TIMEOUT (0xC80)
#define CONN_P(x)      ((int)((x)/1.25f))
#define CONN_L(x)      ((int)((x)/0.625f))
#define CONN_P1        ((int)(_advParams.getInterval()+5)/1.25f)//(0x4C)//(0x6C)
#define CONN_P2        ((int)(_advParams.getInterval()+5)/1.25f)//(0x4C)//(0x6C)
#define CONN_L1        (0x0008)
#define CONN_L2        (0x0008)
#define GUARD_INT      5 //msec
#define MIN_INT_CONN   0x0006 //=>7.5msec
#define MAX_INT_CONN   0x0C80 //=>4000msec
#define DEF_INT_CONN   0x0140 //=>400msec (default value for connection interval)

/**************************************************************************/
/*!
    \brief

*/
/**************************************************************************/
class BlueNRGGap : public Gap
{
public:
    static BlueNRGGap &getInstance() {
        static BlueNRGGap m_instance;
        return m_instance;
    }

    enum Reason_t {
        DEVICE_FOUND,
        DISCOVERY_COMPLETE
    };

    /* Functions that must be implemented from Gap */
    virtual ble_error_t setAddress(addr_type_t type, const BLEProtocol::AddressBytes_t address);
    virtual ble_error_t getAddress(BLEProtocol::AddressType_t *typeP, BLEProtocol::AddressBytes_t address);
    virtual ble_error_t setAdvertisingData(const GapAdvertisingData &, const GapAdvertisingData &);
    virtual ble_error_t startAdvertising(const GapAdvertisingParams &);
    virtual ble_error_t stopAdvertising(void);
    virtual ble_error_t stopScan();
    virtual uint16_t    getMinAdvertisingInterval(void) const {return GapAdvertisingParams::ADVERTISEMENT_DURATION_UNITS_TO_MS(BLUENRG_GAP_ADV_INTERVAL_MIN);}
    virtual uint16_t    getMinNonConnectableAdvertisingInterval(void) const {return GapAdvertisingParams::ADVERTISEMENT_DURATION_UNITS_TO_MS(BLUENRG_GAP_ADV_NONCON_INTERVAL_MIN);}
    virtual uint16_t    getMaxAdvertisingInterval(void) const {return GapAdvertisingParams::ADVERTISEMENT_DURATION_UNITS_TO_MS(BLUENRG_GAP_ADV_INTERVAL_MAX);}
    virtual ble_error_t disconnect(DisconnectionReason_t reason);
    virtual ble_error_t disconnect(Handle_t connectionHandle, DisconnectionReason_t reason);
    virtual ble_error_t getPreferredConnectionParams(ConnectionParams_t *params);
    virtual ble_error_t setPreferredConnectionParams(const ConnectionParams_t *params);
    virtual ble_error_t updateConnectionParams(Handle_t handle, const ConnectionParams_t *params);

    virtual ble_error_t setDeviceName(const uint8_t *deviceName);
    virtual ble_error_t getDeviceName(uint8_t *deviceName, unsigned *lengthP);
    virtual ble_error_t setAppearance(GapAdvertisingData::Appearance appearance);
    virtual ble_error_t getAppearance(GapAdvertisingData::Appearance *appearanceP);

    virtual ble_error_t setScanningPolicyMode(ScanningPolicyMode_t mode);
    virtual ble_error_t setAdvertisingPolicyMode(AdvertisingPolicyMode_t mode);
    virtual AdvertisingPolicyMode_t getAdvertisingPolicyMode(void) const;
    virtual ScanningPolicyMode_t getScanningPolicyMode(void) const;

    virtual ble_error_t setTxPower(int8_t txPower);
    virtual void        getPermittedTxPowerValues(const int8_t **, size_t *);

    virtual ble_error_t connect(const Address_t peerAddr,
                                Gap::AddressType_t peerAddrType,
                                const ConnectionParams_t *connectionParams,
                                const GapScanningParams *scanParams);

    virtual ble_error_t reset(void);

    void                Discovery_CB(Reason_t reason,
                                     uint8_t adv_type,
                                     uint8_t addr_type,
                                     uint8_t *addr,
                                     uint8_t *data_length,
                                     uint8_t *data,
                                     uint8_t *RSSI);
    ble_error_t         createConnection(void);

    void     setConnectionHandle(uint16_t con_handle);
    uint16_t getConnectionHandle(void);

    bool getIsSetAddress();

    // ADV timeout handling
    Timeout getAdvTimeout(void) const {
        return advTimeout;
    }
    uint8_t getAdvToFlag(void) {
        return AdvToFlag;
    }
    void setAdvToFlag(void);

    // SCAN timeout handling
    Timeout getScanTimeout(void) const {
        return scanTimeout;
    }
    uint8_t getScanToFlag(void) {
        return ScanToFlag;
    }
    void setScanToFlag(void);

    void Process(void);

    GapScanningParams* getScanningParams(void);

    virtual ble_error_t startRadioScan(const GapScanningParams &scanningParams);

    void setConnectionInterval(uint16_t interval);
    void setGapRole(Role_t role);

private:
    uint16_t m_connectionHandle;
    Role_t gapRole;
    AddressType_t addr_type;
    Address_t _peerAddr;
    AddressType_t _peerAddrType;
    uint8_t bdaddr[BDADDR_SIZE];
    bool _scanning;
    bool _connecting;
    bool isSetAddress;
    uint8_t deviceAppearance[2];

    // ADV timeout handling
    Timeout advTimeout;
    bool AdvToFlag;

    // SCAN timeout handling
    Timeout scanTimeout;
    bool ScanToFlag;

    static uint16_t SCAN_DURATION_UNITS_TO_MSEC(uint16_t duration) {
        return (duration * 625) / 1000;
    }

    uint16_t scanInterval;
    uint16_t scanWindow;
    uint16_t advInterval;
    uint16_t slaveConnIntervMin;
    uint16_t slaveConnIntervMax;
    uint16_t conn_min_interval;
    uint16_t conn_max_interval;
    void setAdvParameters(void);
    void setConnectionParameters(void);

    Gap::AdvertisingPolicyMode_t advertisingPolicyMode;
    Gap::ScanningPolicyMode_t scanningPolicyMode;

    Whitelist_t whitelistAddresses;

    ble_error_t updateAdvertisingData(void);

    BlueNRGGap() : AdvToFlag(false), ScanToFlag(false) {
        m_connectionHandle = BLE_CONN_HANDLE_INVALID;
        addr_type = BLEProtocol::AddressType::RANDOM_STATIC;

        /* Set the whitelist policy filter modes to IGNORE_WHITELIST */
        advertisingPolicyMode = Gap::ADV_POLICY_IGNORE_WHITELIST;
        scanningPolicyMode    = Gap::SCAN_POLICY_IGNORE_WHITELIST;

        isSetAddress = false;
        memset(deviceAppearance, 0, sizeof(deviceAppearance));
    }

    BlueNRGGap(BlueNRGGap const &);
    void operator=(BlueNRGGap const &);

    GapAdvertisingData _advData;
    GapAdvertisingData _scanResponse;
};

#endif // ifndef __BLUENRG_GAP_H__
