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

#include "ble_utils.h"

/**************************************************************************/
/*!
    @brief  sets values of EN_HIGH_POWER and PA_LEVEL corresponding to dBMLevel of tx power

*/
/**************************************************************************/
tBleStatus getHighPowerAndPALevelValue(int8_t dBMLevel, int8_t& EN_HIGH_POWER, int8_t& PA_LEVEL) {
    tBleStatus ret = BLE_STATUS_SUCCESS;

    if(dBMLevel==-18) {
        EN_HIGH_POWER = 0;
        PA_LEVEL = 0;         
    }
    else if(dBMLevel==-15) {
        EN_HIGH_POWER = 0;
        PA_LEVEL = 1;
    }
    else if(dBMLevel==-14) {
        EN_HIGH_POWER = 1;
        PA_LEVEL = 0;
    }
    else if(dBMLevel==-12) {
        EN_HIGH_POWER = 0;
        PA_LEVEL = 2;
    }
    else if(dBMLevel==-11) {
        EN_HIGH_POWER = 1;
        PA_LEVEL = 1;
    }
    else if(dBMLevel==-9) {
        EN_HIGH_POWER = 0;
        PA_LEVEL = 3;
    }
    else if(dBMLevel==-8) {
        EN_HIGH_POWER = 1;
        PA_LEVEL = 2;
    }
    else if(dBMLevel==-6) {
        EN_HIGH_POWER = 0;
        PA_LEVEL = 4;
    }
    else if(dBMLevel==-5) {
        EN_HIGH_POWER = 1;
        PA_LEVEL = 3;
    }
    else if(dBMLevel==-2) {
        EN_HIGH_POWER = 1;
        PA_LEVEL = 4;
    }
    else if(dBMLevel==0) {
        EN_HIGH_POWER = 0;
        PA_LEVEL = 6;
    }
    else if(dBMLevel==2) {
        EN_HIGH_POWER = 1;
        PA_LEVEL = 5;
    }
    else if(dBMLevel==4) {
        EN_HIGH_POWER = 1;
        PA_LEVEL = 6;
    }
    else if(dBMLevel==5) {
        EN_HIGH_POWER = 0;
        PA_LEVEL = 7;
    }
    else if(dBMLevel==8) {
        EN_HIGH_POWER = 1;
        PA_LEVEL = 7;
    }
    else {
        ret = ERR_INVALID_HCI_CMD_PARAMS;
    }

    return ret;
}
