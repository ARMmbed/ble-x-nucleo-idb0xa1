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

#include "Utils.h"

#if NEED_CONSOLE_OUTPUT
Serial  pc(USBTX, USBRX);
#endif /* #if NEED_CONSOLE_OUTPUT */

/**************************************************************************/
/*!
    @brief  sets values of EN_HIGH_POWER and PA_LEVEL corresponding to dBMLevel of tx power

    @returns    value of tx power in dbm actually set

    @params[in] dBMLevel
                dBMLevel of tx power to be set
                
    @params[in] dBMLevel
                dBMLevel of tx power to be set                
                
    @endcode
*/
/**************************************************************************/
double getHighPowerAndPALevelValue(int8_t dBMLevel, int8_t& EN_HIGH_POWER, int8_t& PA_LEVEL) {
    double dbm = (double) dBMLevel;
    if(dbm<-18.0) {
        dbm = -18;
        EN_HIGH_POWER = 0;
        PA_LEVEL = 0;         
    }
    else if(dbm>8.0) {
        dbm = 8;
        EN_HIGH_POWER = 1;
        PA_LEVEL = 7;        
    }
    
    // As a policy we are setting tx power level to the higher side
    if((dbm>-18.0) && (dbm<=-15)) {
        // set tx power to -15dBM
        EN_HIGH_POWER = 0;
        PA_LEVEL = 0;
    }
    else if((dbm>-15) && (dbm<=-14.7)) {
        // set tx power to -14.7dBM
        EN_HIGH_POWER = 0;
        PA_LEVEL = 1;         
    }
    else if((dbm>-14.7) && (dbm<=-11.7)) {
        // set tx power to -11.7dBM
        EN_HIGH_POWER = 1;
        PA_LEVEL = 1;            
    }   
    else if((dbm>-11.7) && (dbm<=-11.4)) {
        // set tx power to -11.4dBM
        EN_HIGH_POWER = 0;
        PA_LEVEL = 2;            
    }
    else if((dbm>-11.4) && (dbm<=-8.4)) {
        // set tx power to -8.4dBM
        EN_HIGH_POWER = 1;
        PA_LEVEL = 2;           
    }  
    else if((dbm>-8.4) && (dbm<=-8.1)) {
        // set tx power to -8.1dBM
        EN_HIGH_POWER = 0;
        PA_LEVEL = 3;           
    }
    else if((dbm>-8.1) && (dbm<=-5.1)) {
        // set tx power to -5.1dBM
        EN_HIGH_POWER = 1;
        PA_LEVEL = 3;          
    }   
    else if((dbm>-5.1) && (dbm<=-4.9)) {
        // set tx power to -4.9dBM
        EN_HIGH_POWER = 0;
        PA_LEVEL = 4;            
    }
    else if((dbm>-4.9) && (dbm<=-2.1)) {
        // set tx power to -2.1dBM
        EN_HIGH_POWER = 1;
        PA_LEVEL = 4;          
    }
    else if((dbm>-2.1) && (dbm<=-1.6)) {
        // set tx power to -1.6dBM
        EN_HIGH_POWER = 0;
        PA_LEVEL = 5;           
    }
    else if((dbm>-1.6) && (dbm<=1.4)) {
        // set tx power to -1.6dBM
        EN_HIGH_POWER = 1;
        PA_LEVEL = 5;           
    }   
    else if((dbm>1.4) && (dbm<=1.7)) {
        // set tx power to 1.7dBM
        EN_HIGH_POWER = 0;
        PA_LEVEL = 6;            
    }
    else if((dbm>1.7) && (dbm<=4.7)) {
        // set tx power to 4.7dBM
        EN_HIGH_POWER = 1;
        PA_LEVEL = 6;            
    }  
    else if((dbm>4.7) && (dbm<=5.0)) {
        // set tx power to 5.0dBM
        EN_HIGH_POWER = 0;
        PA_LEVEL = 7;           
    }
    else if((dbm>5.0) && (dbm<=8)) {
        // set tx power to 8.0dBM
        EN_HIGH_POWER = 1;
        PA_LEVEL = 7;              
    }  
    
    return dbm;         
}
