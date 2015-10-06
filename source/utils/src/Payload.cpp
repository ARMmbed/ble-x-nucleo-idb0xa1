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

#include <Payload.h>   

Payload::Payload() {
    stringLength = 0;
    payloadUnitCount = 0;
    payload = NULL;
}

Payload::Payload(const uint8_t *tokenString, uint8_t string_ength) {
    // initialize private data members
    stringLength = string_ength;
    payloadUnitCount = 0;
    payload = NULL;
    
    int index = 0;
    while( index!=stringLength) {
        int len=tokenString[index];
        index=index+1+len;
        payloadUnitCount++;               
    }
    
    UnitPayload *obj = new UnitPayload[payloadUnitCount];
    int i=0;
    int c=0;
    int j,k;

    while(i<payloadUnitCount)
    {   
        obj[i].length=tokenString[c];
        obj[i].id=tokenString[c+1];

        obj[i].data = new uint8_t[obj[i].length];
        for(j=c+2,k=0;(j<(c+obj[i].length+1))&&(k<obj[i].length-1);j++,k++)
        {
            obj[i].data[k]=tokenString[j];
            
        }           
        
        c=c+obj[i].length+1;
        i++;

    }
    payload = obj;
}

uint8_t Payload::getPayloadUnitCount() {
    return payloadUnitCount;
}

uint8_t Payload::getIDAtIndex(int index) {
    return payload[index].get_id();
}

uint8_t Payload::getLengthAtIndex(int index) {
    return payload[index].get_length();
}

uint8_t* Payload::getDataAtIndex(int index) {
    return payload[index].get_data();
}

int8_t Payload::getInt8AtIndex(int index) {
    uint8_t* str = payload[index].get_data();
    int8_t value = (int8_t)str[0];
    return value;
}

uint16_t Payload::getUint16AtIndex(int index) {
    uint16_t* str = (uint16_t*)payload[index].get_data();
    uint16_t value = str[0];
    return value;    
}

uint8_t* Payload::getSerializedAdDataAtIndex(int index) {
    uint8_t length = payload[index].get_length();
    uint8_t* data = payload[index].get_data();
    uint8_t id = payload[index].get_id();
    uint8_t *serializedAdData = new uint8_t[length];
    
    serializedAdData[0] = id;
    for(int i=0; i<length-1; i++) {
        serializedAdData[i+1] = data[i];
    }
    return serializedAdData;
}  
