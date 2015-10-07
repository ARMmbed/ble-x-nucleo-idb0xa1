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

#include "mbed.h"

#ifndef __PAYLOAD_H__
#define __PAYLOAD_H__

class UnitPayload
{
public:
    uint8_t length;
    uint8_t id;    
    uint8_t *data;
    uint8_t *idptr;    

    
    
    void set_length(uint8_t l) {
        length=l;   
    }

    void set_id(uint8_t i) {
        id=i;
    }

    void set_data(uint8_t* data1) {
        for(int j=0;j<length;j++)
        {
            data[j]=data1[j];
        }   
    }

    uint8_t get_length()  {
        return length;      
    }

    uint8_t get_id()  {
        return id;      
    }

    uint8_t* get_data() {
        return data;        
    }

};

class Payload {
    UnitPayload *payload;
    int stringLength;
    int payloadUnitCount;
    
public:
    Payload(const uint8_t *tokenString, uint8_t string_ength);
    Payload();
    uint8_t getPayloadUnitCount();
    
    uint8_t getIDAtIndex(int index);  
    uint8_t getLengthAtIndex(int index);   
    uint8_t* getDataAtIndex(int index);    
    int8_t getInt8AtIndex(int index);  
    uint16_t getUint16AtIndex(int index); 
    uint8_t* getSerializedAdDataAtIndex(int index);         
};


class PayloadUnit {
private:        
    uint8_t* lenPtr;
    uint8_t* adTypePtr;
    uint8_t* dataPtr;
    
public:
    PayloadUnit() {
        lenPtr = NULL;
        adTypePtr = NULL;
        dataPtr = NULL;
    }
    
    PayloadUnit(uint8_t *len, uint8_t *adType, uint8_t* data) {
        lenPtr = len;
        adTypePtr = adType;
        dataPtr = data;
    }    
    
    void setLenPtr(uint8_t *len)   {
        lenPtr = len;
    } 
    
    void setAdTypePtr(uint8_t *adType)   {
        adTypePtr = adType;
    }  
    
    void setDataPtr(uint8_t *data)   {
        dataPtr = data;
    }      
    
    uint8_t* getLenPtr()   {
        return lenPtr;
    } 
    
    uint8_t* getAdTypePtr()   {
        return adTypePtr;
    }  
    
    uint8_t* getDataPtr()   {
        return dataPtr;
    }   
    
    void printDataAsHex()   {
        int i = 0;
        printf("AdData=");
        for(i=0; i<*lenPtr-1; i++) {
            printf("0x%x ", dataPtr[i]);
        }
        printf("\n");
    }     
    
    void printDataAsString()   {
        int i = 0;
        printf("AdData=");
        for(i=0; i<*lenPtr; i++) {
            printf("%c", dataPtr[i]);
        }
        printf("\n");
    }                                                   
    
};

class PayloadPtr {
private:
    PayloadUnit *unit;
    int payloadUnitCount;
public:    
    PayloadPtr(const uint8_t *tokenString, uint8_t string_ength) {
        // initialize private data members
        int stringLength = string_ength;
        payloadUnitCount = 0;
        
        int index = 0;
        while(index!=stringLength) {
            int len=tokenString[index];
            index=index+1+len;
            payloadUnitCount++;               
        }
        
        // allocate memory to unit
        unit = new PayloadUnit[payloadUnitCount];
        int i = 0;
        int nextUnitOffset = 0;
        
        while(i<payloadUnitCount) {           
            unit[i].setLenPtr((uint8_t *)tokenString+nextUnitOffset);
            unit[i].setAdTypePtr((uint8_t *)tokenString+nextUnitOffset+1);
            unit[i].setDataPtr((uint8_t *)tokenString+nextUnitOffset+2);
            
            nextUnitOffset += *unit[i].getLenPtr()+1;
            i++;

        }
    }
    
    PayloadUnit getUnitAtIndex(int index) {
        return unit[index];
    }
    
    int getPayloadUnitCount() { return payloadUnitCount; }
    
    
};    

#endif // __PAYLOAD_H__
