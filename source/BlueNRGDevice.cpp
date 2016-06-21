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
  * @file    BlueNRGDevice.cpp 
  * @author  STMicroelectronics
  * @brief   Implementation of BLEDeviceInstanceBase
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
  
/** @defgroup BlueNRGDevice
 *  @brief BlueNRG BLE_API Device Adaptation
 *  @{
 */
 
#include "mbed-drivers/mbed.h"
#include "BlueNRGDevice.h"
#include "BlueNRGGap.h"
#include "BlueNRGGattServer.h"

#include "btle.h"
#include "Utils.h"
#include "osal.h"

#include "debug.h"
#include "stm32_bluenrg_ble.h"

extern "C" {
    #include "hci.h"
    #include "bluenrg_utils.h"
}

#define HEADER_SIZE 5
#define MAX_BUFFER_SIZE 255

/**
 * The singleton which represents the BlueNRG transport for the BLEDevice.
 *
 * See file 'x_nucleo_idb0xa1_targets.h' for details regarding the peripheral pins used!
 */
#include "x_nucleo_idb0xa1_targets.h"

BlueNRGDevice bluenrgDeviceInstance(IDB0XA1_PIN_SPI_MOSI,
                                    IDB0XA1_PIN_SPI_MISO,
                                    IDB0XA1_PIN_SPI_SCK,
                                    IDB0XA1_PIN_SPI_nCS,
                                    IDB0XA1_PIN_SPI_RESET,
                                    IDB0XA1_PIN_SPI_IRQ);

/**
* BLE-API requires an implementation of the following function in order to
* obtain its transport handle.
*/
BLEInstanceBase *
createBLEInstance(void)
{
    return (&bluenrgDeviceInstance);
}

/**************************************************************************/
/**
    @brief  Constructor
     * @param mosi mbed pin to use for MOSI line of SPI interface
     * @param miso mbed pin to use for MISO line of SPI interface
     * @param sck mbed pin to use for SCK line of SPI interface
     * @param cs mbed pin to use for not chip select line of SPI interface
     * @param rst mbed pin to use for BlueNRG reset
     * @param irq mbed pin for BlueNRG IRQ
*/
/**************************************************************************/
BlueNRGDevice::BlueNRGDevice(PinName mosi,
                             PinName miso,
                             PinName sck,
                             PinName cs,
                             PinName rst,
                             PinName irq) : 
	isInitialized(false), spi_(mosi, miso, sck), nCS_(cs), rst_(rst), irq_(irq)
{
    // Setup the spi for 8 bit data, low clock polarity,
    // 1-edge phase, with an 8MHz clock rate
    spi_.format(8, 0);
    spi_.frequency(8000000);
	
    // Deselect the BlueNRG chip by keeping its nCS signal high
    nCS_ = 1;

    wait_us(500);

    // Prepare communication between the host and the BlueNRG SPI interface
    HCI_Init();

    // Set the interrupt handler for the device
    irq_.mode(PullDown); // set irq mode
    irq_.rise(&HCI_Isr);
}

/**************************************************************************/
/**
    @brief  Destructor
*/
/**************************************************************************/
BlueNRGDevice::~BlueNRGDevice(void)
{
}

/**
  * @brief  Get BlueNRG HW version in bootloader mode
  * @param  hw_version The HW version is written to this parameter
  * @retval It returns BLE_STATUS_SUCCESS on success or an error code otherwise
  */
uint8_t BlueNRGDevice::getUpdaterHardwareVersion(uint8_t *hw_version)
{
	uint8_t status;

	status = getBlueNRGUpdaterHWVersion(hw_version);

	return (status);
}

/**
  * @brief  Flash a new firmware using internal bootloader.
  * @param  fw_image     Pointer to the firmware image (raw binary data,
  *                      little-endian).
  * @param  fw_size      Size of the firmware image. The firmware image size shall
  *                      be multiple of 4 bytes.
  * @retval int      It returns BLE_STATUS_SUCCESS on success, or a number
  *                  not equal to 0 in case of error
  *                  (ACI_ERROR, UNSUPPORTED_VERSION, WRONG_IMAGE_SIZE, CRC_ERROR)
  */
int BlueNRGDevice::updateFirmware(const uint8_t *fw_image, uint32_t fw_size)
{
	int status = program_device(fw_image, fw_size);
	
	return (status);
}


/**
  * @brief  Initialises anything required to start using BLE
  * @param[in] instanceID
  *              The ID of the instance to initialize.
  * @param[in] callback
  *              A callback for when initialization completes for a BLE
  *              instance. This is an optional parameter set to NULL when not
  *              supplied.
  *
  * @return BLE_ERROR_NONE if the initialization procedure was started
  *         successfully.
  */
ble_error_t BlueNRGDevice::init(BLE::InstanceID_t instanceID, FunctionPointerWithContext<BLE::InitializationCompleteCallbackContext *> callback)
{
	if (isInitialized) {
        	BLE::InitializationCompleteCallbackContext context = {
        	    BLE::Instance(instanceID),
        	    BLE_ERROR_ALREADY_INITIALIZED
        	};
        	callback.call(&context);
        	return BLE_ERROR_ALREADY_INITIALIZED;
    	}
	
	/* ToDo: Clear memory contents, reset the SD, etc. */
	// Init the BlueNRG/BlueNRG-MS stack
	// By default, we set the device GAP role to PERIPHERAL
	btleInit(BlueNRGGap::getInstance().getIsSetAddress(), GAP_PERIPHERAL_ROLE_IDB04A1);
	
	isInitialized = true;
	BLE::InitializationCompleteCallbackContext context = {
	        BLE::Instance(instanceID),
	        BLE_ERROR_NONE
	};
	callback.call(&context);
    
	return BLE_ERROR_NONE;
}


/**
    @brief  Resets the BLE HW, removing any existing services and
            characteristics
    @param[in] void
    @returns    void
*/
void BlueNRGDevice::reset(void)
{
    wait_us(500);

    /* Reset BlueNRG SPI interface */
	  rst_ = 0;
  	wait_us(5);
	  rst_ = 1;
  	wait_us(5);

    /* Wait for the radio to come back up */
    wait_us(500);

}

/*!
  @brief  Wait for any BLE Event like BLE Connection, Read Request etc.    
  @param[in] void
  @returns    char *      
*/
void BlueNRGDevice::waitForEvent(void)
{
	bool must_return = false;

	do {
		BlueNRGGap::getInstance().Process();
		
		HCI_Process();
		
		if(must_return) return;

		__WFE(); /* it is recommended that SEVONPEND in the
			    System Control Register is NOT set */
		must_return = true; /* after returning from WFE we must guarantee
				       that conrol is given back to main loop before next WFE */
	} while(true);

} 
 
/*!
    @brief  get GAP version
    @brief Get the BLE stack version information
    @param[in] void
    @returns    char *
    @returns char *
*/
const char *BlueNRGDevice::getVersion(void)
{
    return getVersionString();
}

/**************************************************************************/
/*!
    @brief  get reference to GAP object
    @param[in] void
    @returns    Gap&      
*/
/**************************************************************************/
Gap        &BlueNRGDevice::getGap()        
{
    return BlueNRGGap::getInstance();
}

const Gap  &BlueNRGDevice::getGap() const        
{
    return BlueNRGGap::getInstance();
}

/**************************************************************************/
/*!
    @brief  get reference to GATT server object
    @param[in] void
    @returns    GattServer&    
*/
/**************************************************************************/
GattServer &BlueNRGDevice::getGattServer() 
{
    return BlueNRGGattServer::getInstance();
}

const GattServer &BlueNRGDevice::getGattServer() const
{
    return BlueNRGGattServer::getInstance();
}
 
/**************************************************************************/
/*!
    @brief  shut down the BLE device
    @param[out] error if any
*/
/**************************************************************************/
ble_error_t  BlueNRGDevice::shutdown(void) {
    if (!isInitialized) {
        return BLE_ERROR_INITIALIZATION_INCOMPLETE;
    }

    /* Reset the BlueNRG device first */
    reset();

    /* Shutdown the BLE API and BlueNRG glue code */
    ble_error_t error;

    /* GattServer instance */
    error = BlueNRGGattServer::getInstance().reset();
    if (error != BLE_ERROR_NONE) {
       return error;
    }

    /* GattClient instance */
    error = BlueNRGGattClient::getInstance().reset();
    if (error != BLE_ERROR_NONE) {
        return error;
    }

    /* Gap instance */
    error = BlueNRGGap::getInstance().reset();
    if (error != BLE_ERROR_NONE) {
        return error;
    }

    isInitialized = false;

    return BLE_ERROR_NONE;

}
																							
/**
 * @brief  Reads from BlueNRG SPI buffer and store data into local buffer.
 * @param  buffer   : Buffer where data from SPI are stored
 * @param  buff_size: Buffer size
 * @retval int32_t  : Number of read bytes
 */
int32_t BlueNRGDevice::spiRead(uint8_t *buffer, uint8_t buff_size)
{
  uint16_t byte_count;
  uint8_t len = 0;
  uint8_t char_ff = 0xff;
  volatile uint8_t read_char;
	
	uint8_t i = 0;
	volatile uint8_t tmpreg;

  uint8_t header_master[HEADER_SIZE] = {0x0b, 0x00, 0x00, 0x00, 0x00};
  uint8_t header_slave[HEADER_SIZE];

  /* Select the chip */
  nCS_ = 0;
	
  /* Read the header */  
  for (i = 0; i < 5; i++)
  { 
		tmpreg = spi_.write(header_master[i]);
		header_slave[i] = (uint8_t)(tmpreg);
  } 
	
  if (header_slave[0] == 0x02) {
    /* device is ready */
    byte_count = (header_slave[4]<<8)|header_slave[3];
  
    if (byte_count > 0) {
  
      /* avoid to read more data that size of the buffer */
      if (byte_count > buff_size){
        byte_count = buff_size;
      }
  
      for (len = 0; len < byte_count; len++){
        read_char = spi_.write(char_ff);
				buffer[len] = read_char;
      }
    }    
  }
  /* Release CS line to deselect the chip */
  nCS_ = 1;
	
  // Add a small delay to give time to the BlueNRG to set the IRQ pin low
  // to avoid a useless SPI read at the end of the transaction
  for(volatile int i = 0; i < 2; i++)__NOP();
  
#ifdef PRINT_CSV_FORMAT
  if (len > 0) {
    print_csv_time();
    for (int i=0; i<len; i++) {
      PRINT_CSV(" %02x", buffer[i]);
    }
    PRINT_CSV("\n");
  }
#endif
  
  return len;   
}

/**
 * @brief  Writes data from local buffer to SPI.
 * @param  data1    : First data buffer to be written
 * @param  data2    : Second data buffer to be written
 * @param  Nb_bytes1: Size of first data buffer to be written
 * @param  Nb_bytes2: Size of second data buffer to be written
 * @retval Number of read bytes
 */
int32_t BlueNRGDevice::spiWrite(uint8_t* data1,
				uint8_t* data2, uint8_t Nb_bytes1, uint8_t Nb_bytes2)
{  
	int32_t result = 0;
	uint32_t i;
	volatile uint8_t tmpreg;
    
  unsigned char header_master[HEADER_SIZE] = {0x0a, 0x00, 0x00, 0x00, 0x00};
  unsigned char header_slave[HEADER_SIZE]  = {0xaa, 0x00, 0x00, 0x00, 0x00};

  disable_irq();

  /* CS reset */
  nCS_ = 0;

  /* Exchange header */  
  for (i = 0; i < 5; i++)
  { 
		tmpreg = spi_.write(header_master[i]);
		header_slave[i] = tmpreg;
  } 
	
  if (header_slave[0] == 0x02) {
    /* SPI is ready */
    if (header_slave[1] >= (Nb_bytes1+Nb_bytes2)) {
  
      /*  Buffer is big enough */
			for (i = 0; i < Nb_bytes1; i++) {
				spi_.write(*(data1 + i));
      }
      for (i = 0; i < Nb_bytes2; i++) {
				spi_.write(*(data2 + i));
      }			
    } else {
      /* Buffer is too small */
      result = -2;
    }
  } else {
    /* SPI is not ready */
    result = -1;
  }
    
  /* Release CS line */
  //HAL_GPIO_WritePin(BNRG_SPI_CS_PORT, BNRG_SPI_CS_PIN, GPIO_PIN_SET);
  nCS_ = 1;
			
  enable_irq();
    
  return result;
}

bool BlueNRGDevice::dataPresent()
{
    return (irq_ == 1);
}

void BlueNRGDevice::disable_irq()
{
    irq_.disable_irq();
}

void BlueNRGDevice::enable_irq()
{
    irq_.enable_irq();
}
