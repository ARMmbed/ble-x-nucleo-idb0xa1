# BLE-X-NUCLEO-IDB0XA1

BLE_API wrapper Library for X-NUCLEO-IDB05A1 BlueNRG (Bluetooth Low Energy) Expansion Board

## Introduction

This firmware package implements the port of BLE_API to STMicroelectronics' [X-NUCLEO-IDB05A1](https://developer.mbed.org/components/X-NUCLEO-IDB05A1-Bluetooth-Low-Energy/) Bluetooth Low Energy Nucleo Expansion Board.

### Arduino Connector Compatibility Warning

X-NUCLEO-IDB05A1 is Arduino compatible with an exception: instead of using pin **D13** for the SPI clock, pin **D3** is used.
The default configuration for this library is having the SPI clock on pin **D3**.

To be fully Arduino compatible, X-NUCLEO-IDB05A1 needs a small HW patch.

For X-NUCLEO-IDB05A1 this patch consists in removing zero resistor **R4** and instead soldering zero resistor **R6**.

In case you patch your board, then you also have to configure this library to use pin **D13** to drive the SPI clock. To this aim you need to compile this driver with macro `BLUENRG_PIN_SPI_SCK=D13` defined.

If you use pin **D13** for the SPI clock, please be aware that on STM32 Nucleo boards you may **not** drive the LED, otherwise you will get a conflict: the LED on STM32 Nucleo boards is connected to pin **D13**.

Referring to the current list of tested platforms (see [X-NUCLEO-IDB05A1](https://developer.mbed.org/components/X-NUCLEO-IDB05A1-Bluetooth-Low-Energy/) page), the patch is required by [ST-Nucleo-F103RB](https://developer.mbed.org/platforms/ST-Nucleo-F103RB/); [ST-Nucleo-F302R8](https://developer.mbed.org/platforms/ST-Nucleo-F302R8/); [ST-Nucleo-F411RE](https://developer.mbed.org/platforms/ST-Nucleo-F411RE/); [ST-Nucleo-F446RE](https://developer.mbed.org/platforms/ST-Nucleo-F446RE/); and [FRDM-K64F](https://developer.mbed.org/platforms/FRDM-K64F/).

### Firmware update

For better performance and compatibility with latest mbed API, you should update firmware of X-NUCLEO-IDB05A1 component by using this simple [application](https://developer.mbed.org/teams/ST/code/BlueNRG-MS-Stack-Updater).

### Driver configuration

In order to use the BlueNRG-MS module together with other targets, you need to set the macros defined in file [bluenrg_targets.h](https://github.com/ARMmbed/ble-x-nucleo-idb0xa1/blob/master/bluenrg/bluenrg_targets.h). Please, update the [mbed_lib.json](https://github.com/ARMmbed/ble-x-nucleo-idb0xa1/blob/master/mbed_lib.json) to include the list of extra macros that configure the driver for your target.

## Example Applications

To run BLE example applications using X-NUCLEO-IDB05A1 Expansion Board based on mbed OS and built with [mbed-cli](https://github.com/ARMmbed/mbed-cli), please refer to section [Using ST shield on other targets](https://github.com/ARMmbed/mbed-os-example-ble#using-st-nucleo-shield-on-other-targets) in the official [mbed-os-example-ble](https://github.com/ARMmbed/mbed-os-example-ble) page.
