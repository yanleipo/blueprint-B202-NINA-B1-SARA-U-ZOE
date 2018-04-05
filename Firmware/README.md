# B202 demo software

## General behavior

* Bluetooth address and RSSI value of remote Bluetooth Low Energy devices found during a 5 seconds scan is sent to the a cloud service via the cellular module using http post.

## Program the device

Precompiled firmware is provided and can be used with B202 hardware. The .hex file is available in the ```/hex``` folder.   
**Please note!** This firmware requires SoftDevice s132_nrf52_5.0.0 to be programmed to the device.

## Compile the software

* Clone this repository   
```> git clone https://github.com/u-blox/B202-NINA-B1-SARA-U-ZOE```

* Download the source code for nRF5 SDK version 14.2.0 from Nordic Semi

* Copy ```custom_board.h``` to the ```/components/boards``` folder of the SDK

* Copy the folder ```src``` to the ```/example``` folder of the SDK

* Open the SEGGER Embedded Studio project file located in ```src\ninab1\s132\ses\```