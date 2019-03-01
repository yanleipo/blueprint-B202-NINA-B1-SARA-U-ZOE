# B202 demo software

## General behavior

Track B202 location and push location information to thingsboard server. Location and temperature information can be viewed at http://ec2-18-224-182-187.us-east-2.compute.amazonaws.com:8080/dashboard/6f068ff0-3b34-11e9-b693-1dfede625bdd?publicId=5a9a5500-3b35-11e9-b693-1dfede625bdd . 

Blue LED will flash every 3 seconds after B202 register to the network and start pushing data to the cloud. 

For new tracker to be added, a new B202 device need to be created in thingsboard server. The IMEI information of the SARA-U2 need to be entered in the as device credentials. You may contact leipo.yan@u-blox.com to add your device.

Default APN is "internet". It's possible to change the APN before SARA-U2 tries to register to the network (30 seconds within bootup). This can be done using nRF Connect software. Scan and connect to device B202-xxxx. In the Nordic UART service, write text "apn=e-ideas" (without quiote) to Nordic UART RX to change the APN to "e-ideas". 

It is possible to use EVK-NINA-B1 to wirelessly connect to B202 and debug SARA-U2 over UART. To do so:
* Flash s132_nrf52_6.1.0_softdevice.hex and ble_app_uart_c_pca10040_s132.hex to EVK-NINA-B1. 
* Make sure that B202 is not connected to other BLE devices. Reboot EVK-NINA-B1. EVK-NINA-B1 will be automatically connect to B202 over NUS service.
* Use serial terminal client to open the com port of EVK-NINA-B1 (115200, 8N1, no flow control). Enter "dbg-cel=1" (without quote) and press enter. Then the AT command issued on the terminal will be forwarded to SARA-U2. Response will be shown on the serial terminal as well.


## Program the device

Precompiled firmware is provided and can be used with B202 hardware. The .hex file is available in the ```/hex``` folder.   
Flash s132_nrf52_6.1.0_softdevice.hex and b202_s132.hex to B202.

## Compile the software

* Clone this repository   
```> git clone https://github.com/u-blox/B202-NINA-B1-SARA-U-ZOE```

* Download the source code for nRF5 SDK version 15.2.0 from Nordic Semi

* Copy ```custom_board.h``` to the ```/components/boards``` folder of the SDK

* Copy the folder ```Firmware``` to the ```/example``` folder of the SDK

* Open the SEGGER Embedded Studio project file located in ```src\ninab1\s132\ses\```

## Known issues
There is a problem reading temperature from LSM6DSL sensor. It's always 0. Value of WHO_AM_I (0x0F) register is 0x69 instead of 0x6A.

