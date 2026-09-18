# Ring_Network
# README.md for Ring Network ( Using RS485).

In the whole Ring network currently participating totally 3 devices.
1. RPI-5
2. STM32 NucleoF407ZG
3. ESP32 S3.

RPI acts a controller of the network. It continuously monitors both  the STM and ESP and sends data to both the devices.
RPI sends data to STM via ESP32.
The complete Ring network is established with RS485 communication.

If any device (ex.. esp) is inactive then forward routing fails and RPI starts  reverse routing and sends data to STM directly.
If devices active again in forward routing then RPI restarts the forward routing.

files:
------
1.Nucleo_Ring_Node --Nucleo board
2.RS_485_S3 ----ESP32S3
3.rs485_tx_new ---RPI5

