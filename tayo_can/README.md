# Kvaser ROS Interface API
The Kvaser ROS Interface API is based on the Autonomous Stuff Code (https://github.com/astuff/kvaser_interface). Kvser_interface is modified to enable CANFD communication.

This package was developed as a standardized way to access Kvaser CAN devices from ROS. It can either be used as a development API by including the header <kvaser_interface/kvaser_interface.h> and linking against `libros_linuxcan.so` or the stand-alone node `kvaser_can_bridge` can communicate with a CAN device independently.

Installation
The `kvaser_interface` package depends on the Kvaser CANLIB API. You can install the Kvaser CANLIB from source [directly from Kvaser](https://www.kvaser.com/downloads/), however the easiest way to install is using our ppa which distributes them as deb packages:
```bash
sudo apt-add-repository ppa:astuff/kvaser-linux
sudo apt update
sudo apt install kvaser-canlib-dev kvaser-drivers-dkms
```
Now that the dependencies are installed, we can install kvaser_interface

## The `kvaser_can_bridge` Node

**TOPICS**

*can_tx* [can_msgs::Frame]
*can_tx* [can_msgs::FrameFd]

This topic is published by the node. It expects to have other nodes subscribe to it to receive data which are *sent by the CAN device*.

*can_rx* [can_msgs::Frame]
*can_rx* [can_msgs::FrameFd]

This topic is subscribed to by the node. It expects to have data published to it which are intended to be *received by the CAN device*.

**PARAMETERS**

*~can_hardware_id*

This is the Kvaser Hardware ID (serial number) of the connected device.

*~can_circuit_id*

This is the 0-based index of the channel number *on the specific hardware device* designated by the *~can_hardware_id*.

*~can_bit_rate*

This is the communication rate to be used on the CAN channel in bits per second (default: 500000).

*~canfd*

This is the canfd flag (0 : not canfd, 1 : canfd).

*~canfd_tseg1*

Time segment 1, that is, the number of quanta from (but not including) the Sync Segment to the sampling point (default: 15).

*~canfd_tseg2*

Time segment 2, that is, the number of quanta from the sampling point to the end of the bit (default: 4).

*~canfd_sjw*

The Synchronization Jump Width (default: 4).

*~canfd_data_rate*

The canfd data rate (defalut : 2000000)

