# record_pcap_linux

Demo to capture Ethernet packets from selected device on Linux, and save packets to a ***.pcap** file.

## 1.Requirements

1) libpcap
2) cmake 3.23 or higher

to install *libpcap* :

```
sudo apt install libpcap-dev
```

## 2.Usage

Excute:

```
cd benewake_lidar_driver/tools/record_pcap_linux
mkdir build
cd build
cmake .. && make
sudo ./record_pcap
```

To stop capture, press **CRTL + C** when under excution terminal.

## 3.Other

On windows, should use **winpcap** to capture packets. It has simillar functions and procedure with **libpcap**. Please refer to **winpcap**'s documents.