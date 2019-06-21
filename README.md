# UART OTA firmware updater

This firmware updater is based on the native_ota_example by Espressif.
In some cases, it is not possible to use a factory OTA update software and two OTA partitions.
In case of the FLipMouse, we need at least 2MB for the firmware and around 1MB for data (web page, config,...),
so it is not possible to use the normal OTA setup.

This firmware is used as factory image, which can be small if no BT/WiFi is used.
In our case, we assigned 512kB to this update firmware. According to `make size-components`, it should
fit into a 256kB partition.

## Usage

1. Flash this firmware with `make flash monitor`
2. Attach RX/TX/GND pins to a different USB/UART converter (do NOT use UART0!)
3. Compile the command line tool (see folder update_commandline)
3. Start the update utility

## Partition setup

```
#16k NVS
nvs,      data, nvs,     0x9000,  0x4000
#8k OTA infos
otadata,  data, ota,     0xd000,  0x2000
#4k PHY data
phy_init, data, phy,     0xf000,  0x1000

#512k factory (UART Bootloader)
factory,  0,    0,       0x10000, 0x80000

#2MB OTA
ota_0,    0,    ota_0,  0x90000, 0x200000

#1.4MB storage
storage,  data, spiffs,   0x290000,  0x170000
```

## Configuration

Changing the UART pins or the UART number, please adjust following defines:

```
//TX pin
HAL_SERIAL_TXPIN
//RX pin
HAL_SERIAL_RXPIN
//UART unit number
HAL_SERIAL_UART
```


## Contributions

The main OTA example code is provided by Espressif:
https://github.com/espressif/esp-idf/tree/master/examples/system/ota/native_ota_example

The C command line utility for GNU/Linux is mostly based on the serial example code,
provided by wallyk / Gabriel Staples:
https://stackoverflow.com/questions/6947413/how-to-open-read-and-write-from-serial-port-in-c

## License

This software is released under GNU GPL v3 or any later version.
See LICENSE.txt for full license text.
