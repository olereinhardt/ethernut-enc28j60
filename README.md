# ENC28J60-Driver for the ethernut RTOS

This project aims to create a driver for the popular, low-cost ENC28J60 10Mbit/s Ethernet-Chip for the [ethernut](http://ethernut.de) Real Time Operating System.
While this project should work on any Hardware supported by the ethetnut-OS that has an SPI-bus, it is currently being developed on the WiFiMCU Chip (based on EMW3165, ARM(tm) Cortex M4 Chip).

# Wiring diagram for WiFiMCU
The wiring diagram is provided for the SPI4-Bus, the only one that is fully accessible by the WiFiMCU Pins.
```
WiFiMCU          <----->          ENC28J60
D12 (SCK)                              SCK
D1  (MOSI)                       MOSI (SI)
D14 (MISO)                       MISO (SO)
D4  (NSS)                               CS
```
