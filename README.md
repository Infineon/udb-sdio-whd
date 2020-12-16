# UDB SDIO for Wi-Fi Host Driver

### Overview

This library provides a UDB based SDIO interface that allows for communicating between a PSoC 6 and a wireless device such as the CYW4343W, CYW43438, or CYW43012. This library allows PSoC 6 devices that do not have a dedicated SDHC hardware block, but do have UDBs, to work with the [Wi-Fi Host Driver (WHD)](https://github.com/cypresssemiconductorco/wifi-host-driver) library.

**NOTE:** This library does not provide a complete SDIO implementation. It is only intended for use with a wireless device.

**NOTE:** This library is only compatible with PSoC 6 Board Support Packages (BSPs) version 1.2.0 and later. Prior to this version, portions of this library were directly included as part of the BSP.

### Whats included
There are three configurations of this library to choose from based on what PSoC 6 pins are intended for SDIO communication. Selection of which port to use is done by specifying the appropriate component to the makefile. Details for this are described in the Quick Start section below. The table below shows the supported ports and which pins on that port map to which SDIO function.

| SDIO Function | Port 2 | Port 9 | Port 12 |
| ------------- | ------ | ------ | ------- |
| SDIO_D0       | P2_0   | P9_0   | P12_1   |
| SDIO_D1       | P2_1   | P9_1   | P12_2   |
| SDIO_D2       | P2_2   | P9_2   | P12_3   |
| SDIO_D3       | P2_3   | P9_3   | P12_4   |
| SDIO_CMD      | P2_4   | P9_4   | P12_5   |
| SDIO_CLK      | P2_5   | P9_5   | P12_0   |


### Quick Start
1. Update the application or BSP makefile to indicate which port to use for SDIO communication
* For Port 2: ```COMPONENTS+=UDB_SDIO_P2```
* For Port 9: ```COMPONENTS+=UDB_SDIO_P9```
* For Port 12: ```COMPONENTS+=UDB_SDIO_P12```

2. Update the application or BSP makefile to indicate that Wi-Fi is supported
```DEFINES+=CYBSP_WIFI_CAPABLE```

3. Add the WHD and associated wireless libraries to the application

4. Refer to the WHD documentation for setting up the networking interface


### Restrictions
To use this library, the following must be true:
1. ClkSlow & ClkPeri must both run at the same speed
2. ClkSlow & ClkPeri must run at 4x the desired SDIO speed
3. The SDIO clock must run at between 20-25MHz
4. The first 8-bit peripheral clock divider must be reserved for use by this driver
5. The following DMA channels must be reserved for use by this driver
* DataWire 0 channel 0
* DataWire 0 channel 1
* DataWire 1 channel 1
* DataWire 1 channel 3

**NOTE:** The optimal configuration is to have ClkSlow and ClkPeri running at 100 MHz and for the SDIO to run at 25 MHz.

### More information

* [API Reference Guide](https://cypresssemiconductorco.github.io/udb-sdio-whd/html/index.html)
* [Cypress Semiconductor, an Infineon Technologies Company](http://www.cypress.com)
* [Cypress Semiconductor GitHub](https://github.com/cypresssemiconductorco)
* [ModusToolbox](https://www.cypress.com/products/modustoolbox-software-environment)
* [Wi-Fi Host Driver](https://github.com/cypresssemiconductorco/wifi-host-driver)
* [PSoC 6 Code Examples using ModusToolbox IDE](https://github.com/cypresssemiconductorco/Code-Examples-for-ModusToolbox-Software)
* [PSoC 6 Middleware](https://github.com/cypresssemiconductorco/psoc6-middleware)
* [PSoC 6 Resources - KBA223067](https://community.cypress.com/docs/DOC-14644)

---
© Cypress Semiconductor Corporation, 2019-2020.
