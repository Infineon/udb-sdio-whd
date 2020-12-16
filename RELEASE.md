# UDB SDIO for WHD Release Notes
This library provides a UDB based SDIO interface that allows for communicating between a PSoC 6 and a wireless device such as the CYW4343W, CYW43438, or CYW43012.

### What's Included?
* UDB based SDIO interface for WHD using Port 2
* UDB based SDIO interface for WHD using Port 9
* UDB based SDIO interface for WHD using Port 12

### What Changed?
#### v1.1.1
* Fixed potential issue with Port 12 handling of data bus busy
#### v1.1.0
* Fixed bug where driver could try to send data while bus was still busy
* Minor code style cleanups
#### v1.0.1
* Minor update for documentation & branding
#### v1.0.0
* Initial release

### Supported Software and Tools
This version of the UDB SDIO interface for WHD was validated for compatibility with the following Software and Tools:

| Software and Tools                        | Version |
| :---                                      | :----:  |
| ModusToolbox Software Environment         | 2.1     |
| GCC Compiler                              | 9.2     |
| IAR Compiler                              | 8.4     |
| ARM Compiler 6                            | 6.11    |

Minimum required ModusToolbox Software Environment: v2.0

### More information

* [API Reference Guide](https://cypresssemiconductorco.github.io/udb-sdio-whd/html/index.html)
* [Cypress Semiconductor, an Infineon Technologies Company](http://www.cypress.com)
* [Cypress Semiconductor GitHub](https://github.com/cypresssemiconductorco)
* [ModusToolbox](https://www.cypress.com/products/modustoolbox-software-environment)
* [PSoC 6 Code Examples using ModusToolbox IDE](https://github.com/cypresssemiconductorco/Code-Examples-for-ModusToolbox-Software)
* [PSoC 6 Middleware](https://github.com/cypresssemiconductorco/psoc6-middleware)
* [PSoC 6 Resources - KBA223067](https://community.cypress.com/docs/DOC-14644)

---
© Cypress Semiconductor Corporation, 2019-2020.