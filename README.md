An open-source firmware for the STM32 co-processor on the Shelly Dimmer and Shelly Dimmer 2.

[![License: GPL v3](https://img.shields.io/badge/License-GPLv3-blue.svg)](https://www.gnu.org/licenses/gpl-3.0)
[![Gitpod Ready-to-Code](https://img.shields.io/badge/Gitpod-Ready--to--Code-blue?logo=gitpod)](https://gitpod.io/#https://github.com/jamesturton/shelly-dimmer-stm32)

# Disclaimer

:warning: **DANGER OF ELECTROCUTION** :warning:

The Shelly Dimmer 1 and Shelly Dimmer 2 connect to mains electricity (AC power), therefore there is danger of electrocution if not installed properly. If you don't know how to install it, please call an electrician (***Beware:*** certain countries prohibit installation without a licensed electrician present). Remember: _**SAFETY FIRST**_. It is not worth the risk to yourself, your family and your home if you don't know exactly what you are doing. Never tinker or try to flash a device using the serial programming interface while it is connected to MAINS ELECTRICITY (AC power).

We don't take any responsibility nor liability for using this software nor for the installation or any tips, advice, videos, etc. given by any member of this site or any related site.

# Pre-build binary
See the [releases](https://github.com/jamesturton/shelly-dimmer-stm32/releases) page to download the latest pre-build binary.

# Instructions

Use [Gitpod](https://gitpod.io/#https://github.com/jamesturton/shelly-dimmer-stm32) to edit and compile the firmware from within your browser.

For building local on your PC do:

 1. ```git clone --recurse-submodules https://github.com/jamesturton/shelly-dimmer-stm32.git```
 2. ```cd shelly-dimmer-stm32```
 3. ```make -C libopencm3 # (Only needed once)```
 4. ```make -C src```

If you have an older git, or got ahead of yourself and skipped the ```--recurse-submodules```
you can fix things by running ```git submodule update --init``` (This is only needed once)

To flash the firmware directly to the STM32 chip using a programmer such as the Black Magic Probe (ST-Link could also be used):

 5. ```make -C src flash BMP_PORT=/dev/ttyBmpGdb```

# Communication protocol
Information on the communication protocol used can be found [here](COMMUNICATION.md).

# Pinout
Here is a guess of the pinout of the *STM32F031K6* chip.

![stm32f031k6](https://user-images.githubusercontent.com/6130792/86444616-494b8080-bd11-11ea-8eeb-c07b69b8af35.PNG)

| Pin   | Name      | Function - Shelly Dimmer 1    | Function - Shelly Dimmer 2    | Pin type |
| ----- | --------- | ----------------------------- | ----------------------------- | - |
| 1     | VDD       | 3V3                           | 3V3                           |  |
| 2     | OCS_IN    | -                             | -                             |  |
| 3     | OCS_OUT   | -                             | -                             |  |
| 4     | NRST      | NRST                          | NRST                          |  |
| 5     | VDDA      | 3V3                           | 3V3                           |  |
| 6     | PA0       | CF1 HLW8012                   | -                             | Input  |
| 7     | PA1       | CF HLW8012                    | -                             | Input  |
| 8     | PA2       | -                             | -                             | Input  |
| 9     | PA3       | -                             | ??                            | Input  |
| 10    | PA4       | -                             | -                             | Input  |
| 11    | PA5       | -                             | Live pin sense                | Analog |
| 12    | PA6       | TEST POINT                    | -                             | Input  |
| 13    | PA7       | TEST POINT                    | Output pin sense              | Analog |
| 14    | PB0       | -                             | ??                            | Input  |
| 15    | PB1       | -                             | Board HW version?             | Input  |
| 16    | PB2       | Zero-crossing detection       | ??                            | Input ExtInt  |
| 17    | VDD       | GND                           | GND                           |  |
| 18    | PA8       | ON MOSFET 1                   | -                             | Input  |
| 19    | PA9       | USART_TX                      | USART_TX                      | AltFun |
| 20    | PA10      | USART_RX                      | USART_RX                      | AltFun |
| 21    | PA11      | ON MOSFET 2                   | ON MOSFET 1                   | Output |
| 22    | PA12      | -                             | ON MOSFET 2                   | Output |
| 23    | PA13      | SWDIO                         | SWDIO                         | AltFun |
| 24    | PA14      | SWCLK                         | SWCLK                         | AltFun |
| 25    | PA15      | -                             | -                             | Input  |
| 26    | PB3       | -                             | -                             | Input  |
| 27    | PB4       | -                             | -                             | Input  |
| 28    | PB5       | -                             | -                             | Input  |
| 29    | PB6       | -                             | -                             | Input  |
| 30    | PB7       | -                             | Pin7 A42 BAKN                 | Input ExtInt Live - Output |
| 31    | BOOT0     | BOOT0                         | BOOT0                         |  |
| 32    | PB8       | SEL HLW8012                   | -                             | Input  |

# Useful links
 - [HLW8012 Information](https://tinkerman.cat/post/hlw8012-ic-new-sonoff-pow)
 - [STM32F031 Reference manual](https://www.st.com/resource/en/reference_manual/dm00031936-stm32f0x1stm32f0x2stm32f0x8-advanced-armbased-32bit-mcus-stmicroelectronics.pdf)
