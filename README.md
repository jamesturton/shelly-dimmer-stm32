An open-source firmware for the STM32 co-processor on the Shelly Dimmer.

[![License: GPL v3](https://img.shields.io/badge/License-GPLv3-blue.svg)](https://www.gnu.org/licenses/gpl-3.0)

# Instructions
 1. git clone --recurse-submodules https://github.com/jamesturton/shelly-dimmer-stm32.git
 2. cd shelly-dimmer-stm32
 3. make -C libopencm3 # (Only needed once)
 4. make -C shelly-dimmer-stm32

If you have an older git, or got ahead of yourself and skipped the ```--recurse-submodules```
you can fix things by running ```git submodule update --init``` (This is only needed once)

# Pinout
Here is a guess of the pinout of the *STM32F031K6* chip.

![stm32f031k6](https://user-images.githubusercontent.com/6130792/86444616-494b8080-bd11-11ea-8eeb-c07b69b8af35.PNG)

| Pin   | Name      | Function                  | Comment                       |
| ----- | --------- | ------------------------- | ----------------------------- |
| 1     | VDD       | 3V3                       |                               |
| 2     | OCS_IN    | NC                        |                               |
| 3     | OCS_OUT   | NC                        |                               |
| 4     | NRST      | NRST                      |                               |
| 5     | VDDA      | 3V3                       |                               |
| 6     | PA0       | CF1 HLW8012               |                               |
| 7     | PA1       | CF HLW8012                |                               |
| 8     | PA2       | NC                        |                               |
| 9     | PA3       | NC                        |                               |
| 10    | PA4       | NC                        |                               |
| 11    | PA5       | NC                        |                               |
| 12    | PA6       | TEST POINT                | Only to test point?           |
| 13    | PA7       | TEST POINT                | Only to test point?           |
| 14    | PB0       | NC                        |                               |
| 15    | PB1       | NC                        |                               |
| 16    | PB2       | Zero-crossing detection   |                               |
| 17    | VDD       | GND                       |                               |
| 18    | PA8       | ON MOSFET 1               | OSG65R200J                    |
| 19    | PA9       | USART_TX                  |                               |
| 20    | PA10      | USART_RX                  |                               |
| 21    | PA11      | ON MOSFET 2               | OSG65R200J                    |
| 22    | PA12      | NC                        |                               |
| 23    | PA13      | SWDIO                     |                               |
| 24    | PA14      | SWCLK                     |                               |
| 25    | PA15      | NC                        |                               |
| 26    | PB3       | NC                        |                               |
| 27    | PB4       | NC                        |                               |
| 28    | PB5       | NC                        |                               |
| 29    | PB6       | NC                        |                               |
| 30    | PB7       | NC                        |                               |
| 31    | BOOT0     | BOOT0                     | Also connected to S4789BB?    |
| 32    | PB8       | SEL HLW8012               |                               |

# Useful links
 - https://tinkerman.cat/post/hlw8012-ic-new-sonoff-pow
