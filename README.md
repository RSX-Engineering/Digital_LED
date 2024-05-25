# Digital LED

* WS281x signal decoder
* 4-channel LED driver & color mixer, based of ProffieOSx's [Analog LED](https://github.com/RSX-Engineering/ProffieOSx/wiki/Analog-LEDs) module

### Hardware setup
* STM32C011F4 microcontroller 
* WS signal input at PA0 (TIM1)
* LED PWM outputs at PA6, PA7, PA8 & PB7 (TIM3)
* ST-Link at SWD pins
* (optional) UART-USB converter at PA1 & PB6

### Software setup
* Install STM32Cube IDE from [ST website](https://www.st.com/en/development-tools/stm32cubeide.html) or the STM32 VS Code extension
* Open project folder, build and debug or program


