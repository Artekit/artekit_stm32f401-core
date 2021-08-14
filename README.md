# Artekit STM32F401 Core Files

This repository contains the core files for the Arduino IDE supporting Artekit boards based on the STmicroelectronics STM32F401.

The currently supported boards are the [Artekit PropBoard](https://www.artekit.eu/products/devboards/propboard/) and the [Artekit WaveTooEasy](https://www.artekit.eu/products/devboards/wavetooeasy/) boards

![](https://www.artekit.eu/resources/doc/wavetooeasy/artekit-stm32f401-boards.jpg)

## Installation

1) Download and install the standard Arduino IDE version that suits your operating system from the [this link](https://www.arduino.cc/en/Main/Software).

2) After installing the IDE, open it up and navigate to *File* -> *Preferences*.

    ![](https://www.artekit.eu/resources/doc/propboard-manual/arduino-ide-preferences.jpg)

3) Copy the following URL:

    `https://www.artekit.eu/software/package_artekit_index.json`

    And paste the URL into the *Additional Board Manager URLs* box, like in the picture here below. If you already have another URL in that box, you can add a comma at the end and paste the Artekit URL right after. Then click "OK".

    ![](https://www.artekit.eu/resources/doc/propboard-manual/arduino-ide-manager.jpg)

4) Use the *Boards Manager* from the *Tools* menu and search for *"Artekit"*. Select *"Artekit Labs STM32F401 boards"* and install the latest version.

    ![](https://www.artekit.eu/resources/doc/wavetooeasy/boards-manager.jpg)

## Links

* [PropBoard product page](https://www.artekit.eu/products/devboards/propboard/)
* [PropBoard manual & APIs](https://www.artekit.eu/doc/categories/propboard/)
* [WaveTooEasy product page](https://www.artekit.eu/products/devboards/wavetooeasy/)
* [WaveTooEasy manual](https://www.artekit.eu/doc/guides/wavetooeasy/)
* [WaveTooEasy - Library for Arduino](https://github.com/Artekit/Artekit_WaveTooEasy)
* [WaveTooEasy - Generic C library](https://github.com/Artekit/wavetooeasy-c-lib)

## Bugs report

You can report bugs here by creating a new issue or in the [Artekit forum](https://forum.artekit.eu/).

## License

Core files and Bootloader are released under the GNU GPLv3 license.

Copyright (c) 2021 Artekit Labs.
