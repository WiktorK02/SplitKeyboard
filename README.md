# Split Keyboard on STM32WB55 with BLE

## Overview
This project is a **split mechanical keyboard** powered by an **STM32WB55 microcontroller** with **Bluetooth Low Energy (BLE) support**. It enables seamless wireless communication between the two halves of the keyboard, reducing cable clutter while maintaining fast and reliable keypress transmission.

## Features
- **Split Design** – Two separate halves for ergonomic typing
- **Wireless Communication** – BLE connectivity between keyboard halves
- **UART Magnetic Connector** – Used only for charging and connecting the left half
- **Powered by STM32WB55** – Efficient, low-power microcontroller with built-in BLE
- **Custom Firmware** – Flexible and easily configurable
- **Mechanical Key Switch Support** – Compatible with MX-style key switches
- **Low Latency** – Optimized for minimal input delay
- **Battery-Powered** – Support for rechargeable batteries

## Hardware
### Components
- **Microcontroller:** STM32WB55 (integrated BLE support)
- **Key Switches:** MX-compatible mechanical switches
- **Diodes:** 1N4148 (for matrix scanning)
- **Power Management:** Li-Po battery with charging circuit
- **PCB:** Custom-designed PCB for matrix scanning and BLE communication
- **Magnetic Connector (UART):** Used exclusively for charging and connecting the left half

### Circuit Design
The keyboard uses a **row-column scanning** approach, with one half acting as the master (handling BLE communication and USB) and the other as the slave. The halves communicate wirelessly, and the left half can be charged and optionally connected using **UART magnetic connectors**, but not for data transfer.

## Software

### Key Features
- **BLE HID Profile** – Works with Windows, macOS, Linux, and Android
- **Custom Key Mapping** – Easily configurable key layouts
## TODO / Future Improvements
- [ ] Improve power consumption for longer battery life
- [ ] Add rotary encoder support
- [ ] Develop a configuration tool for remapping keys via BLE
- [ ] Low-Power Mode Optimized for battery efficiency

## How to Contribute
1. Fork the Project
2. Clone repo with your GitHub username instead of ```YOUR-USERNAME```:<br>
```
$ git clone https://github.com/YOUR-USERNAME/ALED-module
```
3. Create new branch:<br>
```
$ git branch BRANCH-NAME 
$ git checkout BRANCH-NAME
```
4. Make changes and test<br>
5. Submit Pull Request with comprehensive description of change
## License 
[MIT license](LICENSE)
