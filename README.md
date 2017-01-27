# WarkaWater
A water monitoring system using a STM32L0 microcontroller. The system is
composed of a STM32L0 microcontroller, a DHT11/22 temperature and humidity
sensor and a MS5540C pressure sensor.

## Requirements

### Windows
* IAR Embedded Workbench
* STM32 ST-LINK utility (recommended)

### macOS
* Command Line Tools (CLT) for Xcode: `xcode-select --install`,
  [developer.apple.com/downloads](https://developer.apple.com/downloads) or
    [Xcode](https://itunes.apple.com/us/app/xcode/id497799835)
* [Homebrew/brew](https://github.com/Homebrew/brew/) (recommended to install openOCD and arm-none-eabi)
* OpenOCD (>= 0.10.0)
* GNU ARM Embedded Toolchain (arm-none-eabi)

After the Command Line Tools were successfully installed, the remaining requirements can be installed using Homebrew.

1. Install Homebrew: [brew.sh](http://brew.sh/)

2. Install GCC ARM Toolchain using `brew`:

    ```
    $ brew install gcc-arm-none-eabi
    ```

3. Install openOCD using `brew`:

    ```
    $ brew install openocd
    ```

## How to build

### Windows
1. Double-click on `EWARM/Project.eww` to open the workspace.

2. Compile

3. Run/Debug

### macOS
1. (optional) Edit the Makefile to configure your toolchain paths. Default value
is configured to look for the openOCD directory in the Homebrew directory.
2. Compile your project:

    ```bash
    $ make

    ```
3. Plug-in your board and run openOCD with the appropriate configuration file in another terminal to run a gdb server:

    ```bash
    $ openocd -f scripts/board/stm32l0discovery.cfg
    ```

4. Launch a debug session, load the binary, and debug:

    ```bash
    $ make debug
    ```
    ```
    (gdb) load out.elf
    (gdb) monitor arm semihosting enable
    (gdb) continue
    ```

#### Semihosting
Console output using the C printf function can be achieved using semihosting. Semihosting has to be enabled in openOCD in order to be used. 
When OpenOCD is launched, semihosting can be activated using the following command with a telnet connection:
```bash
$ telnet localhost 4444
arm semihosting enable
```
Semihosting can also be enabled directly using gdb:
```
(gdb) monitor arm semihosting enable
```

## Common build error examples
#### ``./src/main.c:130: undefined reference to `HAL_RCC_OscConfig'``

Object file `stm32l0xx_hal_rcc.o` was not found in the list of object files to
be used by the linker. Uncomment `# OBJECTS  += stm32l0xx_hal_rcc.o` in the
Makefile.

#### ``./Src/main.c:144:46: error: 'FLASH_LATENCY_0' undeclared (first use in this function)``

HAL module header file is missing. Select appropriate module in the HAL
configuration file: `stm32l0xx_hal_conf.h`
