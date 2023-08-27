# Pickleball Sensor

This repository contains a Zephyr application that reads accelerometer data from an LSM6DSL 6-axis sensor and advertises the data over Bluetooth LE. The code has been developed for and tested on Seeed Studio XIAO nRF52840 Sense.

## Getting Started

Before getting started, make sure you have a proper Zephyr development
environment. Follow the official
[Zephyr Getting Started Guide](https://docs.zephyrproject.org/latest/getting_started/index.html).

### Building and running

To build the application, run the following command:

```shell
west build -b $BOARD app
```

where `$BOARD` is the target board. For the XIAO nRF52840 Sense, `$BOARD` is `xiao_ble_sense`.

Once you have built the application, run the following command to flash it:

```shell
west flash
```

For the Seeed Studio XIAO nRF52840 Sense, `west flash` will only work when connected via a [Seeeduino XIAO Expansion Board](https://wiki.seeedstudio.com/Seeeduino-XIAO-Expansion-Board/).

Without the expansion board, you can manually upload the UF2 file by double pressing the reset button on the board, which will make the device discoverable as a drive on the computer. Copy and paste the `zephyr.uf2` file found in `build/zephyr` to the drive and delete `CURRENT.uf2`.
