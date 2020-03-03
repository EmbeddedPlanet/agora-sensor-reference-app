# Agora Sensor Reference Application

This project demonstrates the usage of the sensors on the Embedded Planet Agora.

## Application functionality

1. The Agora's sensors are powered on and initialized.
1. Every 30 seconds, new readings are obtained from the sensors and the values are printed on the debug console.

Steps to compile and run application:

1. [Install Mbed CLI](https://os.mbed.com/docs/mbed-os/latest/quick-start/offline-with-mbed-cli.html).

1. Clone this repository on your system, and change the current directory to where the project was cloned:

    ```bash
    $ git clone git@github.com:embeddedplanet/agora-sensor-reference-app && cd agora-sensor-reference-app
    ```

## Building and running

1. Connect a USB cable between the USB port on the board and the host computer.
2. <a name="build_cmd"></a> Run the following command to build the example project and program the microcontroller flash memory:
    ```bash
    $ mbed compile -m EP_AGORA -t GCC_ARM --flash
    ```
The binary is located at `./BUILD/EP_AGORA/GCC_ARM/agora-sensor-reference-app.bin`.

Alternatively, you can manually copy the binary to the board, which you mount on the host computer over USB.

```bash
$ mbed compile -S
```

## Related Links

* [Mbed OS Stats API](https://os.mbed.com/docs/latest/apis/mbed-statistics.html).
* [Mbed OS Configuration](https://os.mbed.com/docs/latest/reference/configuration.html).
* [Mbed OS Serial Communication](https://os.mbed.com/docs/latest/tutorials/serial-communication.html).
* [Mbed OS bare metal](https://os.mbed.com/docs/mbed-os/latest/reference/mbed-os-bare-metal.html).
* [Mbed boards](https://os.mbed.com/platforms/).

### License and contributions

The software is provided under Apache-2.0 license. Contributions to this project are accepted under the same license. Please see contributing.md for more info.

This project contains code from other projects. The original license text is included in those source files. They must comply with our license guide.
