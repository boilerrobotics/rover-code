# How to calibrate ODrive

The ODrive we are using is from [FlipSky](https://flipsky.net/products/odesc3-6-optimizes-high-performance-brushless-motor-high-power-dual-drive-controller-foc-bldc-based-on-odrive).
It is hardware version 3.6 56 Volts variant.
By default, it comes with firmware version 0.5.1 which is obsoleted.
There are many improvements on new version.
Additionally, the official ODrive document no longer supports version 0.5.3 or earlier.
At minimum, we should use firmware version 0.5.4.

To upgrade firmware from version 0.5.1 to 0.5.4, first create a virtual environment of Python 3.8 or earlier.
During firmware update process, the script will call `fractions.gcd()` which has been moved to `math.gcd()` since Python 3.9.
If you run the update with `odrivetool dfu`, the erasing old firmware will be successfully but the program will crash when trying to write a new one.
The result is dead ODrive with no firmware.
If that happens, you will need to download pre-compiled firmware from [here](https://github.com/odriverobotics/ODrive/releases).
We need `.bin` which is available only up to firmware version 0.5.4.
Then flash a new firmware using this command.

```bash
sudo apt install dfu-util
sudo dfu-util -a 0 -s 0x08000000 -D ODriveFirmware.bin
```

You will also need to force ODrive in to a DFU mode by put a dip switch number 2 and power cycling the ODrive.
This will put `BOOT0` to ground.
See more detail from the [schematic](https://github.com/odriverobotics/ODriveHardware/blob/master/v3/v3.5docs/schematic_v3.5.pdf).
Note that schematics for hardware version 3.5 and 3.6 are identical.
Don't forget to put the dip switch to the original place and power cycling.

In normal process, `odrivetool dfu` will look for a newer firmware from the server.
The latest odrive from PyPi will throw an error.
We need to use version 0.5.x.
You can install by `pip install odrive==0.5.*`.
Then run `odrivetool dfu`. 
This command will update the firmware to version 0.5.4.
