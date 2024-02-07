## Upgrade Firmware

The ODrive we have comes with firmware version 0.5.1, which is out of date.
It lacks of multiple features that mentioned in the latest documents.
This makes development and debug difficult.
To upgrade to the newer firmware, there are 2 options:

1. Using built-in DFU tool
2. Using `dfu-util`.

### Using ODrive built-in DFU tool

First, use Python 3.8 or earlier.
The flashing process will call `fractions.gcd()` which has been moved to `math.gcd()` in Python 3.9.
The erasing process will be completed with no error.
But, the error will happen during flashing process.
At the end, you will get an ODrive with no firmware.
If that happens, you must use external DFU tool.

Second, the update could be done up to version 0.5.4 only.
You must install odrive version 0.5.1 to 0.5.4 via pip.
Then when you run `odrivetool dfu`, it will download and flashing ODrive firmware version 0.5.4.

### Using `dfu-util`

You will need firmware.bin for this operation.
You can find binary firmware file up to version 0.5.4.
Look at the `assets` of each release in [GitHub](https://github.com/odriverobotics/ODrive/releases).

For version 0.5.5 and later, you must compile the firmware manually. You will need following packages to compile.

```bash
sudo apt install tup gcc-arm-none-eabi build-essential git-lfs openocd python3-yaml python3-jinja2 python3-jsonschema
```

And Python `cantools` version 38.0.2 or earlier. It is recommend to create an virtual environment.

```bash
pip install cantools==38.0.2
```

1. Clone or download the source code from [GitHub](https://github.com/odriverobotics/ODrive/tags). 
Noted: if you clone, you will need to switch branch to the firmware version that you want to build.     
2. In `Firmware`, change `tup.config.default` to `tup.config`
3. In `tup.config`, specify the hardware variant. In our case, it is `v3.6-56v`
4. Run `make` or `make -jx` where `x` is number of thread you want to assign for compiling
5. New firmware will be in `build`

Next, you need to put ODrive in the DFU mode.
You can do that by flicking the dip switch number 2.
It will pull `BOOT_0` pin from the MCU to DFU mode.
See the [schematic](https://github.com/odriverobotics/ODriveHardware/blob/master/v3/v3.5docs/schematic_v3.5.pdf) for more detail.
Note that hardware version 3.5 and 3.6 are identical.
You must reboot the ODrive to see the effect.
If you do correctly, when you run `lsusb` command, you should see

```bash
STMicroelectronics STM Device in DFU Mode
```

If you see below, it means that ODrive is in the normal mode.

```bash
Generic ODrive Robotics ODrive v3
```

Now, you can flash the new firmware by this command

```bash
sudo dfu-util -a 0 -s 0x08000000 -D build/ODriveFirmware.bin
```

Don't forget to switch the dip switch back and reboot Odrive.
