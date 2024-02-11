# Jetson TX2

To work with Jetson, you will need to create an [Nvidia developer](https://developer.nvidia.com/) account.

## Compatibility

Jetson products are running by Nvidia customized Ubuntu called "JetPack".
Since Jetson TX2 has reached EOL, it could not run JetPack version newer than 4 which is derived from Ubuntu 18.04 ([reference](https://developer.nvidia.com/embedded/jetpack-archive))
The latest support version is 4.6.4.
If you want need to install JetPack on Jetson, you will need to have Ubuntu 18.04 install as a host machine.

![JetPack Roadmap](https://developer.download.nvidia.com/embedded/images/jetson_roadmap/Jetson-JetPack_SW_Roadmap-2023-09-18.png)

The latest JatPack version is 6 (released in 2024 - [roadmap](https://developer.nvidia.com/embedded/develop/roadmap)).
This version derived from Ubuntu 22.04.
It requires Jetson Orin series.
We should consider upgrade to [Jetson Orin Nano dev kit](https://developer.nvidia.com/buy-jetson) when it is possible.
We could consider to use the modules instead of development kit.
So, we have choice for RAM or upgrade to Jetson Orin NX if we need more processing power.
To use module, we will also need to purchase carrier boards.
There are plenty of options from several vendors.
Thus, it gives us flexibility to choose power supply and I/O that fit our needs.
We can also design a carrier board.
The reference design document is available on [download center](https://developer.nvidia.com/embedded/downloads).
Open Hardware design are also available, for [example](https://github.com/antmicro/jetson-orin-baseboard).

![Jetson Roadmap](https://developer.download.nvidia.com/embedded/images/jetson_roadmap/Jetson_modules-Commercial_roadmap-2023-09-01.png)

Upgrade to new version will minimize incompatible issues from the current Ubuntu and ROS versions.
It will also be more powerful computationally and allow us to use modern software package such as [Isaac-ROS](https://developer.nvidia.com/isaac-ros) (require JetPack version 5 or newer).

## Install JetPack SDK

Download `Nvidia SDK Manager` from this [link](https://developer.nvidia.com/embedded/downloads).
You would need to create an account before you can download it.

Then install the SDK Manager by running

```bash
sudo apt install ./sdkmanager_x.y.z-abcdc_amd64.deb
```

Note that the number and build version could be changed at the time you download it.
You might encounter issues during the installation.
It will cause by missing dependencies.
Carefully read the error messages which will tell you what packages you will need to install before running the above command.
You may use this link as a [reference](https://docs.nvidia.com/sdk-manager/download-run-sdkm/index.html).

Open SDK Manager by running `sdkmanager` in the terminal or you can find the GUI shortcut on the application launcher.
You will be asked to login by opening the browser.
Or you can choose to login via QR (on the top right corner) with your mobile device.

## Format Jetson with new JetPack

Refer to this [document](https://docs.nvidia.com/sdk-manager/install-with-sdkm-jetson/index.html).
Connect Jetson to display with HDMI cable, keyboard with USB (and mouse if you have an USB hub) and host computer with micro USB cable.
You can connect the power supply, but do not turn it on yet.
Open SDK Manger then do following step in order to put Jetson in Force Recover Mode

1. Press and hold the Force Recovery button (REC/S3)
2. Press and hold the Power button (POWER BTN/S4)
3. Release the Power button
4. Release the Force Recovery button

Now, the SDK Manager on the host machine should detect the Jetson.
Follow the steps.
Note, it might take hours to complete.


