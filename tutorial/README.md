# Tutorial Module

## Install Software

Our core software framwork is Robot Operating System or [ROS](https://www.ros.org/).
There are legacy ROS which is no longer actively developed. T
he latest version (Noetic Ninjemys) will reach End of Life in May 2025.
Therefore, we will use ROS 2 instead.
The latest ROS 2 release is Iron Irwini (May 2023).
We will use ROS or ROS2 interchangeably and it will always refer to ROS Iron Irwini.
ROS can run on many operating systems but we highly recommend you to use it with Ubuntu.
We also highly recommend to create a virtual machine and install ROS on it.
Therefore, you will not mess up with your host machine.
The list below is software you need to install.

1. Virtual Machine. We will need to install Ubunto in the machine.
   The recommendation is [Virtual Box](https://www.virtualbox.org/).

2. [Ubuntu 22.04](https://ubuntu.com/download/desktop) (LTS - Long Term Support). This is a first tier support by ROS.

3. ROS Iron.

### Installation Guide

<ol>
<li> Create a new virtual machine. We recommend to set the machine spec as following

- Type: Linux
- Version: Ubuntu (64-bit)
- Memory: 4096 MB or more (Ff you local machine has less than 8GB of RAM, you may need to use lightweight Ubuntu instead)
- Virtual Hard Drive: 25 GB or more (You will create a Dynamic VirtualBox Disk Image. It will take the actual space the VM use but not more than 25GB or at the capaticy you allow it to use)

<li> Mount the Ubuntu ISO that you download by clicking settings -> Stoage. Click on CD drive icon then choose the disk file.

![choose-iso](./img/select-iso.png)

<li> Start the machine. You will follow the installation instruction. 
Along the process, you will be a admin of this virtual machine. 
The password that you choose for it is important. 
If you forget the password, there is no way to recover it. 
You will need this password to install software packages later.

<li> Follow this tutorial to install ROS: https://docs.ros.org/en/iron/Installation/Ubuntu-Install-Debians.html

<li> You can choose either (but not both) `desktop` or `base`. 
The base version has everything we are using right now. 
The desktop version has tools that could be benefitial but we don't use it often. 
If you choose to install the base version, keep in mind that you might not be able to run some commands.

<li> At the end, run the following command, it will add a script to activate ROS everytime you open the new terminal.

```bash
echo "source /opt/ros/iron/setup.bash" >> ~/.bashrc
```

</ol>

### Visual Studio Code and GitHub

We recommend using [Visual Studio Code ](https://code.visualstudio.com/)(VS Code) for writing code.
It comes with extensions that will make coding much easier.
You will need to install [Git](https://git-scm.com/).
For Ubuntu, run the following command to on the terminal

```bash
sudo apt install git
```

Now, you have everything ready to clone the source code and start editing.
You have two options to clone the source code: command or VS Code.
This tutorial will only cover the VS Code option.
First, copy the source code URL from GitHub.
![github-url](./img/github-url.png)

On the VS Code, click View -> Command Pallete (Ctrl + Shift + P) then type "git: clone".
![gitclone](./img/gitclone.png)

VS Code will ask for the URL.
![placeurl](./img/place-url.png)

Select the location that you want to keep the source code. Then you have a source code in your local machine.

## ROS 2 Basics

This section will cover most of what you will need to set up a publisher and subscriber through ROS2. If you ever want additional information on what other things can be done with ROS or want to check some information, their documentation can be found [here](https://docs.ros.org/en/iron/Tutorials.html). The goal of this tutorial is to familiarize you with the fundamentals of ROS communication. This is primarily done by several "nodes" (essentially single pieces of independent code) sending and receiving messages. The simplest form this takes is a publisher and subscriber. One node will write a message to a specific topic (basically just a named place where the value will exist) and another node repeatedly checks that topic to see if anything has been sent. This kind of system is very useful for much of what we need for the rover. For example, one node can constantly publish the state of a joystick and another node can read this in and convert it to motor outputs.

### Creating a ROS 2 workspace

In order to start working with nodes through ROS, you will first need to create the workspace that they will run in. Whenever working with multiple nodes, you will want to start by setting a value called your ROS Domain ID. This eesentially sets your ROS up so that nodes will be able to communicate with each other but it will ignore unrelated nodes on the same network. Each time you open a terminal, you can set this by running the command
```
export ROS_DOMAIN_ID=<your_domain_id>
```
where your_domain_id is any integer between 0 and 101. Once you have your domain id set, make sure that you consistently use the same number for other instances that you want to work together.

A workspace is the directory you have ROS 2 packages in. A package is an organizational unit for your code, which allows it to be run as a node and work with other ROS features. To start, you'll want to make and enter a directory for your workspace.
```
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
```
Feel free to name your workspace folder whatever you would like, but make sure that it still contains the src folder, which is very important to a lot of ROS functionality.
