StarryPilot
============================

[中文页](中文说明.md) |

# About
A lightweight and powerful autopilot software, which focus on research and development of state-of-the-art software for UAVs. One of the project’s primary goals is to provide an open and flexible platform making it easy to be applied to a broad range of domains.

# Feature
- RT-Thread RTOS, Fatfs file system, System components, such as IPC, Msh shell system, file manager, parameter system, log system, etc.
- Completely support with Pixhawk hardware
- ADRC control & PID control
- Support with Mavlink(QGround Control)
- Support with Gazebo hardware-in-the-loop (HITL) simulation

# Control
[![ADRC vs PID](docs/images/adrc_video_demo.png)](https://www.youtube.com/watch?v=77-_nF-qqpA&t=63s)

- [**优酷视频地址**](https://v.youku.com/v_show/id_XMzY2Njg4ODk4NA==.html?spm=a2hzp.8244740.0.0)

# Simulator
- [*Matlab simulator*](https://github.com/JcZou/matlab_quadsim) (Software-in-the-loop)
- [*Gazebo simulator*](https://github.com/JcZou/gazebo_quadsim) (Hardware-in-the-loop)

# Tools
- Calibration software (magnetometer and accelerometer calibration)
- Logger checker

# Usage
The project is developed on Pixhawk (autopilot hardware). To download firmware into Pixhawk, please follow these steps:
- First compile the starry_fmu and generate bin file.
- Use QGroundControl (QGC) to download the bin file into fmu. To download custom firmware, choose the following choice.

![](docs/images/fmu_download.png)

- Now the firmware of starry_fmu should be correctly downloaded (If failed, please try again, or use the newer version of QGC). Connect Radio-telemetry to the **TELEM 2** port of Pixhawk, then you should see the Msh shell system output via serial terminal (default baudrate is **57600**).

![](docs/images/msh.png)

- If you didn't format the SD card before, please type `mkfs` command to format the SD card.
- Then you should download the starry_io firmware. To do so, first compile starry_io project to get the bin file and name it as **starryio.bin**. Copy the starryio.bin to the **root directory** of SD card, then open Msh shell system and type `uploader`, which is shown below.

![](docs/images/io_download.png)

- Choose **file system** option to start download. Notice that if it's the first time that you download starry_io, after you type uploader, you should push reset button of io (in the side of Pixhawk) to let io enter the bootloader.
- Congratulation, now the download is finished!

# Compile Environment
- IDE: The compile environment is based on Keil MDK5.

![](docs/images/mdk5.png)

- GNU GCC: For more instructions, please refer to [*README.md*](https://github.com/JcZou/StarryPilot/blob/master/starry_fmu/Project/stm32f40x/README.md)

# Developer List
- *JcZou*: jiachi.zou@gmail.com, zoujiachi666@163.com 
- *weety*: luohui2320@gmail.com
---------------------------

**QQ群: 459133925**
