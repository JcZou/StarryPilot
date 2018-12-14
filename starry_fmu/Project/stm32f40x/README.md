Scons building system userguid
============================

# Scons building system install
- [**Scons installation guide **](https://www.rt-thread.org/download/manual/rtthread_manual.zh.pdf)
- Python 2.x (Python 2.7.x is preferable) is needed
- Scons 2.x (Scons 2.5.x is prederable) is needed

# announcements
we using arm-none-eabi- (version: 7-2018-q2-update is recommended) toolchain, don't using linux version gcc toolchain. The project use the newlib of the GCC toolchain.

# building
starry_fmu build dirctory in starry_fmu/Project/stm32f40x. Before building, we first change current work dir to stm32f40x.
building step
- cd starry_fmu/Project/stm32f40x
- scons -j4

When finished, you will get starry_fmu.bin and starry_fmu.px4. starry_fmu.px4 is firmware in px4 format.


