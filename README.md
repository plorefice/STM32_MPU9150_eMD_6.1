STM32_MPU9150_eMD_6.1
=====================

Embedded MotionDriver™ 6.1 example using an STM32F4-Discovery board connected to an MPU-9150 breakout board via I²C.

The MPU-9150 is a 9-axis MotionTracking device designed for the low power, low cost, and high performance requirements of consumer electronics equipment including smartphones, tablets and wearable sensors.
It includes a 3-axis accelerometer, a 3-axis gyroscope and a 3-axis magnetometer.

This example uses the new Embedded MotionDriver 6.1 SDK that allows to perform 9-axis sensor fusion on all the ARM Cortex-Mx devices, thanks to the Digital Motion Processor (DMP) integrated inside the MPU which performs 6-axis sensor fusion in the DMP co-processor, and the Motion Processing Library (MPL) that allows to fuse the previous results with the compass raw data in the host processor.

The resulting data is sent to the host PC via USB using a VCP on the host itself.

How to use
==========

Connect the IMU to the board using the following scheme.

**PLEASE NOTE**: In this setup, the breakout board that has been used already integrates a 5V-to-3V3 voltage regulator. **DO NOT** connect 5V directly to the MPU-9150.

    +------------------+               +----------------------+
    |     MPU-9150     |               |   STM32F4DISCOVERY   |
    |                  |               |                      |
    |             INT  +---------------+  PB4                 |
    |             GND  +---------------+  GND                 |
    |             VDD  +---------------+  5V                  |
    |             SDA  +---------------+  PB7                 |
    |             SCL  +---------------+  PB6                 |
    |                  |               |                      |
    +------------------+               +----------------------+

Once the firmware has been uploaded, the board will start sending raw quaternion data to the host PC at the frequency specified during the initialization phase. This data can be visualized via a GUI using the python scripts provided with the SDK (*adjustments to the orientation matrices may be needed for a correct visualization*).
