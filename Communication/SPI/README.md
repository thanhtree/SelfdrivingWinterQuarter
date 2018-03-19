# SPI Communication with Raspberry Pi and NVIDIA Jetson

This document outlines instructions on setting up SPI communication between the Raspberry Pi and NVIDIA Jetson. 
As opposed to processing video frames on the Raspberry Pi, we need to run the lane line detection algorithm on the Jetson and 
send the appropriate steering angle calculation to the Raspberry Pi, which will control the RC Car. 

## SPI on Raspberry Pi (Stretch)

SPI Loopback test: https://www.raspberrypi.org/documentation/hardware/raspberrypi/spi/README.md

## SPI on Nvidia Jetson TX2

SPI installation: https://elinux.org/Jetson/TX2_SPI