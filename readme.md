# Capstone Remote Station ROS Catkin Workspace

Project Paper: See Aerobot_ITC22-compressed.pdf

## High level setup for Remote Station
- Install Focal (Ubu 20.04) On Laptop
- Install ROS Noetic Desktop (Supports Python 3)
- Clone this repo
- Configure device ports in remote_jumbo.launch, see below
- <code>roslaunch remote_jumbo.launch</code> in terminal

## Configuring Device Ports
Serial ports are inconsistently assigned on boot, so it is necessary to assign each device the serial <code>dev</code>
- For each serial device
  - Unplug and replug device from board
  - Use <code>dmesg | grep tty</code> to find the latest attached port
  - Use this <code>dev</code> in <code>roslaunch remote_jumbo.launch</code>

## Connecting to Robot
If AION Gold Image was installed, UGV will host wifi direct network SSID: AIONio-3acd p: aionrobotics
- Connect to wifi direct
- <code>ssh nvidia@10.0.1.128</code> in terminal (ip is def gateway) p: nvidia

For remote desktop install nomachine, Jetson TX2 is ARM.  
For normal work power TX2 via wall barrel plug adapter and connect an HDMI monitor.
