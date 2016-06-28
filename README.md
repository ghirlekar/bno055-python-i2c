# bno055-python-i2c
Python module for the Adafruit BNO055 on the Raspberry Pi connected through I2C.

# Getting started procedure
These steps are for the a Raspberry Pi 2 running Ubuntu 14.04 (Trusty). The equivalent steps for Raspbian are well documented by Adafruit and others.

## Installing Kernel support
Open this file
```bash
sudo nano /etc/modules
```
and add the followings lines to it
```
i2c-bcm2708 
i2c-dev
```

## Install the necessary python modules

```bash
sudo apt-get install python-smbus
sudo apt-get install i2c-tools
```
python-smbus is the python module that we use for I2C communication on the Raspberry Pi.

## Test it
Connect your i2c peripheral and run this in the shell
```bash
sudo i2cdetect -y 1
```

# Corrupted data issues
If you notice that you have bad data (randomly fluctuating data values) coming from the device, it may be due to the I2C clock speed.
First check what is the baudrate of the I2C bus using
```bash
sudo cat /sys/module/i2c-bcm2708/parameters/baudrate
```
I have noticed the following behaviour of I2C by varying the baudrate

Baudrate|Behaviour
--------|---------
 400KHz | Bus fails to initialize, instead get IOError  
 100KHz | Bus initializes and device communicates with ~50% data corrupted
 70KHz  | Bus initializes and device communicates with occasional data value corrupted
 50KHz  | Bus initializes and device communicates with no apparent data corruption
 
 You can change the baudrate to 50KHz this way
 ```bash
 sudo modprobe -r i2c-bcm2708
 sudo modprobe i2c-bcm2708 baudrate=50000
 ```
