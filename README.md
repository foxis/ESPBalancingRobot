# ESPBalancingRobot
ESP8266 ESP-1 WiFi Balancing robot

Originally the robot was built using Arduino pro mini and all it could do was balance. I was thinking to add BT module to control it until I discovered that I have ESP8266 lying around. Also I decided to use a better IMU algorithm instead of DPU of the MPU6050.
Thus the ESPBalancingRobot was born.


# Configuration before Building
Before building and uploading please create `wificonfig.h` file with the following contents:
```
#define WIFI_SSID        "ssid"
#define WIFI_PASSWORD    "password"
#define ARDUINO_HOSTNAME "hostname"
```

If you fork the repo, you can still have this file as it is in `.gitignore` and won't be added during commit.

# Construction

![Schematics of the robot](schematics/Balancing_Robot_bb.png)

## Wheel base

Wheel base consists of two geared motors, some scrap wheels and a 3D printed motor holder which is basically a tube with slots
to hold wooden sheet. Later I added pads so that the robot wouldn't suffer much after falling over.

![Overview of the robot](schematics/robot.jpg)

## Motor controller

I used a 1A motor controller based on L293, however chinese manufacturers seemed to be too secretive about the chip used... Even though they advertise it as L293 motor controller board on ebay :)

![Motor controller](schematics/motor-driver.png)

## IMU

IMU was used a simple MPU6050.

## The ESP-1 module

For robot control we need six pins(not counting ADC pin for battery monitoring): 2x for I2C, 4x for PWM control.
As ESP-1 module clearly doesn't have enough pins we need to add those. With a steady hand, small soldering iron, some flux and good eyes it is possible to solder those additional ports directly to the ESP chip pins.

![Additional GPIO soldered directly to the chip pins](schematics/menu_ota_port.png)

I used RX/TX pins for I2C as the chip emits some noise on those pins while booting and MPU doesn't seem to be upset about that. Although the motors might be.

Also I had to add pullUP resistors to GPIO0 and GPIO2 and a beafy capacitor on the 3.3V rail.

One thing to mention about the motors. For some reason one of my motor drivers and ESP modules during the early stages of the build just died on me. I figured that it had something to do with cheap motor driver burnup. So this time I added 1k resistors in series just to be safe.

# Code


# PID tuning
