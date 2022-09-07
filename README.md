# BalancingRobot

This repository contains all resource files needed to replicate the high speed, two wheeled balancing robot. 

For some videos, see my [youtube channel](https://www.youtube.com/watch?v=D7hvI_Tb0o4). 

For more info, and future updates, see [my website](http://elexperiment.nl/2018/11/high-speed-balancing-robot-introduction/)

For Software Setup see [Software/README.md](Software/README.md)

Please be aware that this code is very experimental / far from complete. So, you'll probably have to implement some stuff yourself. Also, use at your own risk.

### Branches
- master: stable version
- devel: experimental features, use at your own risk
- ps3control: more experimental features. It includes compatibility with a PS3 controller, self righting, and fall detection. 
I'll try to merge all the branches at some point...

# General instructions

## IMU calibration
Once you have everything up and running, the IMU needs to be calibrated. The web interface has two buttons, one for gyroscope and one for accelerometer calibration. 

The gyroscope has a (small) offset, which is calibrated away with a constant correction factor (for all three axes). Lay the robot flat on the ground, without any movement. Then, click the gyro calibration button.

The accelerometer is always placed at a (small, or e.g. 90 degree) angle. By clicking on the second button, the current angle of the robot is used as 0. In other words, the robot should be standing still (without any velocity), due to the position controller bringing the robot to an equilibrium. If you can't get the robot to stand stable, for example because the IMU is placed at a 90 degree offset, lift the robot from the ground, hold it vertically, and then click the button. Afterwards, repeat the "normal" calibration procedure (using the position controller).

Both calibration results are stored in the EEPROM automatically.

## Stepper motor current adjustment
You'll need to manually adjust the current setting potentiometers on the stepper drivers (unless you use the ESP32 DAC, see PCB readme). 

This is done with one stepper driver at a time, such that you can see the current consumption. I use a lab power supply, as it has a current meter built-in. You could also add a multimeter / current meter in series with a battery, for example. So, plug in only one stepper driver, and power on the robot. Never switch stepper drivers (nor the wires to the motors) when powered, magic smoke guaranteed!

Now, use a small screwdriver to adjust the stepper motor current. The "correct" current depends on multiple factors, mostly battery voltage. The higher the voltage, the lower the current draw, as the driver works like a buck regulator. I usually aim for say 0.3 or 0.4A (for one stepper motor) at 12V. The other prominent limit for stepper current is heat: if the drivers get too warm, simply lower stepper current. 

## Stepper recommendation
Use NEMA17 steppers with a hight (without axis) of 50mm and 200 steps per rotation = 1.8 deg. The smaller ones does not work very well.
