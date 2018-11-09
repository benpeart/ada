KiCad design files for the BalancingRobot PCB. Used components (with some random links for examples):
* [ESP32 Devkit V1 module, 30 pins](https://www.aliexpress.com/item/ESP32-Development-Board-WiFi-Bluetooth-Ultra-Low-Power-Consumption-Dual-Core-ESP-32-ESP-32S/32802431728.html?spm=a2g0s.9042311.0.0.26604c4dM62q0I)
* [DRV8825 stepper driver](https://www.aliexpress.com/item/Free-shipping-10pcs-lot-3D-Printer-StepStick-DRV8825-Stepper-Motor-Drive-Carrier-Reprap-4-layer-PCB/32292074706.html?spm=a2g0s.9042311.0.0.26604c4dM62q0I)
* [MPU6050 IMU](http://www.aliexpress.com/item/GY-521-MPU-6050-MPU6050-Module-3-Axis-analog-gyro-sensors-3-Axis-Accelerometer-Module/32340949017.html?spm=a2g0s.9042311.0.0.26604c4dM62q0I)
* [Buck converter](https://www.aliexpress.com/item/10PCS-Mini-3A-DC-DC-Converter-Step-Down-Module-Adjustable-3V-5V-16V-Power-for-RC/32639738406.html?spm=a2g0s.9042311.0.0.66ef4c4duolobi)
* 2x 100uF 35V electrolytic capacitor
* Some female headers to fit the modules
* SMD resistors, 0805 size: 5x 3.3kOhm, 1x 100kOhm
* SMD capacitors, 0805 size: 1x 1uF, 1x 10uF
* A polyfuse. Rating depends on chosen battery voltage. Lower voltage means higher current (the steppers work as a buck regulator). I use a 1A polyfuse at 24V.