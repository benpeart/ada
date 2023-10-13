This folder contains the software source files, written for an ESP32 using the Arduino framework. PlatformIO is used as environment, as it is very easy to use, and provides somewhat more advanced functions, compared to the very basic Arduino IDE. 

# Build environment
For setting up PlatformIO, see https://docs.platformio.org/en/latest/ide/vscode.html. 

Clone the repository. I strongly advice to not download the source files as a zip package, but to use Git properly. The code is very experimental, and will be often updated.

Once installed, go to PlatformIO home in the IDE. In there, click "Open Project", and navigate to the location where you cloned the repository. Select the "Software" folder, and then click "Open "Software"". 

Open src/main.cpp. You should now be able to combile (ctrl+alt+B).

# Uploading firmware
For uploading, I strongly prefer OTA, as this works very nice in PlatformIO.  

Beware: when using the A4988 driver, you have to specify a flag. When using any other driver, remove this flag. This is because the A4988 has a different microstepping selection table. 

## Serial port
For the first upload, you'll need to upload via USB / the serial port. In platformio.ini, state the COM port under which the ESP32 module is connected (uncomment the line with an IP address / host name), for example 
upload_port = COM3. 
; upload_port = balancingRobot.local

Hit the upload button. Currently, the auto reset functionality of the ESP32 module doesn't seem to work, see Issue #10. So, once the message "Serial port COMx" appears, press the enable and boot buttons on the module. First release the boot button, then the enable button. You might have to try a few times. Luckily, once succeeded with the serial upload, you can use OTA upload.

## OTA upload
When the ESP32 boots, it's IP address is printed. In platformio.ini, fill in this IP address under upload_port. Or, even better, use the hostname balancingRobot.local. This means you don't have to mess around with IP addresses. Make sure to be connected to the same WiFi network as the ESP32, and hit upload.

*Update*: in the ps3control branch, the ESP32 has insufficient flash memory to store the OTA upload. Hence, it is disabled in this branch. 

## File system
Initially, or when changing the web page files, you need to upload the file system. To do so, open a terminal within the PlatformIO environment, and run:
platformio run --target uploadfs

This will upload all content of the data folder to the ESP32 flash memory (either via the serial port or OTA).

# WiFi connection
After flashing, the ESP32 will start an access point (AP), named balancingRobot. The default key is "turboturbo". Once connected, open the balancingRobot web configuration page (index3.htm). Here, among others, you can change the WiFi options. Under "WiFi configuration", enter the SSID and key of your home network, change the selector from "AP" to "SSID", and click the "set" button, followed by the "reboot" button. If all goes well, the ESP32 will now connect to your home network. 

*Update:* I noticed that in the latest version, the web pages are not correctly loaded, when the ESP32 is running in access point mode. When connected to a known SSID, everything works as it should.
To configure the WiFi settings without a web page, we use the serial interface (115200 baud). 
The command structure is: \
a "w" for wireless related settings, followed by a "s" for SSID, "k" for key, and "m" for WiFi mode (0 = access point, 1 = connect to SSID), followed by the content, followed by a terminating "x". The recipe for connecting to a known SSID is thus:\
wsYOURSSIDx\
wkYOURKEYx\
wm1x\
Restart the ESP32, and it should connect to the known WiFi network ("wrx" will reset the ESP32).\
The serial interface will print information on whether the connection to a known SSID is successful. If the home network cannot be found, the AP will be started.

# Web page editor
hostName.local/edit

User/pass: admin, admin

Be aware though that changes here are not kept on your PC, so make sure to copy and paste everything.

# Plotting signals
hostName.local/plotTest.htm

To be merged into index.htm at some point

For this to work, first load plotTest.htm, then index.htm. This allows to view signals, while simultaneously adjusting parameters. 

# Wifi control
For a slider based web page to control your robot (this will also work in access point mode)

IPaddress/control.htm
hostName.local/control.htm
or, if you connect to the access point of the robot: 192.168.4.1/control.htm

# Bluetooth control
The ps3control branch supports a ps3 controller. This is convenient, as the ESP32 has bluetooth, and no additional hardware is required. 

To be documented, soon hopefully. Very short: use sixaxispairtool to assign the correct MAC address to the PS3 controller. The MAC address is printed over the serial interface, and is also visible in the web interface. 

The ps3control branch doesn't support OTA anymore, as flash space is limited on the ESP32. 

