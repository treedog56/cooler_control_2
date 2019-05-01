This code is the source code used to control a smart controller for an evaporative cooler.  I wanted to automate functions of the cooler such as turning the main fan on and off. Turning the water pump for the cooling medium on and off.  Giving greater control to turning the parts of the system on and off based on indoor and outdoor temperatures.
I found that retail aftermarket controllers, were limited in their functions. 
The goals of this project were to do the following.
1.	Monitor indoor, outside, and system temperatures, humidity and barometric presser.  This was done using three   Adafruit BME280 I2C Temperature Humidity Pressure Sensor.
a.	Since I had three separate sensors doing the same functions, I used Bme_Functions.h to initialize, process, store, and retrieve data from the sensors as objects.  This allowed for a standard way to tie all data and functions to a sensor and reducing the coding needed to interact with sensors.  
b.	In addition, two Waterproof DS18B20 Digital temperature sensor on opposite sides of the water tank to average the temperature of the water in the tank.  I used DS1820_Sensors.h to initialize, process, store, and retrieve data from the sensors as objects.   
c.	HelperFunctions.h is used to give a standardized read and write functions to the IC2 bus for Bme_Functions.h, DS1820_Sensors.h

2.	Based on time, temperatures and humidity do the following.
a.	Turn on and off the main fan.
b.	Turn on and off the water pump that wets the cooling medium based on temperatures and humidity, allowing for both operation of the system without using the water cooling adding additional humidity to the house.
c.	Controls the water level in the water tank.
d.	Turns on and off a small water chiller to pre-cool the water going over the cooling medium, and a radiator in efforts to get grater cooling efficiency and reduce the humidity coming into the house.
e.	This control of cycling the above componence were accomplished using five MCP4725 Breakout Boards to hold open, or close five voltage relays.
f.	This is the bulk of the main program found in cooler_control_2.ino, and it uses Time_Functions.h to create a system clock and update the system clock, with a pull to a NTP server once an hour to reset the clock if there is any drift in the system clock.
g.	A TSL2591 High Dynamic Range Digital Light Sensor was added to the system to track radiant lighting, for tracking purposes, with an intent to see if it could be used in help predict weather patterns in advance to allow for a pre-house cooling before warmer weather but has not been implemented.

3.	 Have the capability of running autonomously, this is still a work in progress, looking to implement the following features to achieve this.  
a.	Using both gathered sensor information, and parsed weather reports, predictably create cooling cycles loads a day in advance.
b.	Using data gained from the system and indoor temperatures predict the time it will take to cool the house to setpoints.
c.	Using the data from points a and b, create a dynamic run time schedule to allow for use of early morning cool temperatures to pre-cool the house to reduce runtime with humidity.
d.	Use no more power to cool the house than the current AC systems.

4.	Control of the system and sensors is achieved using an Espressif ESP2866 as a micron control, all sensors communicate through the IC2 bus architecture. Use of a TCA9545 IC2 bus multiplexer allows for both like zone sensor grouping and avoids IC2 address conflicts for sensors with like hardcodes address. 

To Do’s
1.	Resolve IC2 issues that occur when using the Espressif ESP32, currently causes a erroneous read of the bus and cases the program to crash.  At the time of development, IC2 bus issues were a known issue with the Espressif ESP32.

2.	Reimplement the web interface for system status and displays.  Currently a telnet like console is available through ssh with limited function.

3.	Reimplement watchdog controls to recover and restart from system or sensor hangs.

4.	Clean up the code and continue to remove functions that are in the main loop into their own header files.

5.	Remove unused portion of codes and comments.

6.	Improve comments explaining the code and what it is doing.

7.	Work on adding and implementing the autonomous function stated in goal number three.


Thanks to the many postings of code sources and snippets for the sensors I am using in this project.  There were many that ideas and code were tested and used in the project and were critical to get to this point.

