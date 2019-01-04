# Agenator_Arduino
Source code for the Arduino microcontroller used in Agenator: An open source modular beef-aging system with control and/or recording capabilities for relative humidity, fan speed, weight, and air velocity

When uploading the code to an Arduino microcontroller, you will be using "Agenator.ino". However, it contains references to a few classes stored in the "libraries" folder. You should move the contents of the "libraries" folder to the location of your Arduino user libraries. For a Windows computer, this is usually located at "C:\Users\[username]\Documents\Arduino\libraries" where [username] is your username.

The Arduino board should be an Arduino Uno Rev 3 board and off-brand boards should work fine too. However, you must upgrade the board to an Atmega1284P chip due to the large program size. There is a "drop-in" replacement to do this: http://www.firebirduino.com/uu/index.html . Assuming your Arduino board uses a DIP package for the Atmega328P, all you need to do is to pry off the Atmega328P with a flat head screwdriver and install this upgrade.
