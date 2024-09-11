# Arduino Opta Data Collector
Using an Arduino Opta Wifi AFX00002 as a data collector for my reverse osmosis water purification plant. 

# Background
I took over a 20 year old water purification system controlled by a Crouzet Millenium II+ PLC. Since this system does not have any communication capabilities, I decided add some modern software to get a better monitoring of the system.

The old way to get notified about problems with the system was that there was no water in the taps. The new system will send iOS push notifications and emails.

First part of project is to use the Opta for monitoring the activities of the Millenium and report the data to my backend server. When that is working reliably, I will write the code to control the pumps and valves with the Opta and eventually retire the old PLC.

The data is sent via MQTT to a Linux server running a Laravel/MySQL app. 

# Arduino Opta Wifi
I selected the Arduino Opta Wifi controller because it was the only Arduino compatible unit with wifi I could find that looked professional and would fit a DIN rail cabinet. The Controllino hardware looked ok as well but did not have wifi.

# Some things good to know about Arduino Opta
* It requires a 12 - 24V power supply. I bought a "MEAN WELL HDR-15-12" from Amazon and it works ok. However, It seems like some of the sensors I am planning to use require 24V, so I might switch to that later.
* It has 8 digital/analog 0-10V inputs. It seemed impractical with a 12V system requiring 10V input, but it turned out the 0-10V is only for the analog signal. 12V input works fine for digital.
* It only has 4 outputs, in the form of 230V relays. However, the 8 inputs can also be used for output. In theory they may work with an I2C display. I may try this later.
* The documentaion about anything Arduino is slightly confusing. For a long time I was stuck with an unreliable MQTT connection that would not reconnect once the connection was lost. I wrongly assumed I needed to use the BearSSL library to run SSL, but apparently mbed TLS is already included in the Opta library package. After switching to mbed TLS the connection has been stable. Only documentation I have found about Opta libararies is to look in the folder that gets installed when you add the Opta board to the Arduino IDE.
* Opta requires a driver to show up as a serial port in Mac OS. However, it will only activate if the cable is connected to the computer when the Opta starts or restarts. If the Opta is already running you will have to press reset for it to connect to the computer.
