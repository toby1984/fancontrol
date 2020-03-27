# Fan controller for HPE Microserver

I bought a HPE Microserver only to realize afterwards that the stock fan is WAY too loud to use this server as a 24/7 NAS in my flat. After some digging it turned out that HPE uses special Delta fans that are controlled by an inverted PWM signal (0% duty-cycle = full-on, 100% = off) and come with a special "revolution detect" (RD) signal that indicates whether the fan is jammed. So no easy way of replacing the fan with a more silent one (not to mention that HPE uses a custom 6-pin connector while regular chassis fans use a 4-pin one...).

To fix the fan noise (and also have some fun along the way), I've come up with the following solution:

1.) An Arduino-based fan controller (I had some spare Arduino Nanos lying around) that outputs a regular PWM signal so the usual PC fans work
2.) A daemon process running on the microserver that continously monitores system temperatures and sends the desired fan speed to the Arduino via a USB serial line

Requirements:

- regular 120mm fan with PWM speed control (4-pin connector)
- Arduino-compatible AVR Atmega328p board (I'm using the watchdog timer, 2 PWM outputs (buzzer+fan), one 16-bit timer and 2 8-bit timers) ... you can probably get away with a smaller AVR but I didn't test this)
- ISP programmer for your Arduino board (you can probably also upload the binary some other way, I just never bothered)
- Soldering iron & a steady hand (I just cut off the cable from the stock Delta fan and used it to connect the replacement fan)

## Arduino (fancontrol-avr folder):

The assembly code (yes, I could've written this in C ... but where's the fun in that?) compiles with my own AVR assembler (https://github.com/toby1984/javr) but with minor modifications you should be able to also build it with other assemblers (watch out for LPM instructions, my assembler assumes byte not word addresses).
For convenience I've provided a raw binary as well that you can upload with a 'avrdude' line similar to this:


    /usr/bin/avrdude -F -V -c stk500v2 -p ATMEGA328P -P /dev/ttyACM0 -b 115200 -U flash:w:fancontrol.flash.raw

You need to select the right programmer type , serial device and speed of course.


## Hardware setup

TODO...

## Fan controller service (fancontrol-daemon folder):

I've written a small C program that will use libatasmart / libsensor to get temperature readings, translate them into a fan speed value and send this to the Arduino via the USB serial port. This whole solution only works nicely because the HPE Microserver has an internal USB port, for regular PCs you would need to somehow hook-up the Arduino to an internal USB header on the mainboard which would be more hassle.

I've used Ubuntu 19.10 to develop it with the following libraries:

- libsystemd (libsystemd.so.0.26.0)
- libatasmart (libatasmart.so.4.0.5)
- libsensors (libsensors.so.5.0.0)
- libcjson (libcjson.so.1.7.10)

I've also used GNU Make 4.2.1 and gcc version 9.2.1 20191008 (Ubuntu 9.2.1-9ubuntu2) 

Software installation:

1. Build the software

   Just change to the 'fancontrol-daemon' folder and run

   ````
    make
   ````

   Copy the resulting binary to wherever suits you.

2. You'll need to create a JSON configuration to map sensors to temperature zones and temperatures to fan speeds.
   An example file (example.json) is contained inside the 'systemd' folder.
3. You'll also want to install the program as a proper daemon that gets started when the server comes up. Copy
   the fancontrol.service file to /etc/systemd/system, edit it so that the binary path as well as the path
   to your JSON configuration file are correct and then execute the following commands

   ````
    systemctl daemon-reload
    systemctl start fancontrol.service
    # and if all goes well..
    systemctl enable fancontrol.service
   ````
