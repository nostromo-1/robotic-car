# Robotic Car
Project of a robotic car with Raspberry Pi as control unit

This project is about building a hobby robotic car.
It is a 4WD or 2WD car, with a raspberry pi as MCU, and controlled by a wiimote.
It needs some external electronics.
Communication with the robot is achieved via bluetooth (the wiimote) and wifi (start/stop program, error and status messages).

[![Robotic car](https://github.com/nostromo-1/robotic-car/blob/master/photos/robotcar.jpg)](https://github.com/nostromo-1/robotic-car)

## Features
* 4WD or 2WD robotic car (depending on chasis)
* It can operate under the control of a Wiimote or in autonomous mode
* The Wiimote can be used to:
  * Move forward (A button) or backward (B button)
  * Turn right (RIGHT button) or left (LEFT button)
  * Increase ('+' button) or decrease speed ('-' button). Speed is signalled in the leds
  * Activate a buzzer sound (DOWN button)
  * Play a police sound (UP button) over a loudspeaker. If you push again the UP button while still playing, it stops playing
  * Increase (button ´1´ + button '+') or decrease (button '1' + button '-') volume of sound
* It continuously monitors distance to an obstacle in the front side
* If an obstacle is near, it will drive around it. It signals the obstacle by buzzing and stopping. It then turns until no obstacle is found.
* It monitors battery voltage and shows low battery state by turning off a LED. If battery is low, it also signals it via GPIO, and buzzes as a warning. It also shows battery status as a symbol in the display.
* If a button is pressed, it starts scanning for wiimotes and connects to one
* If a pi-camera is attached, it can be used to display the image in a web browser (using https://github.com/silvanmelchior/RPi_Cam_Web_Interface)
* It displays messages in a display
* It features a KARR-type scanner

## Parts
The following parts are needed to build it:
* Car chasis. For example, http://www.leantec.es/robotica/59-kit-robot-de-4-ruedas-con-ultrasonido.html
* 4 6V DC motors (if 4WD) or 2 motors (if 2WD)
* Motor controller: a L298N based circuit board, like http://www.leantec.es/motores-y-controladores/82-l298-controlador-de-motores-con-doble-puente-h.html
* Distance sensor HC-SR04
* Display module SSD1306 (like https://www.sunfounder.com/oled-ssd1306-module.html)
* A Raspberry pi. I use a Raspberry Pi 3 Model B, it has built-in bluetooth and wifi. You can also use a Raspberry Pi Zero with a USB hub (like https://shop.pimoroni.com/products/zero4u), a wifi dongle and a bluetooth dongle. Or the newly released Pi Zero W!
* Transistors, capacitors, resistors, push button and LED for voltage checker and scan button. A 6V buzzer. The schematics can be seen in https://github.com/nostromo-1/robotic-car/blob/master/schematics/Esquema-coche.pdf
* For the audio amplifier: a LM386 integrated circuit, an 8 ohm small speaker and some resistors and capacitors. The schematics are in https://github.com/nostromo-1/robotic-car/blob/master/schematics/Esquema-coche%20ampli.pdf. The audio signal is taken from the GPIO, it does not use the audio output jack, so it also works on a Raspberry Pi Zero.
* For the battery status monitor: a PCF8591, connected to a bit banged I2C bus in the GPIO (activated in /boot/config.txt).
* Power supply: two 18650 type batteries in series, protected. I use 2600 mAh Nitecore. The 5V supply for the Pi comes from a switching regulator. I use the [S7V7F5](https://www.pololu.com/product/2119). Alternatively, you can use 6 NiMH AA batteries.

## Software
The robot runs on raspbian, I have tested it on the releases from mid 2016. It is programmed in C. It makes use of the pigpio library (http://abyz.co.uk/rpi/pigpio/) for GPIO access, allowing it to play sound and use PWM at the same time. It also needs the bluetooth and alsa libraries.
The program avoids active loops in order to make a light use of CPU. Its CPU usage is about 7% (which is due to pigpio). It makes use of event loops and semaphores in order to avoid active loops.
The display control code is included in the software for simplicity, it does not need any display driver library.

The following packages need to be installed on plain raspbian (`sudo apt-get install`):
* libbluetooth-dev
* libcwiid-dev
* libasound2-dev

After installing them copy the code files, the Makefile and the sounds directory, and run `make robot`. After compiling, run it with `sudo ./robot`.


  
