# robotic-car
Project of a robotic car with raspberry pi as control unit

This project is about building a hobby robotic car.
It is a 4WD or 2WD car, with a raspberry pi as MCU, and controlled by a wiimote.
It adds some external electronics.
Communication with the robot is achieved via bluetooth (the wiimote) and wifi (start/stop program, error and status messages).

## Features
* 4WD or 2WD robotic car
* It can operate under the control of a Wiimote or in autonomous mode
* The Wiimote can be used to:
  * Move forward (A button) or backward (B button)
  * Turn right (RIGHT button) or left (LEFT button)
  * Increase ('+' button) or decrease speed ('-' button). Speed is signalled in the leds
  * Activate a buzzer sound (DOWN button)
  * Play a police sound (UP button). If you push again the button while still playing, it stops playing
  * Increase (button ´1´ + button '+') or decrease (button '1' + button '-') volume of sound
* It continuously monitors distance to an obstacle in the front side
* If an obstacle is near, it will drive around it. It signals the obstacle by buzzing and stopping. It then turns until no obstacle is found.
* It monitors battery voltage and shows low battery state by dimming a LED. If battery is low, it signals via GPIO, and buzzes as a warning
* If a button is pressed, it starts scanning for wiimotes and connects to one
* If a pi-camera is attached, it can be used to display the image in a web browser (using https://github.com/silvanmelchior/RPi_Cam_Web_Interface)
* Battery: 6 AA rechargeable NiMH for the motor and electronics, a 5V power bank for the raspberry pi. I prefer to use separate power supplies, though a single one could be used.

## Parts
The following parts are needed to build it:
* Car chasis. For example, http://www.leantec.es/robotica/59-kit-robot-de-4-ruedas-con-ultrasonido.html
* 4 motors (if 4WD) or 2 motors (if 2WD)
* Motor controller: a L298N based circuit board, like http://www.leantec.es/motores-y-controladores/82-l298-controlador-de-motores-con-doble-puente-h.html
* Distance sensor HC-SR04
* A Raspberry pi. I use a Raspberry Pi 3 Model B, it has built-in bluetooth and wifi. You can also use a Raspberry Pi Zero with a USB hub (like https://shop.pimoroni.com/products/zero4u), a wifi dongle and a bluetooth dongle. Or the newly released Pi Zero W!
* Transistors, capacitors, resistors, push button and LED for voltage checker and scan button. A 6V buzzer. The schematics can be seen in https://github.com/nostromo-1/robotic-car/blob/master/schematics/Esquema-coche.pdf
* For the audio amplifier: a LM386 integrated circuit, an 8 ohm small speaker and some resistors and capacitors. The schematics are in https://github.com/nostromo-1/robotic-car/blob/master/schematics/Esquema-coche%20ampli.pdf. The audio signal is taken from the GPIO, it does not use the audio output jack, so it also works on a Raspberry Pi Zero.

## Software
The robot runs on raspbian, I have tested it on the releases from mid 2016. It is programmed in C. It makes use of the [pigpio library] (http://abyz.co.uk/rpi/pigpio/) for GPIO access. It also needs the bluetooth and alsa libraries.
The program avoids active loops in order to make a light use of CPU. Its CPU usage is about 7% (which is due to pigpio). It makes use of event loops and semaphores in order to avoid active loops.

The following packages need to be installed (`sudo apt-get install`):
* libbluetooth-dev
* libcwiid-dev
* libasound2-dev

After installing them copy the C files, the Makefile and the sounds directory, and run `make robot`. After compiling, run it with `sudo ./robot`.


  
