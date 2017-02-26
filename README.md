# robotic-car
Project of a robotic car with raspberry pi as control unit

This project is about building a hobby robotic car.
It is a 4WD or 2WD car, with a raspberry pi as MCU, and controlled by a wiimote.
It adds some external electronics.

Features:
- 4WD or 2WD robotic car
- It can operate under the control of a Wiimote or in autonomous mode
- The Wiimote can be used to:
  - Move forward (A button) or backward (B button)
  - Turn right (RIGHT button) or left (LEFT button)
  - Increase (+ button) or decrease speed (- button). Speed is signalled in the leds
  - Activate a buzzer sound (DOWN button)
  - Play a police sound (UP button). If you push again the button while still playing, it stops playing
- It continuously monitores distance to an obstacle in the front side
- If an obstacle is near, it will drive around it. It signals the obstacle by buzzing and stopping. It then turns until no obstacle is found.
- It monitors battery voltage and buzzes if batteries are low
- If a button is pressed, it starts scanning for wiimotes and connects to one
- If a pi-camera is attached, it can be used to display the image in a web browser
- Battery: 6 AA rechargeable NiMH for the motor and electronics, a 5V power bank for the raspberry pi. I prefer to use separate power supplies, though a single one could be used.

Communication with the robot is achieved via bluetooth (the wiimote) and wifi (start/stop program, error and status messages). 
The following parts are needed to build it:
- Car chasis. For example, http://www.leantec.es/robotica/59-kit-robot-de-4-ruedas-con-ultrasonido.html
- 4 motors (if 4WD) or 2 motors (if 2WD)
- Motor controller: a L298N based circuit board, like http://www.leantec.es/motores-y-controladores/82-l298-controlador-de-motores-con-doble-puente-h.html
- Distance sensor HC-SR04
- A Raspberry pi. I use a Raspberry Pi 3 Model B, it has buil-in bluetooth and wifi. You can also use a Raspberry Pi Zero with a USB hub (like https://shop.pimoroni.com/products/zero4u), a wifi dongle and a bluetooth dongle.
  
