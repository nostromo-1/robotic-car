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

Communication with the robot is achieved via bluetooth (the wiimote) and wifi (start/stop program, error and status messages). 
  
