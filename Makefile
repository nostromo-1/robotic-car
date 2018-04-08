
CC=gcc
CFLAGS=-I.
DEBUG = -g
DEPS = 
BTLIBS = -lcwiid -lbluetooth
PIOLIBS = -lpigpio -lpthread
AUDIOLIBS = -lasound
MATHLIB = -lm


%.o: %.c $(DEPS)
	$(CC) -c -o $@ $< $(CFLAGS) $(DEBUG)



robot: motor.o sound.o oled96.o imu.o pcf8591.o
	$(CC) -o $@ $^ $(CFLAGS) $(PIOLIBS) $(BTLIBS) $(AUDIOLIBS) $(MATHLIB)	

