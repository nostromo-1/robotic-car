
CC=gcc
CFLAGS=-I. -O
DEBUG = -g
DEPS = 
OBJS = motor.o sound.o oled96.o imu.o ekf.o pcf8591.o bmp280.o
BTLIBS = -lcwiid -lbluetooth
PIOLIBS = -lpigpio -lpthread
AUDIOLIBS = -lasound
MATHLIB = -lm


%.o: %.c $(DEPS)
	$(CC) -c -o $@ $< $(CFLAGS) $(DEBUG)



robot: $(OBJS)
	$(CC) -o $@ $^ $(CFLAGS) $(PIOLIBS) $(BTLIBS) $(AUDIOLIBS) $(MATHLIB) 	
	sudo chown root $@ 
	sudo chmod u+s $@

clean:
	rm -f $(OBJS) robot
