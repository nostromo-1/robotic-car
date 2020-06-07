
EXE := robot
SRC_DIR := src
OBJ_DIR := src
DEBUG = -g
CFLAGS := -O $(DEBUG)
SRC := $(wildcard $(SRC_DIR)/*.c)
OBJ := $(SRC:$(SRC_DIR)/%.c=$(OBJ_DIR)/%.o)

BTLIBS = -lcwiid -lbluetooth
PIOLIBS = -lpigpio -lpthread
AUDIOLIBS = -lasound
MATHLIB = -lm
LIBS := $(BTLIBS) $(PIOLIBS) $(AUDIOLIBS) $(MATHLIB)



$(OBJ_DIR)/%.o: $(SRC_DIR)/%.c
	$(CC) -I$(SRC_DIR) $(CFLAGS) -c $< -o $@


all: $(EXE)

$(EXE): $(OBJ)
	$(CC) $^ $(LIBS) -o $@
	sudo chown root $@ 
	sudo chmod u+s $@


clean:
	$(RM) $(OBJ) $(EXE)
