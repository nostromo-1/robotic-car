
CC := gcc
EXE := robot
SRC_DIR := src
OBJ_DIR := src
SRC := $(wildcard $(SRC_DIR)/*.c)
OBJ := $(SRC:$(SRC_DIR)/%.c=$(OBJ_DIR)/%.o)
DEP := $(OBJ:.o=.d)

CPPFLAGS := -MMD # Generate dependency files
DEBUG := -g
CFLAGS := -O $(DEBUG) $(CPPFLAGS)

BTLIBS := -lcwiid -lbluetooth
PIOLIBS := -lpigpio -lpthread
AUDIOLIBS := -lasound
MATHLIB := -lm
LIBS := $(BTLIBS) $(PIOLIBS) $(AUDIOLIBS) $(MATHLIB)


$(OBJ_DIR)/%.o: $(SRC_DIR)/%.c
	$(CC) -I $(SRC_DIR) $(CFLAGS) -c $< -o $@


$(EXE): $(OBJ)
	$(CC) $^ $(LIBS) -o $@
	sudo chown root $@ 
	sudo chmod u+s $@


clean:
	$(RM) $(OBJ) $(DEP) $(EXE) 

 
-include $(DEP)
   