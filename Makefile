CC = gcc
CFLAGS = -Wall -Wextra -O2 -std=c99
SRC = main.c simcore.c map.c conveyor.c agv.c robot.c constraint.c stats.c
OBJ = $(SRC:.c=.o)
TARGET = warehouse_sim

.PHONY: all clean run

all: $(TARGET)

$(TARGET): $(OBJ)
	$(CC) $(OBJ) -o $(TARGET)

%.o: %.c
	$(CC) $(CFLAGS) -c $< -o $@

run: $(TARGET)
	./$(TARGET)

clean:
	rm -f $(OBJ) $(TARGET)
