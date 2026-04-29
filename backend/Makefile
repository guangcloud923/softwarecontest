CC = gcc
CFLAGS = -Wall -Wextra -O2 -std=c99
LDFLAGS = -lws2_32 -lm

SRC = main.c simcore.c map.c pathfinding.c scheduler.c \
      agv.c conveyor.c robot.c defrag.c constraint.c stats.c server.c
OBJ = $(SRC:.c=.o)
TARGET = warehouse_sim

.PHONY: all clean run run-server

all: $(TARGET)

$(TARGET): $(OBJ)
	$(CC) $(OBJ) -o $(TARGET) $(LDFLAGS)

%.o: %.c
	$(CC) $(CFLAGS) -c $< -o $@

run: $(TARGET)
	./$(TARGET) 30 42

run-server: $(TARGET)
	./$(TARGET) -s -p 8080 30 42

clean:
	rm -f $(OBJ) $(TARGET)
