CC = arm-hismall-linux-gcc

TARGET = fu

INC_PATH = /home/yukuai/develop/pj1301/software/localsdk/include
LIB_PATH = /home/yukuai/develop/pj1301/software/localsdk/lib

CFLAGS += -g -Wall -fno-strict-aliasing -I$(INC_PATH) 
LDFLAGS += -L$(LIB_PATH)
LDFLAGS += -lpthread -llocalsdk -lctrlprotocol


OBJS = main.o io.o


all:$(TARGET)


$(TARGET): $(OBJS)
	$(CC) -o $@ $^ $(LDFLAGS)


%.o: %.c
	$(CC) $(CFLAGS) -c $< -o $@


clean:
	rm -f $(OBJS)
	rm -f $(TARGET)

