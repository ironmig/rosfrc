# source files.
CPPSRC =        time.cpp duration.cpp rosfrc/RosRobot.cpp
SRC =   embedded_linux_comms.c

CPPOBJ = $(CPPSRC:.cpp=.o)
OBJ = $(SRC:.c=.o)

OUT = librosembeddedlinux.a

# include directories
INCLUDES = -I/usr/arm-frc-linux-gnueabi/include \
        -I/usr/arm-frc-linux-gnueabi/include/c++/4.9.3 \
        -I/usr/arm-frc-linux-gnueabi/include/c++/4.9.3/arm-frc-linux-gnueabi \
        -I/usr/arm-frc-linux-gnueabi/include/c++/4.9.3/backward \
        -I$(HOME)/wpilib/cpp/current/include \
        -I$(HOME)/wpilib/user/cpp/include \
        -I./

# C++ compiler flags (-g -O2 -Wall)
CCFLAGS = -O0 -g3 -Wall -c -fmessage-length=0 -DBUILD_LIBROSEMBEDDEDLINUX -std=c++11

# compiler
CCC = /usr/bin/arm-frc-linux-gnueabi-g++
CC = /usr/bin/arm-frc-linux-gnueabi-gcc
AR = /usr/bin/arm-frc-linux-gnueabi-ar

# library paths
#LIBS = -L../ -L/usr/local/lib -lm

# compile flags
LDFLAGS = -g

.SUFFIXES: .cpp .c

default: $(OUT)

.c.o:
		$(CC) $(INCLUDES) $(CCFLAGS) -c $< -o $@

.cpp.o:
		$(CCC) $(INCLUDES) $(CCFLAGS) -c $< -o $@

$(OUT): $(CPPOBJ) $(OBJ)
		${AR} rcs $(OUT) $(OBJ) $(CPPOBJ)

clean:
		rm -f $(OBJ) $(CPPOBJ) $(OUT) Makefile.bak 
