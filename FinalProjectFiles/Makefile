# Compiler and flags
CXX := g++
C := gcc
CXXFLAGS := -std=c++23 -Wall -Werror -pedantic -g
CFLAGS   := -Wall -g -Ofast -march=native -DWITH_RTL
LDLIBS+=$(shell pkg-config --libs librtlsdr) -lpthread -lm -lsfml-graphics -lsfml-window -lsfml-system


# Target executable name
TARGET := sequencer

# Source files
SRCS := Sequencer.cpp adsb.cpp
C_SRCS :=  msk.c acars.c acarsdec.c rtl.c output.c cJSON.c netout.c label.c fileout.c
OBJS := $(C_SRCS:.c=.o) $(SRCS:.cpp=.o) 

# Default rule
all: $(TARGET)

# Linking
$(TARGET): $(OBJS)
	$(CXX) $(CXXFLAGS) -o $(TARGET) $(OBJS) $(LDLIBS)

# Compilation rule
%.o: %.cpp
	$(CXX) $(CXXFLAGS) -c $< -o $@

# Compile C files
%.o: %.c
	$(CC) $(CFLAGS) -c $< -o $@


# Clean rule
clean:
	rm -f $(OBJS) $(TARGET)

# Phony targets
.PHONY: all clean
