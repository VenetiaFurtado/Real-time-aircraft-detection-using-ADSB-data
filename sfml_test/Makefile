# Compiler and flags
CXX = g++
CXXFLAGS = -std=c++17 -Wall -Wextra

# SFML libraries to link
LIBS = -lsfml-graphics -lsfml-window -lsfml-system

# Your source and target
SRC = main.cpp
TARGET = adsb_plot

# Default build rule
all: $(TARGET)

$(TARGET): $(SRC)
	$(CXX) $(CXXFLAGS) -o $(TARGET) $(SRC) $(LIBS)

# Clean up
clean:
	rm -f $(TARGET)
