# =====================================================
# Makefile unique : PC (natif) + Raspberry Pi 64 bits (natif)
# =====================================================

CXX      = g++
CC       = gcc
CFLAGS   = -Wall -Wextra -Iinclude
CXXFLAGS = -Wall -Wextra -Iinclude -std=c++11

# Détection automatique d'OpenCV
PKG_CONFIG_NAME = opencv4
ifeq ($(shell pkg-config --exists opencv4 && echo 1 || echo 0),0)
    PKG_CONFIG_NAME = opencv
endif

ifeq ($(shell pkg-config --exists $(PKG_CONFIG_NAME) && echo 1),1)
    CXXFLAGS += $(shell pkg-config --cflags $(PKG_CONFIG_NAME))
    LDLIBS   += $(shell pkg-config --libs $(PKG_CONFIG_NAME))
    CXXFLAGS += -DHAVE_OPENCV=1
    $(info === OpenCV détecté : $(PKG_CONFIG_NAME) ===)
else
    $(warning === OpenCV non détecté → compilation sans fonctionnalités OpenCV ===)
    CXXFLAGS += -DHAVE_OPENCV=0
endif

LDLIBS += -lm

# Répertoires
BUILD_DIR   = build
BIN_DIR     = bin
SRC_DIR     = src
EXAMPLE_DIR = example

# Sources
C_SOURCES   = $(wildcard $(SRC_DIR)/*.c)
CPP_SOURCES = $(wildcard $(SRC_DIR)/*.cpp)

C_OBJECTS   = $(C_SOURCES:$(SRC_DIR)/%.c=$(BUILD_DIR)/%.o)
CPP_OBJECTS = $(CPP_SOURCES:$(SRC_DIR)/%.cpp=$(BUILD_DIR)/%.o)
OBJECTS     = $(C_OBJECTS) $(CPP_OBJECTS)

# Exemples
EXAMPLES_SRC = $(wildcard $(EXAMPLE_DIR)/*.c)
EXAMPLES_BIN = $(patsubst $(EXAMPLE_DIR)/%.c,$(BIN_DIR)/%,$(EXAMPLES_SRC))

# =====================================================
# Règles
# =====================================================

all: dirs $(EXAMPLES_BIN)

dirs:
	@mkdir -p $(BUILD_DIR) $(BIN_DIR)

$(BUILD_DIR)/%.o: $(SRC_DIR)/%.c | dirs
	$(CC) $(CFLAGS) -c $< -o $@

$(BUILD_DIR)/%.o: $(SRC_DIR)/%.cpp | dirs
	$(CXX) $(CXXFLAGS) -c $< -o $@

$(BIN_DIR)/%: $(EXAMPLE_DIR)/%.c $(OBJECTS) | dirs
	$(CXX) $(CFLAGS) $< $(OBJECTS) -o $@ $(LDLIBS)

clean:
	@rm -rf $(BUILD_DIR)

fclean: clean
	@rm -rf $(BIN_DIR)

re: fclean all

help:
	@echo "Cibles : all, clean, fclean, re, help"
	@echo "Exemples compilés automatiquement : $(notdir $(EXAMPLES_BIN))"

.PHONY: all clean fclean re help dirs