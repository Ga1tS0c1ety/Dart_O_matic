# =====================================================
# Makefile unique : PC (natif) + Raspberry Pi 64 bits (natif)
# =====================================================

CXX      = g++
CC       = gcc
CFLAGS   = -Wall -Wextra -Iinclude
CXXFLAGS = -Wall -Wextra -Iinclude

# Détection automatique d'OpenCV via pkg-config (opencv4 ou opencv selon le système)
PKG_CONFIG_NAME = opencv4
ifeq ($(shell pkg-config --exists opencv4 && echo 1 || echo 0),0)
    PKG_CONFIG_NAME = opencv
endif

# Si pkg-config trouve OpenCV → on l'utilise
ifeq ($(shell pkg-config --exists $(PKG_CONFIG_NAME) && echo 1),1)
    CXXFLAGS += $(shell pkg-config --cflags $(PKG_CONFIG_NAME))
    LDLIBS   += $(shell pkg-config --libs $(PKG_CONFIG_NAME))
    USE_OPENCV = 1
    $(info === OpenCV détecté via pkg-config ($(PKG_CONFIG_NAME)) ===)
else
    $(warning === OpenCV non détecté → compilation sans project_point_opencv ===)
    USE_OPENCV = 0
endif

LDLIBS += -lm

# Répertoires
BUILD_DIR = build
BIN_DIR   = bin
SRC_DIR   = src
EXAMPLE_DIR = example

# Sources et objets
C_SOURCES   = $(wildcard $(SRC_DIR)/*.c)
CPP_SOURCES = $(wildcard $(SRC_DIR)/*.cpp)

OBJECTS = $(C_SOURCES:$(SRC_DIR)/%.c=$(BUILD_DIR)/%.o)
ifeq ($(USE_OPENCV),1)
    OBJECTS += $(CPP_SOURCES:$(SRC_DIR)/%.cpp=$(BUILD_DIR)/%.o)
endif

# Exemples (un exécutable par fichier .c dans example/)
EXAMPLES_SRC = $(wildcard $(EXAMPLE_DIR)/*.c)
EXAMPLES_BIN = $(patsubst $(EXAMPLE_DIR)/%.c, $(BIN_DIR)/%, $(EXAMPLES_SRC))

# =====================================================
# Règles
# =====================================================

all: dirs $(EXAMPLES_BIN)

dirs:
	@mkdir -p $(BUILD_DIR) $(BIN_DIR)

# Compilation C pur
$(BUILD_DIR)/%.o: $(SRC_DIR)/%.c | dirs
	$(CC) $(CFLAGS) -c $< -o $@

# Compilation C++ (seulement si OpenCV disponible)
$(BUILD_DIR)/%.o: $(SRC_DIR)/%.cpp | dirs
	$(CXX) $(CXXFLAGS) -c $< -o $@

# Link des exemples (on utilise g++ car il peut linker du C et C++)
$(BIN_DIR)/%: $(EXAMPLE_DIR)/%.c $(OBJECTS) | dirs
	$(CXX) $(CFLAGS) $< $(OBJECTS) -o $@ $(LDLIBS)

# Nettoyage
clean:
	rm -rf $(BUILD_DIR)

fclean: clean
	rm -rf $(BIN_DIR)

re: fclean all

.PHONY: all clean fclean re dirs