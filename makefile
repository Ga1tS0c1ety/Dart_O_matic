# =====================================================
# Makefile — Projet C (PC + Raspberry Pi 64 bits)
# =====================================================

# Nom du projet
TARGET = mon_projet

# Répertoires
SRC_DIR    = src
INC_DIR    = include
BUILD_DIR  = build
BIN_DIR    = bin
LIB_DIR    = lib

# =====================================================
# ===== Build PC (natif)
# =====================================================

CC = gcc
CFLAGS = -Wall -Wextra -I$(INC_DIR)

LDFLAGS = -L$(LIB_DIR)
LDLIBS  = -lm

SRC = $(wildcard $(SRC_DIR)/*.c)
OBJ = $(SRC:$(SRC_DIR)/%.c=$(BUILD_DIR)/%.o)

# =====================================================
# ===== Build Raspberry Pi 64 bits (cross)
# =====================================================

RPI_CC = aarch64-linux-gnu-gcc

BUILD_DIR_RPI = build/rpi
BIN_DIR_RPI   = bin/rpi

RPI_CFLAGS = -Wall -Wextra -I$(INC_DIR)
RPI_LDFLAGS =
RPI_LDLIBS  = -lm

OBJ_RPI = $(SRC:$(SRC_DIR)/%.c=$(BUILD_DIR_RPI)/%.o)

# =====================================================
# ===== Règles principales
# =====================================================

all: $(BIN_DIR)/$(TARGET)

rpi: $(BIN_DIR_RPI)/$(TARGET)

# =====================================================
# ===== Link PC
# =====================================================

$(BIN_DIR)/$(TARGET): $(OBJ) | $(BIN_DIR)
	@echo " [LINK PC] $@"
	$(CC) $(CFLAGS) $(OBJ) -o $@ $(LDFLAGS) $(LDLIBS)

# =====================================================
# ===== Compilation PC
# =====================================================

$(BUILD_DIR)/%.o: $(SRC_DIR)/%.c | $(BUILD_DIR)
	@echo " [CC PC] $< → $@"
	$(CC) $(CFLAGS) -c $< -o $@

# =====================================================
# ===== Link Raspberry Pi
# =====================================================

$(BIN_DIR_RPI)/$(TARGET): $(OBJ_RPI) | $(BIN_DIR_RPI)
	@echo " [LINK RPI] $@"
	$(RPI_CC) $(OBJ_RPI) -o $@ $(RPI_LDFLAGS) $(RPI_LDLIBS)

# =====================================================
# ===== Compilation Raspberry Pi
# =====================================================

$(BUILD_DIR_RPI)/%.o: $(SRC_DIR)/%.c | $(BUILD_DIR_RPI)
	@echo " [CC RPI] $< → $@"
	$(RPI_CC) $(RPI_CFLAGS) -c $< -o $@

# =====================================================
# ===== Création des dossiers
# =====================================================

$(BIN_DIR) $(BUILD_DIR) $(BIN_DIR_RPI) $(BUILD_DIR_RPI):
	@mkdir -p $@

# =====================================================
# ===== Nettoyage
# =====================================================

clean:
	@echo " Suppression des fichiers objets..."
	rm -rf $(BUILD_DIR)/*.o
	rm -rf $(BUILD_DIR_RPI)/*.o

fclean: clean
	@echo " Suppression des exécutables..."
	rm -rf $(BIN_DIR)/$(TARGET)
	rm -rf $(BIN_DIR_RPI)/$(TARGET)

re: fclean all

# =====================================================
# ===== Aide
# =====================================================

help:
	@echo ""
	@echo " Commandes disponibles :"
	@echo "  make        → Compile la version PC"
	@echo "  make rpi    → Compile la version Raspberry Pi 64 bits"
	@echo "  make clean  → Supprime les fichiers objets"
	@echo "  make fclean → Supprime tout"
	@echo "  make re     → Recompile tout"
	@echo ""

.PHONY: all rpi clean fclean re help
