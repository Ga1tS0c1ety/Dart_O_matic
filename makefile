# Makefile — Projet C avec src/, include/, build/, bin/, et lib/

# Nom du projet (nom de l'exécutable final)
TARGET = mon_projet

# 📁 Répertoires principaux
SRC_DIR = src
INC_DIR = include
BUILD_DIR = build
BIN_DIR = bin
LIB_DIR = lib

#  Compilateur et options
CC = gcc
CFLAGS = -Wall -Wextra -I$(INC_DIR)

# Lien avec les bibliothèques dans lib/
LDFLAGS = -L$(LIB_DIR)
LDLIBS = -lm    

#  Trouve automatiquement tous les fichiers .c dans src/
SRC = $(wildcard $(SRC_DIR)/*.c)

#  Génère les noms des fichiers objets (.o) correspondants dans build/
OBJ = $(SRC:$(SRC_DIR)/%.c=$(BUILD_DIR)/%.o)


#  Règle par défaut

all: $(BIN_DIR)/$(TARGET)


#  Édition de liens : création de l’exécutable final

$(BIN_DIR)/$(TARGET): $(OBJ) | $(BIN_DIR)
	@echo " [LINK] $@"
	$(CC) $(CFLAGS) $(OBJ) -o $@ $(LDFLAGS) $(LDLIBS)

#  Compilation de chaque fichier source en fichier objet

$(BUILD_DIR)/%.o: $(SRC_DIR)/%.c | $(BUILD_DIR)
	@echo " [CC] $< → $@"
	$(CC) $(CFLAGS) -c $< -o $@

#  Création des dossiers s’ils n’existent pas

$(BIN_DIR) $(BUILD_DIR):
	@mkdir -p $@

# Nettoyage des fichiers intermédiaires

clean:
	@echo " Suppression des fichiers objets..."
	rm -rf $(BUILD_DIR)/*.o


#  Nettoyage complet

fclean: clean
	@echo " Suppression des exécutables..."
	rm -rf $(BIN_DIR)/$(TARGET)


#  Recompile tout depuis zéro

re: fclean all


#  Aide (documentation des commandes)

help:
	@echo ""
	@echo " Commandes disponibles :"
	@echo "  make        → Compile le projet (par défaut)"
	@echo "  make clean  → Supprime les fichiers objets (.o)"
	@echo "  make fclean → Supprime tout (binaires + build)"
	@echo "  make re     → Recompile tout depuis zéro"
	@echo "  make help   → Affiche cette aide"
	@echo ""


# 🧩 Règles spéciales

.PHONY: all clean fclean re help
