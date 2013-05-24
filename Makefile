CC	= gcc
CFLAGS	+= -O3
BIN_DIR	= ./bin
SRC_DIR	= ./src
INC_DIR	= ./include
LIB_DIR	= ./lib
BIN	= match

all: $(BIN) libifdam.a

libifdam.a:
	make -C $(SRC_DIR) $@

$(BIN):
	make -C $(SRC_DIR) $@

clean:
	make -C $(SRC_DIR) $@;	\
	make -C $(INC_DIR) $@;	\

distclean: clean
	rm -f $(LIB_DIR)/*
	rm -f $(BIN_DIR)/*
