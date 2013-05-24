CC	= gcc
CFLAGS	+=
BIN_DIR = ../bin
INC_DIR	= ../include
LIB_DIR	= ../lib
INCL	= -I$(INC_DIR) `pkg-config --cflags opencv gtk+-2.0`
LIBS	= -L$(LIB_DIR) -lifdam `pkg-config --libs opencv gtk+-2.0`
OBJ	= imgfeatures.o utils.o sift.o kdtree.o minpq.o
BIN     = match

all: $(BIN) libifdam.a

libifdam.a: $(OBJ)
	ar rc $(LIB_DIR)/$@ $(OBJ)
	ranlib  $(LIB_DIR)/$@

match: libifdam.a match.c
	$(CC) $(CFLAGS) $(INCL) match.c -o $(BIN_DIR)/$@ $(LIBS)

imgfeatures.o: imgfeatures.c $(INC_DIR)/imgfeatures.h
	$(CC) $(CFLAGS) $(INCL) -c imgfeatures.c -o $@

utils.o: utils.c $(INC_DIR)/utils.h
	$(CC) $(CFLAGS) $(INCL) -c utils.c -o $@

sift.o: sift.c $(INC_DIR)/sift.h
	$(CC) $(CFLAGS) $(INCL) -c sift.c -o $@

kdtree.o: kdtree.c $(INC_DIR)/kdtree.h
	$(CC) $(CFLAGS) $(INCL) -c kdtree.c -o $@

minpq.o: minpq.c $(INC_DIR)/minpq.h
	$(CC) $(CFLAGS) $(INCL) -c minpq.c -o $@

clean:
	rm -f *~ *.o core
