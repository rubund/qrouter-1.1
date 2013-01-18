prefix = /usr/local
bindir = ${prefix}/bin
cfgdir = ${prefix}/lib/qrouter

CC = gcc -m64 -fPIC
CFLAGS = -g
DFLAGS = -DHAVE_TYPES_H -DLIBDIR=\"${cfgdir}\" -DQVERSION=\"`cat VERSION`\"

SRC=    config.c qrouter.c node.c maze.c lef.c
OBJ=    config.o qrouter.o node.o maze.o lef.o
HEADER= config.h qrouter.h node.h maze.h lef.h

all: qrouter

qrouter: qrouter.o $(OBJ) $(HEADER)
	$(CC) $(CFLAGS) -o qrouter $(OBJ) -lm

install: qrouter
	mkdir -p ${bindir}
	cp qrouter ${bindir}

qrouter.o: qrouter.c  $(HEADER)
	$(CC) $(CFLAGS) -c qrouter.c $(DFLAGS)

config.o: config.c config.h qrouter.h
	$(CC) $(CFLAGS) -c config.c $(DFLAGS)

lef.o: lef.c $(HEADER)
	$(CC) $(CFLAGS) -c lef.c $(DFLAGS)

node.o: node.c $(HEADER)
	$(CC) $(CFLAGS) -c node.c $(DFLAGS)

maze.o: maze.c $(HEADER)
	$(CC) $(CFLAGS) -c maze.c $(DFLAGS)

clean:
	rm -rf *~ *.o

veryclean: clean
	rm -f qrouter

distclean: veryclean
