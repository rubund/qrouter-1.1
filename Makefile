#
# qrouter Makefile
#

# Main compiler arguments
CFLAGS = -g -O2
DEFS = -DPACKAGE_NAME=\"\" -DPACKAGE_TARNAME=\"\" -DPACKAGE_VERSION=\"\" -DPACKAGE_STRING=\"\" -DPACKAGE_BUGREPORT=\"\" -DPACKAGE_URL=\"\" -DSTDC_HEADERS=1 -DHAVE_SETENV=1 -DHAVE_PUTENV=1 -DQROUTER_LIB_DIR=\"/usr/local/share/qrouter\" -DVERSION=\"1.1\"
LIBS = 
LDFLAGS = 
INSTALL = /usr/bin/install -c
prefix = /usr/local

OBJECTS = qrouter.o maze.o node.o config.o lef.o def.o
SOURCES := $(patsubst %.o,%.c,$(OBJECTS))
TARGETS := qrouter$(EXEEXT)

BININSTALL = ${prefix}/bin

all: $(TARGETS)

qrouter$(EXEEXT): $(OBJECTS)
	$(CC) $(LDFLAGS) $(OBJECTS) -o $@ $(LIBS) -lm

install:
	@echo "Installing qrouter"
	$(INSTALL) -d ${BININSTALL}
	$(INSTALL) qrouter ${BININSTALL}

uninstall:
	$(RM) -f ${BININSTALL}/qrouter

clean:
	$(RM) -f $(OBJECTS)
	$(RM) -f $(TARGETS)

veryclean:
	$(RM) -f $(OBJECTS)
	$(RM) -f $(TARGETS)

.c.o:
	$(CC) $(CFLAGS) $(CPPFLAGS) $(DEFS) -c $< -o $@
