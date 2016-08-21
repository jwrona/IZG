################################################################################
# IZG project, framework compilation
################################################################################

################################################################################
# promenne prekladu

ifeq ($(OS), Windows_NT)
    INCS = -I../SDL-1.2.15/include -IQ:/mingw/SDL/SDL
    LIBS = -lmingw32 -lSDLmain -lSDL -L../SDL-1.2.15/lib -LQ:/mingw/SDL
#    REMOVE = del
    REMOVE = rm -f
    BIN = izg_proj.exe
else
    INCS = -I/usr/include/SDL
    LIBS = -lSDL -lm
#    LIBS=-m32 -lSDL
#    LIBS=-m64 -lSDL
    REMOVE = rm -f
    BIN = izg_proj
endif

CC = gcc

CFLAGS = -g3 -Wall $(INCS) -Wno-unused-function -fmessage-length=0
#CFLAGS = -m32 -g3 -Wall $(INCS) -Wno-unused-function -fmessage-length=0
#CFLAGS = -m64 -g3 -Wall $(INCS) -Wno-unused-function -fmessage-length=0

################################################################################
# obecny cil kompilace
all: $(BIN)

################################################################################
# linkovani vysledneho souboru
$(BIN):	main.o student.o render.o transform.o model.o vector.o color.o curve.o
	$(CC) main.o student.o render.o transform.o model.o vector.o color.o curve.o -o $(BIN) $(LIBS)
#	strip $(BIN)

################################################################################
# kompilace hlavniho souboru
main.o: main.c
	$(CC) -c main.c -o main.o $(CFLAGS)

################################################################################
# kompilace studentskeho souboru
student.o: student.c student.h
	$(CC) -c student.c -o student.o $(CFLAGS)

################################################################################
# kompilace dilcich souboru
render.o: render.c render.h
	$(CC) -c render.c -o render.o $(CFLAGS)
transform.o: transform.c transform.h
	$(CC) -c transform.c -o transform.o $(CFLAGS)
model.o: model.c model.h
	$(CC) -c model.c -o model.o $(CFLAGS)
vector.o: vector.c vector.h
	$(CC) -c vector.c -o vector.o $(CFLAGS)
color.o: color.c color.h
	$(CC) -c color.c -o color.o $(CFLAGS)
curve.o: curve.c curve.h
	$(CC) -c curve.c -o curve.o $(CFLAGS)

################################################################################
# cil vycisteni souboru prekladu
clean: clear
clear:
	$(REMOVE) *.o $(BIN)

################################################################################
################################################################################
