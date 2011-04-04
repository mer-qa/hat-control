nilibs=-lnidaqmxbase 
ljlibs=-lm -llabjackusb -lpthread
includes=
flags= -O2 -Wall -g
cc=gcc

libs = shmemlib.o u3.o
exes = hat_drv hat_ctrl

all: $(exes)

shmemlib.o: shmemlib.c
	$(cc) $(includes) $(flags) -c shmemlib.c shmemlib.h

u3: u3.c
	$(cc) $(includes) $(flags) u3.c $(ljlibs) -o u3.o

hat_drv: hat_drv.c u3.o shmemlib.o
	$(cc) $(includes) $(flags) hat_drv.c u3.o shmemlib.o $(ljlibs) -o hat_drv

hat_ctrl: hat_ctrl.c shmemlib.o
	$(cc) $(includes) $(flags) hat_ctrl.c shmemlib.o $(ljlibs) -o hat_ctrl

clean:
	rm -f $(exes) $(libs) *.gch
