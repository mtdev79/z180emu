ifeq ($(OS),Windows_NT)
	SOCKLIB = -lws2_32
endif

CCOPTS ?= -O3 -DSOCKETCONSOLE -std=gnu89

CFLAGS ?= $(CCOPTS) -g
LDFLAGS ?= $(SOCKLIB)

all: sc126 p112 markiv makedisk


sc126.o: sc126.c sconsole.h z180dbg.h z180/z180.h z180/z80daisy.h z180/z80common.h ds1202_1302/ds1202_1302.h sdcard/sdcard.h
sc126: sc126.o
sc126: z180/z180.o z180/z180dasm.o z180/z80daisy.o z180/z80scc.o z180/z180asci.o
sc126: ds1202_1302/rtc_sc126.o ds1202_1302/ds1202_1302.o
sc126: sdcard/sdcard.o

ds1202_1302/rtc_sc126.o: ds1202_1302/rtc.c ds1202_1302/rtc.h
	$(CC) $(CFLAGS) -Dmachine_name=\"sc126\" -DHAVE_SYS_TIME_H -DHAVE_GETTIMEOFDAY -o $@ -c $<

markiv: ide.o z180.o z180dasm.o z80daisy.o z80scc.o z180asci.o markiv.o rtc_markiv.o ds1202_1302.o
	$(CC) $(CCOPTS) -s -o markiv $^ $(SOCKLIB)

markiv.o: markiv.c sconsole.h z180dbg.h z180/z180.h z180/z80daisy.h z180/z80common.h ds1202_1302/ds1202_1302.h
	$(CC) $(CCOPTS) -c markiv.c

rtc_markiv.o: ds1202_1302/rtc.c ds1202_1302/rtc.h
	cd ds1202_1302 ; $(CC) $(CCOPTS) -Dmachine_name=\"markiv\" -DHAVE_SYS_TIME_H -DHAVE_GETTIMEOFDAY -o ../rtc_markiv.o -c rtc.c 

p112: ide.o z180.o z180dasm.o z80daisy.o z80scc.o z180asci.o p112.o rtc_p112.o ds1202_1302.o fdc.o fdd.o fdd_86f.o fdd_common.o fdd_img.o sio_fdc37c66x.o ins8250.o
	$(CC) $(CCOPTS) -s -o p112 $^ $(SOCKLIB)

p112.o:	p112.c sconsole.h z180dbg.h z180/z180.h z180/z80daisy.h z180/z80common.h ds1202_1302/ds1202_1302.h
	$(CC) $(CCOPTS) -c p112.c

ide.o:	ide/ide.c ide/ide.h
	cd ide ; $(CC) $(CCOPTS) -o ../ide.o -c ide.c 

z180.o:	z180/z180.c z180/z180cb.c z180/z180dd.c z180/z180ed.c z180/z180fd.c z180/z180op.c z180/z180xy.c z180/z180.h z180/z180ops.h z180/z180tbl.h z180/z80daisy.h z180/z80common.h
	cd z180 ; $(CC) $(CCOPTS) -o ../z180.o -c z180.c 

z180dasm.o: z180/z180dasm.c z180/z180.h z180/z80common.h
	cd z180 ; $(CC) $(CCOPTS) -o ../z180dasm.o -c z180dasm.c 

z80daisy.o: z180/z80daisy.c z180/z180.h z180/z80daisy.h z180/z80common.h
	cd z180 ; $(CC) $(CCOPTS) -o ../z80daisy.o -c z80daisy.c 

z80scc.o: z180/z80scc.c z180/z80scc.h z180/z80daisy.h z180/z80common.h
	cd z180 ; $(CC) $(CCOPTS) -o ../z80scc.o -c z80scc.c 

z180asci.o: z180/z180asci.c z180/z180asci.h z180/z180.h z180/z80common.h
	cd z180 ; $(CC) $(CCOPTS) -o ../z180asci.o -c z180asci.c 

rtc_p112.o: ds1202_1302/rtc.c ds1202_1302/rtc.h
	cd ds1202_1302 ; $(CC) $(CCOPTS) -Dmachine_name=\"p112\" -DHAVE_SYS_TIME_H -DHAVE_GETTIMEOFDAY -o ../rtc_p112.o -c rtc.c 

ds1202_1302.o: ds1202_1302/ds1202_1302.c ds1202_1302/ds1202_1302.h ds1202_1302/rtc.h
	cd ds1202_1302 ; $(CC) $(CCOPTS) -o ../ds1202_1302.o -c ds1202_1302.c 

fdc.o: fdc/fdc.c fdc/fdc.h fdc/fdd.h fdc/86box.h
	cd fdc ; $(CC) $(CCOPTS) -o ../fdc.o -c fdc.c 

fdd.o: fdc/fdd.c fdc/fdc.h fdc/fdd.h fdc/fdd_86f.h fdc/fdd_img.h fdc/86box.h
	cd fdc ; $(CC) $(CCOPTS) -o ../fdd.o -c fdd.c 

fdd_86f.o: fdc/fdd_86f.c fdc/fdc.h fdc/fdd.h fdc/fdd_86f.h fdc/86box.h
	cd fdc ; $(CC) $(CCOPTS) -o ../fdd_86f.o -c fdd_86f.c 

fdd_common.o: fdc/fdd_common.c fdc/fdd.h fdc/fdd_common.h fdc/86box.h
	cd fdc ; $(CC) $(CCOPTS) -o ../fdd_common.o -c fdd_common.c 

fdd_img.o: fdc/fdd_img.c fdc/fdc.h fdc/fdd.h fdc/fdd_img.h fdc/86box.h
	cd fdc ; $(CC) $(CCOPTS) -o ../fdd_img.o -c fdd_img.c 

sio_fdc37c66x.o: fdc/sio_fdc37c66x.c fdc/fdc.h fdc/fdd.h fdc/sio.h fdc/86box.h ins8250/ins8250.h fdc/lpt.h
	cd fdc ; $(CC) $(CCOPTS) -o ../sio_fdc37c66x.o -c sio_fdc37c66x.c 

#serial.o: fdc/serial.c fdc/serial.h fdc/86box.h
#	cd fdc ; $(CC) $(CCOPTS) -o ../serial.o -c serial.c 

ins8250.o: ins8250/ins8250.c ins8250/ins8250.h
	cd ins8250 ; $(CC) $(CCOPTS) -o ../ins8250.o -c ins8250.c

makedisk: makedisk.o ide.o
	$(CC) $(CCOPTS) -s -o makedisk $^

makedisk.o: ide/makedisk.c
	cd ide ; $(CC) $(CCOPTS) -o ../makedisk.o -c makedisk.c
