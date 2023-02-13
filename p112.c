/*
 * p112.c - P112 emulation.
 *
 * Copyright (c) Michal Tomek 2018-2019 <mtdev79b@gmail.com>
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA
 *  02111-1307  USA.
 *
 */

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdint.h>
#include <sys/time.h>

#ifdef SOCKETCONSOLE
#define BASE_PORT 10180
#define MAX_SOCKET_PORTS 2
int enable_aux = 1;
#include "sconsole.h"
#endif

#ifdef _WIN32
#include <windows.h>
#include <conio.h>
#include <fcntl.h>
#define fileno _fileno
#else
#include <signal.h>
#endif

#include "z180/z180.h"
#include "ide/ide.h"
#include "ds1202_1302/ds1202_1302.h"
#include "fdc/fdd.h"
#include "fdc/fdc.h"
#include "fdc/sio.h"
#include "ins8250/ins8250.h"

UINT8 _ram[1048576];
UINT8 _rom[32768];

#define RAMARRAY _ram
#define ROMARRAY _rom
#include "z180dbg.h"

struct ide_controller *ic0;
FILE* if00;
int ifd00;
struct ide_drive *id00;

uint8_t idemap[16] = {0,0,0,0,0,0,ide_altst_r,0,
				ide_data,ide_error_r,ide_sec_count,ide_sec_num,ide_cyl_low,ide_cyl_hi,ide_dev_head,ide_status_r};
uint8_t ide_lh_flop; // LH in Tilmann's gide1.pds
uint8_t ide_lo_byte; // IC3 '646 in Tilmann's schematic
uint8_t ide_hi_byte; // IC4 '646 in Tilmann's schematic

#define ESCC_DIVISOR 8
#define INS8250_DIVISOR 35
unsigned int escc_clock = ESCC_DIVISOR;
unsigned int ins8250_clock = INS8250_DIVISOR;

rtc_ds1202_1302_t *rtc;

#define BOOT_FDD 1 // drive 0,1 swapped

fdc37c66x_t *fdc37c665;

struct z180_device *cpu;
                       
UINT8 ram_read(offs_t A) {
 	return _ram[A];
}

void ram_write(offs_t A,UINT8 V) {
    _ram[A]=V;
}

UINT8 rom_read(offs_t A) {
 	return _rom[A];
}

int console_char_available() {
#ifdef SOCKETCONSOLE
	  return char_available_socket_port(0);
#else
      return _kbhit();
#endif
}

void escc_tx(device_t *device, int channel, UINT8 Value) {
	if (channel==0) {
	  //printf("TX: %c", Value);
#ifdef SOCKETCONSOLE
	  tx_socket_port(0, Value);
#else
	  fputc(Value,stdout);
#endif
	  //printf("\n");
	}
}

int escc_rx(device_t *device, int channel) {
	int ioData;
	if (channel==0) {
	  //ioData = 0xFF;
	  if(console_char_available()) {
#ifdef SOCKETCONSOLE
	    ioData = rx_socket_port(0);
#else
	    //printf("RX\n");
        ioData = getch();
#endif
		return ioData;
	  }
	}
	return -1;
}

int irq0ackcallback(device_t *device,int irqnum) {
	return 0;
}

int aux_char_available() {
#ifdef SOCKETCONSOLE
	  return char_available_socket_port(1);
#else
	return 0;
#endif
}

void aux_tx(device_t *device, int channel, UINT8 Value) {
	if (channel==0) {
#ifdef SOCKETCONSOLE
	  tx_socket_port(1, Value);
#endif
	}
}

int aux_rx(device_t *device, int channel) {
	int ioData;
	if (channel==0) {
	  if(aux_char_available()) {
#ifdef SOCKETCONSOLE
	    ioData = rx_socket_port(1);
#endif
		return ioData;
	  }
	}
	return -1;
}

void aux_int_state_cb(device_t *device, int state) {
	if (VERBOSE) printf("SER1 int: %d\n",state);
	z180_set_irq_line(cpu,2,state);
}

UINT8 parport_read(device_t *device, int channel) {
	UINT8 x;
	if(channel==0) {
		x=ds1202_1302_read_data_line(rtc);
		if (VERBOSE) printf("RTC read: %02x\n",x);
		return x;
	}
	return 0;
}


void parport_write(device_t *device, int channel, UINT8 value) {
	if(channel==0) {
		// DS1302
		// PA0=IO,PA1=CLK,PA2=/RST
		if (VERBOSE) printf("RTC write: %02x\n",value);
		ds1202_1302_set_lines(rtc,value&4?1:0,value&2?1:0,value&1);
	}
}

UINT8 io_read (offs_t Port) {
	uint8_t ioData = 0;
 	uint16_t idedata;

	Port &= 0xff;
	// GIDE emulation. See Tilmann's gide1.pds
	if (Port >= 0x50 && Port <= 0x5f ) {
		if (Port >= 0x59 && Port <= 0x5f)
			ide_lh_flop = 0;
		if (Port != 0x58)
			ioData = ide_read8(ic0,idemap[Port-0x50]);
		else {
			if (!ide_lh_flop) {
				idedata = ide_read16(ic0,idemap[Port-0x50]);
				ioData = idedata & 0xff;
				if (VERBOSE) printf("GIDE: readlo %x\n",ioData);
				ide_hi_byte = idedata >> 8;
			}
			else
			{
				ioData = ide_hi_byte;
				if (VERBOSE) printf("GIDE: readhi %x\n",ioData);
			}
			ide_lh_flop = !ide_lh_flop;
		}
	}
	else if (Port >= 0x80 && Port <= 0xbf) // IOCS
	{
		if (Port <= 0x9f) // not DMA
			ioData = fdc37c66x_read(0x3b0 | ((Port & 0x10)<<2) | (Port & 0xf), NULL);
		else
		{
			if (VERBOSE) printf("FDC DMA IO read %02x\n",fdc37c665->fdc->dma_buf);//fflush(stdout);
			//z180_set_dreq0(cpu,0);
			//ioData = fdc_dma_write_buf; // return whatever was on the data bus
			ioData = fdc37c665->fdc->dma_buf;
			fdc_dma_ack(fdc37c665->fdc);
		}
	}
	else
		printf("IO: Bogus read %x\n",Port);
	return ioData;
}

void io_write (offs_t Port,UINT8 Value) {
	Port &= 0xff;

	// GIDE emulation
	if (Port >= 0x50 && Port <= 0x5f ) {
		if (Port >= 0x59 && Port <= 0x5f)
			ide_lh_flop = 0;
		if (Port != 0x58)
			ide_write8(ic0,idemap[Port-0x50],Value);
		else {
			if (!ide_lh_flop) {
				ide_lo_byte = Value;
			}
			else
			{
				ide_write16(ic0,idemap[Port-0x50],(Value << 8)| ide_lo_byte);
			}
			ide_lh_flop = !ide_lh_flop;
		}
	}
	else if (Port >= 0x80 && Port <= 0xbf) // IOCS
	{
		if (Port <= 0x9f) // not DMA
			fdc37c66x_write(0x3b0 | ((Port & 0x10)<<2) | (Port & 0xf), Value, NULL);
		else
		{
			if (VERBOSE) printf("FDC DMA IO write: %02x\n",Value);
			//z180_set_dreq0(cpu,0);
			fdc37c665->fdc->dma_buf = Value;
			fdc_dma_ack(fdc37c665->fdc);
		}
	}
	else
		printf("IO: Bogus write %x:%x\n",Port,Value);
}

void do_timers() {
	//16X clock for ESCC
	//printf("escc_clk:%d\n",escc_clock);
	if (!--escc_clock) {
		z80scc_channel_device_timer(cpu->z80scc->m_chanA);
		z80scc_channel_device_timer(cpu->z80scc->m_chanB);
		fdc_poll(fdc37c665->fdc);
		fdd_poll(BOOT_FDD);
		escc_clock = ESCC_DIVISOR;
	} else if (escc_clock == ESCC_DIVISOR/2) { // FDC is roughly 2times faster
		fdc_poll(fdc37c665->fdc);
		fdd_poll(BOOT_FDD);
	}
	if (!--ins8250_clock) {
		ins8250_device_timer(fdc37c665->serial1);
		ins8250_clock = INS8250_DIVISOR;
	}
}

void boot1dma () {
   FILE* f;
   if (!(f=fopen("p112rom.bin","rb"))) {
     printf("No ROM found.\n");
	 g_quit = 1;
   } else {
     fread(&_rom[0],1,32768,f);
     fclose(f);
   }
}

void io_device_update() {
#ifdef SOCKETCONSOLE
    // check socket open and optionally reopen it
    if (!is_connected_socket_port(0)) open_socket_port(0);
	if (enable_aux && !is_connected_socket_port(1)) open_socket_port(1);
#endif
}

void CloseIDE() {
   ide_free(ic0);
}

void InitIDE() {
   ic0=ide_allocate("IDE0");
   if (if00=fopen("ide00.dsk","r+b")) {
     ifd00=fileno(if00);
     ide_attach(ic0,0,ifd00);
   }
   ide_reset_begin(ic0);
   atexit(CloseIDE);
}

#ifndef _WIN32
void sigint_handler(int s)	{
	// POSIX SIGINT handler
	// do nothing
}

void sigquit_handler(int s)	{
	// POSIX SIGQUIT handler
	printf("\nExiting emulation.\n");
	shutdown_socket_ports(); // close sockets to prevent waiting for a connection
	g_quit = 1; // make sure atexit is called
}
#endif

void disableCTRLC() {
#ifdef _WIN32
	HANDLE consoleHandle = GetStdHandle(STD_INPUT_HANDLE);
	DWORD consoleMode;
	GetConsoleMode(consoleHandle,&consoleMode);
	SetConsoleMode(consoleHandle,consoleMode&~ENABLE_PROCESSED_INPUT);
#else
	signal(SIGINT, sigint_handler);
#endif
}

struct address_space ram = {ram_read,ram_write,ram_read};
struct address_space rom = {rom_read,NULL,rom_read};
struct address_space iospace = {io_read,io_write,NULL};

void destroy_rtc()
{
	ds1202_1302_destroy(rtc,1);
}

void fdc_int(void *device, int state) {
	if (VERBOSE) printf("FDC int: %d\n",state);
	z180_set_irq_line(cpu,1,state);
}

void fdc_dma_req(void *device, int state) {
	if (VERBOSE) printf("FDC dreq: %d\n",state);
	z180_set_dreq0(cpu, state);
}

int main(int argc, char** argv)
{
	printf("z180emu v1.0 P112\n");

	disableCTRLC();
#ifndef _WIN32
	// on POSIX, route SIGQUIT (CTRL+\) to graceful shutdown
	signal(SIGQUIT, sigquit_handler);
#endif
	// on MINGW, keep CTRL+Break (and window close button) enabled
	// MINGW always calls atexit in these cases

#ifdef SOCKETCONSOLE
	init_TCPIP();
	init_socket_port(0); // ESCC Console
	init_socket_port(1); // FDC AUX
	atexit(shutdown_socket_ports);
#endif
	io_device_update(); // wait for serial socket connections

	if (argc==2 && !strcmp(argv[1],"d")) starttrace = 0;
	else if (argc==3 && !strcmp(argv[1],"d")) starttrace = atoll(argv[2]);
	VERBOSE = starttrace==0?1:0;

#ifdef _WIN32
	setmode(fileno(stdout), O_BINARY);
#endif

	boot1dma();
	InitIDE();

	rtc = ds1202_1302_init("RTC",1302);
	ds1202_1302_reset(rtc);
	atexit(destroy_rtc);

	fdc37c665 = fdc37c665_init(fdc_int, fdc_dma_req, aux_int_state_cb, aux_rx, aux_tx, NULL,NULL,NULL);
	fdd_init();
	fdd_set_type(BOOT_FDD,fdd_get_from_internal_name("35_2hd"));
	fdd_set_turbo(BOOT_FDD,1);
	fdd_load(BOOT_FDD,"p112-fdd1.img");
			
	cpu = cpu_create_z180("Z182",Z180_TYPE_Z182,16000000,&ram,&rom,&iospace,irq0ackcallback,NULL/*daisychain*/,
		NULL,NULL,NULL,NULL,escc_rx,escc_tx,parport_read,parport_write);
	//printf("1\n");fflush(stdout);
	cpu_reset_z180(cpu);
	//printf("2\n");fflush(stdout);

	struct timeval t0;
	struct timeval t1;
	gettimeofday(&t0, 0);
	int runtime=50000;

	//g_quit = 0;
	while(!g_quit) {
		if(instrcnt>=starttrace) VERBOSE=1;
		cpu_execute_z180(cpu,10000);
		//printf("3\n");fflush(stdout);
		io_device_update();
		/*if (!(--runtime))
			g_quit=1;*/
	}
	gettimeofday(&t1, 0);
	printf("instrs:%llu, time:%g\n",instrcnt, (t1.tv_sec - t0.tv_sec) * 1000.0f + (t1.tv_usec - t0.tv_usec) / 1000.0f);

}
