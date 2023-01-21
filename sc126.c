/*
 * markiv.c - Mark IV emulation.
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
#define MAX_SOCKET_PORTS 1
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
#include "ds1202_1302/ds1202_1302.h"
#include "sdcard/sdcard.h"

// so far only 512k EEPROM+512k RAM is supported
UINT8 _ram[1048576]; // lo 512k is ROM

#define RAMARRAY _ram
#define ROMARRAY NULL
#include "z180dbg.h"

unsigned int asci_clock = 16;

rtc_ds1202_1302_t *rtc;

UINT8 xmem_bank;

struct z180_device *cpu;
                       
UINT8 ram_read(offs_t A) {
    A &= 0xfffff;
    return _ram[A];
}

void ram_write(offs_t A,UINT8 V) {
    A &= 0xfffff;

    // low 512k is eprom
    if (A >= 524288) {
        _ram[A]=V;
    }
}

int char_available() {
#ifdef SOCKETCONSOLE
	  return char_available_socket_port(0);
#else
      return _kbhit();
#endif
}

void asci_tx(device_t *device, int channel, UINT8 Value) {
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

int asci_rx(device_t *device, int channel) {
	int ioData;
	if (channel==0) {
	  //ioData = 0xFF;
	  if(char_available()) {
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
}

void print_bin_u8(UINT8 v) {
    int bits = 8;
    while(bits) {
        if (v & 0x80) {
            printf("1");
        } else {
            printf("0");
        }
        v <<= 1;
        bits--;
    }
}

// The serial port on the Z180 is backwards for SPI sdcards,
// so here is a quick table to bitswap a byte
UINT8 mirtab[256] = {
    0x00, 0x80, 0x40, 0xC0, 0x20, 0xA0, 0x60, 0xE0, 0x10, 0x90, 0x50, 0xD0,
    0x30, 0xB0, 0x70, 0xF0, 0x08, 0x88, 0x48, 0xC8, 0x28, 0xA8, 0x68, 0xE8,
    0x18, 0x98, 0x58, 0xD8, 0x38, 0xB8, 0x78, 0xF8, 0x04, 0x84, 0x44, 0xC4,
    0x24, 0xA4, 0x64, 0xE4, 0x14, 0x94, 0x54, 0xD4, 0x34, 0xB4, 0x74, 0xF4,
    0x0C, 0x8C, 0x4C, 0xCC, 0x2C, 0xAC, 0x6C, 0xEC, 0x1C, 0x9C, 0x5C, 0xDC,
    0x3C, 0xBC, 0x7C, 0xFC, 0x02, 0x82, 0x42, 0xC2, 0x22, 0xA2, 0x62, 0xE2,
    0x12, 0x92, 0x52, 0xD2, 0x32, 0xB2, 0x72, 0xF2, 0x0A, 0x8A, 0x4A, 0xCA,
    0x2A, 0xAA, 0x6A, 0xEA, 0x1A, 0x9A, 0x5A, 0xDA, 0x3A, 0xBA, 0x7A, 0xFA,
    0x06, 0x86, 0x46, 0xC6, 0x26, 0xA6, 0x66, 0xE6, 0x16, 0x96, 0x56, 0xD6,
    0x36, 0xB6, 0x76, 0xF6, 0x0E, 0x8E, 0x4E, 0xCE, 0x2E, 0xAE, 0x6E, 0xEE,
    0x1E, 0x9E, 0x5E, 0xDE, 0x3E, 0xBE, 0x7E, 0xFE, 0x01, 0x81, 0x41, 0xC1,
    0x21, 0xA1, 0x61, 0xE1, 0x11, 0x91, 0x51, 0xD1, 0x31, 0xB1, 0x71, 0xF1,
    0x09, 0x89, 0x49, 0xC9, 0x29, 0xA9, 0x69, 0xE9, 0x19, 0x99, 0x59, 0xD9,
    0x39, 0xB9, 0x79, 0xF9, 0x05, 0x85, 0x45, 0xC5, 0x25, 0xA5, 0x65, 0xE5,
    0x15, 0x95, 0x55, 0xD5, 0x35, 0xB5, 0x75, 0xF5, 0x0D, 0x8D, 0x4D, 0xCD,
    0x2D, 0xAD, 0x6D, 0xED, 0x1D, 0x9D, 0x5D, 0xDD, 0x3D, 0xBD, 0x7D, 0xFD,
    0x03, 0x83, 0x43, 0xC3, 0x23, 0xA3, 0x63, 0xE3, 0x13, 0x93, 0x53, 0xD3,
    0x33, 0xB3, 0x73, 0xF3, 0x0B, 0x8B, 0x4B, 0xCB, 0x2B, 0xAB, 0x6B, 0xEB,
    0x1B, 0x9B, 0x5B, 0xDB, 0x3B, 0xBB, 0x7B, 0xFB, 0x07, 0x87, 0x47, 0xC7,
    0x27, 0xA7, 0x67, 0xE7, 0x17, 0x97, 0x57, 0xD7, 0x37, 0xB7, 0x77, 0xF7,
    0x0F, 0x8F, 0x4F, 0xCF, 0x2F, 0xAF, 0x6F, 0xEF, 0x1F, 0x9F, 0x5F, 0xDF,
    0x3F, 0xBF, 0x7F, 0xFF,
};

UINT8 rtc_latch;
struct sdcard_device sd0;

void sci_write(device_t *device, int channel, UINT8 data) {
    if ((rtc_latch & 0x04) == 0) {
        // The /CS0 is active
        sdcard_write(&sd0, mirtab[data]);
    } else {
        printf("IO:CSI:   TRDR   = 0x%02x (latch=", data);
        print_bin_u8(rtc_latch);
        printf(")\n");
    }
}

int sci_read(device_t *device, int channel) {
    int result = 0xff;
    if ((rtc_latch & 0x04) == 0) {
        // The /CS0 is active
        result = mirtab[sdcard_read(&sd0, 0xff)];
    } else {
        printf("IO:CSI:   TRDR          (latch=");
        print_bin_u8(rtc_latch);
        printf(")\n");
    }
    return result;
}

UINT8 io_read (offs_t Port) {
    Port &= 0xff;
    uint8_t ioData = 0;
    switch (Port) {
        case 0x0c:
            ioData |= ds1202_1302_read_data_line(rtc);
            // D7 = I2C SDA read TODO
            break;
        // case 0x0d: LED port, but doesnt support read
              
        default:
            printf("IO:READ:  0x%04x\n",Port);
            break;
    }
    return ioData;
}

void io_write (offs_t Port,UINT8 Value) {
    Port &= 0xff;
    switch (Port) {
        case 0x0c:
            // D0 = I2C SCL
            // D1 = Flash Select
            // D2 = /SD_CS1
            // D3 = /SD_CS2
    
            ds1202_1302_set_lines(
                rtc,
                Value&0x10?1:0,                 // D4 = /CE
                Value&0x40?1:0,                 // D6 = SCLK
                Value&0x20?0:(Value&0x80?1:0)   // D5 = /WE, D7 = Input
            );

            rtc_latch = Value;

            // Try to avoid printing all RTC accesses 
            if ((Value & 0x10) && (Value & 0xf)!=0) {
                printf("IO:CTRL:  ");
                print_bin_u8(Value);
                printf("\n");
            }

            break;

        case 0x0d:
            //printf("IO:LED:  ");
            //print_bin_u8(Value);
            //printf("\n");
            break;

        case 0x12:
        case 0x13:
        case 0x14:
        case 0x15:
        case 0x16:
            // IDE IO=0x10
            break;

        case 0x20:
        case 0x22:
        case 0x23:
            // PPIDE IO=0x20
            break;

        default:
            printf("IO:WRITE: 0x%04x = 0x%02x\n",Port,Value);
            break;
    }
}

void do_timers() {
	//16X clock for ASCI
	//printf("asci_clk:%d\n",asci_clock);
	if (!--asci_clock) {
		z180asci_channel_device_timer(cpu->z180asci->m_chan0);
		z180asci_channel_device_timer(cpu->z180asci->m_chan1);
		asci_clock = 16;
	}
}

void boot1dma () {
   FILE* f;
   if (!(f=fopen("sc126rom.bin","rb"))) {
     printf("No ROM found.\n");
	 g_quit = 1;
   } else {
     fread(&_ram[0],1,524288,f);
     fclose(f);
   }
}

void io_device_update() {
#ifdef SOCKETCONSOLE
    // check socket open and optionally reopen it
    if (!is_connected_socket_port(0)) open_socket_port(0);
#endif
}

#ifndef _WIN32
void sigint_handler(int s)	{
	// POSIX SIGINT handler
	// do nothing
}

void sigquit_handler(int s)	{
	// POSIX SIGQUIT handler
	// make sure atexit is called
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
//struct address_space rom = {rom_read,NULL,rom_read};
struct address_space iospace = {io_read,io_write,NULL};

void destroy_rtc()
{
	ds1202_1302_destroy(rtc,1);
}

int main(int argc, char** argv)
{
	printf("z180emu v1.0 sc126\n");

	disableCTRLC();
#ifndef _WIN32
	// on POSIX, route SIGQUIT (CTRL+\) to graceful shutdown
	signal(SIGQUIT, sigquit_handler);
#endif
	// on MINGW, keep CTRL+Break (and window close button) enabled
	// MINGW always calls atexit in these cases

#ifdef SOCKETCONSOLE
	init_TCPIP();
	init_socket_port(0); // ASCI Console
	atexit(shutdown_socket_ports);
#endif
	io_device_update(); // wait for serial socket connections

        if (argc > 1 && !strcmp(argv[1],"d")) {
            if (argc == 3) {
                starttrace = atoll(argv[2]);
            } else {
                starttrace = 0;
            }
        }
	VERBOSE = starttrace==0?1:0;

#ifdef _WIN32
	setmode(fileno(stdout), O_BINARY);
#endif

	boot1dma();
        sdcard_reset(&sd0);

	rtc = ds1202_1302_init("RTC",1302);
	ds1202_1302_reset(rtc);
	atexit(destroy_rtc);

	cpu = cpu_create_z180("Z180",Z180_TYPE_Z180,18432000,&ram,NULL,&iospace,irq0ackcallback,NULL/*daisychain*/,
		asci_rx,asci_tx,sci_read,sci_write,NULL,NULL,NULL,NULL);
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
