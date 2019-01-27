/*
 * 86Box	A hypervisor and IBM PC system emulator that specializes in
 *		running old operating systems and software designed for IBM
 *		PC systems and compatibles from 1981 through fairly recent
 *		system designs based on the PCI bus.
 *
 *		This file is part of the 86Box distribution.
 *
 *		Implementation of the SMC FDC37C663 and FDC37C665 Super
 *		I/O Chips.
 *
 * Version:	@(#)sio_fdc37c66x.c	1.0.12	2018/09/15
 *
 * Authors:	Sarah Walker, <http://pcem-emulator.co.uk/>
 *		Miran Grca, <mgrca8@gmail.com>
 *
 *		Copyright 2008-2018 Sarah Walker.
 *		Copyright 2016-2018 Miran Grca.
 */

/* Revision: 2019-01-26 Michal Tomek z180emu */

#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <wchar.h>
#include "86box.h"
//#include "io.h"
//#include "device.h"
//#include "pci.h"
#include "lpt.h"
//#include "serial.h"
#include "../ins8250/ins8250.h"
//#include "disk/hdc.h"
//#include "disk/hdc_ide.h"
#include "fdd.h"
#include "fdc.h"
#include "sio.h"


static uint8_t fdc37c66x_lock[2];
static int fdc37c66x_curreg;
static uint8_t fdc37c66x_regs[16];
static int com3_addr, com4_addr;
static fdc_t *fdc37c66x_fdc;
fdc37c66x_t fdc37c66x;
int serial_base_address[2];

#define sio_log(...) LOG("[sio]" __VA_ARGS__)
#define fatal sio_log

static void write_lock(uint8_t val)
{
        if (val == 0x55 && fdc37c66x_lock[1] == 0x55)
		{
                sio_log("Enter configuration mode\n");
				fdc_3f1_enable(fdc37c66x_fdc, 0);
		}
        if (fdc37c66x_lock[0] == 0x55 && fdc37c66x_lock[1] == 0x55 && val != 0x55)
		{
                sio_log("Exit configuration mode\n");
				fdc_3f1_enable(fdc37c66x_fdc, 1);
		}

        fdc37c66x_lock[0] = fdc37c66x_lock[1];
        fdc37c66x_lock[1] = val;
}

static void ide_handler()
{
	uint16_t or_value = 0;
	//ide_pri_disable();
	if (fdc37c66x_regs[0] & 1)
	{
		sio_log("IDE enabled - unimplemented\n");
		if (fdc37c66x_regs[5] & 2)
		{
			or_value = 0;
		}
		else
		{
			or_value = 0x800;
		}
		//ide_set_base(0, 0x170 | or_value);
		sio_log("ide_set_base 0x%3X\n",0x170 | or_value);
		//ide_set_side(0, 0x376 | or_value);
		sio_log("ide_set_side 0x%3X\n",0x376 | or_value);
		//ide_pri_enable();
	}
	else
		sio_log("IDE disabled\n");
}

static void fdc_handler()
{
	if (fdc37c66x_regs[0] & 0x10)
	{
		sio_log("FDC enabled\n");
	}else {
		sio_log("FDC disabled\n");
	}
}

void serial_setup(int port, uint16_t addr, int irq)
{
	sio_log("Setting serial port %i at %04X\n", port, addr);

	switch(port)
	{
		case 1:
			/*if (!(fdc37c66x_regs[2] & 4))
			{
				printf("set111\n");
			}
			if (serial_base_address[0] != 0x0000)
			{
				serial_remove(port);
			}*/
			if (addr != 0x0000)
			{
				serial_base_address[0] = addr;
				//io_sethandler(addr, 0x0008, serial_read, NULL, NULL, serial_write, NULL, NULL, &serial1);
			}
			//serial1.irq = irq;
			break;
		case 2:
			/*if (!(fdc37c66x_regs[2] & 0x40))
			{
				return;
			}
			if (serial_base_address[1] != 0x0000)
			{
				serial_remove(port);
			}*/
			if (addr != 0x0000)
			{
				serial_base_address[1] = addr;
				//io_sethandler(addr, 0x0008, serial_read, NULL, NULL, serial_write, NULL, NULL, &serial2);
			}
			//serial2.irq = irq;
			break;
		default:
			fatal("serial_setup(): Invalid serial port: %i\n", port);
			break;
	}
}

static void set_com34_addr()
{
	switch (fdc37c66x_regs[1] & 0x60)
	{
		case 0x00:
			com3_addr = 0x338;
			com4_addr = 0x238;
			break;
		case 0x20:
			com3_addr = 0x3e8;
			com4_addr = 0x2e8;
			break;
		case 0x40:
			com3_addr = 0x3e8;
			com4_addr = 0x2e0;
			break;
		case 0x60:
			com3_addr = 0x220;
			com4_addr = 0x228;
			break;
	}
}

static void set_serial1_addr()
{
	//if (fdc37c66x_regs[2] & 4)
	//{
		switch (fdc37c66x_regs[2] & 3)
		{
			case 0:
				serial_setup(1, SERIAL1_ADDR, SERIAL1_IRQ);
				break;

			case 1:
				serial_setup(1, SERIAL2_ADDR, SERIAL2_IRQ);
				break;

			case 2:
				serial_setup(1, com3_addr, 4);
				break;

			case 3:
				serial_setup(1, com4_addr, 3);
				break;
		}
	//}
}

static void set_serial2_addr()
{
	//if (fdc37c66x_regs[2] & 0x40)
	//{
		switch (fdc37c66x_regs[2] & 0x30)
		{
			case 0:
				serial_setup(2, SERIAL1_ADDR, SERIAL1_IRQ);
				break;

			case 1:
				serial_setup(2, SERIAL2_ADDR, SERIAL2_IRQ);
				break;

			case 2:
				serial_setup(2, com3_addr, 4);
				break;

			case 3:
				serial_setup(2, com4_addr, 3);
				break;
		}
	//}
}

static void lpt1_handler()
{
	//lpt1_remove();
	if (fdc37c66x_regs[1] & 4) {
		sio_log("Enabling parallel port\n");
		switch (fdc37c66x_regs[1] & 3)
		{
			case 1:
				//lpt1_init(0x3bc);
				break;
			case 2:
				//lpt1_init(0x378);
				break;
			case 3:
				//lpt1_init(0x278);
				break;
		}
	}
	else
		sio_log("Disabling parallel port\n");
}

void fdc37c66x_write(uint16_t port, uint8_t val, void *priv)
{
	uint8_t valxor = 0;
        if (fdc37c66x_lock[0] == 0x55 && fdc37c66x_lock[1] == 0x55)
        {
                if (port == 0x3f0)
                {
                        if (val == 0xaa)
                                write_lock(val);
                        else {
						sio_log("Select CR%X\n",val);
				fdc37c66x_curreg = val;
				}
#if 0
				if (fdc37c66x_curreg != 0)
				{
	                                fdc37c66x_curreg = val & 0xf;
				}
				else
				{
					/* Hardcode the IDE to AT type. */
	                                fdc37c66x_curreg = (val & 0xf) | 2;
				}
#endif
                }
                else
                {
			if (fdc37c66x_curreg > 15)
				return;
			sio_log("Write to CR%X (%02X)\n",fdc37c66x_curreg,val);

			valxor = val ^ fdc37c66x_regs[fdc37c66x_curreg];
                        fdc37c66x_regs[fdc37c66x_curreg] = val;
                        
			switch(fdc37c66x_curreg)
			{
				case 0:
					if (valxor & 1)
					{
						ide_handler();
					}
					if (valxor & 0x18)
					{
						fdc_handler();
					}
					break;
				case 1:
					if (valxor & 0xf)
					{
						lpt1_handler();
					}
					if (valxor & 0x10)
					{
						if(fdc37c66x_regs[1] & 0x10)
							sio_log("Interrupts active high\n");
						else
						/* NOTE: all callbacks are implemented as active high even when this option is used */
							sio_log("Interrupts active low\n");
					}
					if (valxor & 0x60)
					{
		                                //serial_remove(1);
						set_com34_addr();
						set_serial1_addr();
						set_serial2_addr();
					}
					break;
				case 2:
					if (valxor & 7)
					{
		                                //serial_remove(1);
						if (fdc37c66x_regs[2] & 4)
							sio_log("Enabling serial port 1\n");
						else
							sio_log("Disabling serial port 1\n");
						set_serial1_addr();
					}
					if (valxor & 0x70)
					{
		                                //serial_remove(2);
						if (fdc37c66x_regs[2] & 0x40)
							sio_log("Enabling serial port 2\n");
						else
							sio_log("Disabling serial port 2\n");
						set_serial2_addr();
					}
					break;
				case 3:
					if (valxor & 2)
					{
						fdc_update_enh_mode(fdc37c66x_fdc, (fdc37c66x_regs[3] & 2) ? 1 : 0);
					}
					break;
				case 5:
					if (valxor & 2)
					{
						ide_handler();
					}
					if (valxor & 0x18)
					{
						fdc_update_densel_force(fdc37c66x_fdc, (fdc37c66x_regs[5] & 0x18) >> 3);
					}
					if (valxor & 0x20)
					{
						if (fdc37c66x_regs[5] & 0x20)
							sio_log("Drive 0,1 swap enabled\n");
						else
							sio_log("Drive 0,1 swap disabled\n");
						fdc_set_swap(fdc37c66x_fdc, (fdc37c66x_regs[5] & 0x20) >> 5);
					}
					break;
                        }
                }
        }
        else
        {
                if (port == 0x3f0)
                        write_lock(val);
				else if ((fdc37c66x_regs[0] & 0x10) && port >= fdc37c66x_fdc->base_address && port <= fdc37c66x_fdc->base_address+7)
				{
					fdc_write(port,val,fdc37c66x_fdc);
				}
				else if ((fdc37c66x_regs[2] & 4) && port >= serial_base_address[0] && port <= serial_base_address[0]+7)
				{
					ins8250_device_w(fdc37c66x.serial1,port&7,val);
				}
				else if ((fdc37c66x_regs[2] & 0x40) && port >= serial_base_address[1] && port <= serial_base_address[1]+7)
				{
					ins8250_device_w(fdc37c66x.serial2,port&7,val);
				}

        }
}

uint8_t fdc37c66x_read(uint16_t port, void *priv)
{
        if (fdc37c66x_lock[0] == 0x55 && fdc37c66x_lock[1] == 0x55)
        {
                if (port == 0x3f1) {
					sio_log("Read from CR%X (%02X)\n",fdc37c66x_curreg,fdc37c66x_regs[fdc37c66x_curreg]);
                        return fdc37c66x_regs[fdc37c66x_curreg];
				}
        }
		else if ((fdc37c66x_regs[0] & 0x10) && port >= fdc37c66x_fdc->base_address && port <= fdc37c66x_fdc->base_address+7)
		{
        	return fdc_read(port,fdc37c66x_fdc);
		}
		else if ((fdc37c66x_regs[2] & 4) && port >= serial_base_address[0] && port <= serial_base_address[0]+7)
		{
        	return ins8250_device_r(fdc37c66x.serial1,port&7);
		}
		else if ((fdc37c66x_regs[2] & 0x40) && port >= serial_base_address[1] && port <= serial_base_address[1]+7)
		{
        	return ins8250_device_r(fdc37c66x.serial2,port&7);
		}
		else
			return 0xff;
}

static void fdc37c66x_reset(void)
{
	memset(fdc37c66x_lock, 0, 2);
	memset(fdc37c66x_regs, 0, 16);
        fdc37c66x_regs[0x0] = 0x3b;
        fdc37c66x_regs[0x1] = 0x9f;
        fdc37c66x_regs[0x2] = 0xdc;
        fdc37c66x_regs[0x3] = 0x78;
        fdc37c66x_regs[0x6] = 0xff;
        fdc37c66x_regs[0xe] = 0x02;

	fdc_reset(fdc37c66x_fdc);

	com3_addr = 0x338;
	com4_addr = 0x238;

	//serial_remove(1);
	serial_setup(1, SERIAL1_ADDR, SERIAL1_IRQ);

	//serial_remove(2);
	serial_setup(2, SERIAL2_ADDR, SERIAL2_IRQ);

	//lpt2_remove();

	//lpt1_remove();
	//lpt1_init(0x378);
}

/*static void fdc37c663_reset(void)
{
	fdc37c66x_reset();
        fdc37c66x_regs[0xd] = 0x63;
}*/

static void fdc37c665_reset(void)
{
	fdc37c66x_reset();
        fdc37c66x_regs[0xd] = 0x65;
}

/*void fdc37c663_init(void)
{
	fdc37c66x_fdc = device_add(&fdc_at_smc_device);

        io_sethandler(0x03f0, 0x0002, fdc37c66x_read, NULL, NULL, fdc37c66x_write, NULL, NULL,  NULL);

	fdc37c663_reset();
}*/

fdc37c66x_t *fdc37c665_init(devcb_write_line fdc_int_state_cb, devcb_write_line fdc_dma_req_cb,
	devcb_write_line serial1_int_state_cb, rx_callback_t serial1_rx_cb, tx_callback_t serial1_tx_cb,
	devcb_write_line serial2_int_state_cb, rx_callback_t serial2_rx_cb, tx_callback_t serial2_tx_cb)
{
	fdc37c66x_fdc = fdc_init(FDC_FLAG_AT | FDC_FLAG_SUPERIO, fdc_int_state_cb, fdc_dma_req_cb); /*device_add(&fdc_at_smc_device);*/
	fdc37c66x.fdc = fdc37c66x_fdc;

	fdc37c66x.serial1 = ins8250_device_create("SER1", &fdc37c66x, 1846153, NS16550A, serial1_int_state_cb, serial1_rx_cb, serial1_tx_cb);
	fdc37c66x.serial2 = ins8250_device_create("SER2", &fdc37c66x, 1846153, NS16550A, serial2_int_state_cb, serial2_rx_cb, serial2_tx_cb);
	ins8250_device_reset(fdc37c66x.serial1);
	ins8250_device_reset(fdc37c66x.serial2);

        //io_sethandler(0x03f0, 0x0002, fdc37c66x_read, NULL, NULL, fdc37c66x_write, NULL, NULL,  NULL);

	fdc37c665_reset();

	return &fdc37c66x;
}
