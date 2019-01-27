/*
 * 86Box	A hypervisor and IBM PC system emulator that specializes in
 *		running old operating systems and software designed for IBM
 *		PC systems and compatibles from 1981 through fairly recent
 *		system designs based on the PCI bus.
 *
 *		This file is part of the 86Box distribution.
 *
 *		Definitions for the Super I/O chips.
 *
 * Version:	@(#)sio.h	1.0.3	2018/09/15
 *
 * Author:	Fred N. van Kempen, <decwiz@yahoo.com>
 *		Copyright 2017 Fred N. van Kempen.
 */

/* Revision: 2019-01-26 Michal Tomek z180emu */

#ifndef EMU_SIO_H
# define EMU_SIO_H

#define SERIAL1_ADDR		0x03f8
#define SERIAL1_IRQ		4
#define SERIAL2_ADDR		0x02f8
#define SERIAL2_IRQ		3

typedef struct fdc37c66x {
	fdc_t *fdc;
	struct ins8250_device *serial1;
	struct ins8250_device *serial2;
} fdc37c66x_t;

uint8_t fdc37c66x_read(uint16_t port, void *priv);
void fdc37c66x_write(uint16_t port, uint8_t val, void *priv);

extern void	superio_detect_init(void);
extern void	fdc37c663_init(void);
//extern void	fdc37c665_init(void);
fdc37c66x_t *fdc37c665_init(devcb_write_line fdc_int_state_cb, devcb_write_line fdc_dma_req_cb,
	devcb_write_line serial1_int_state_cb, rx_callback_t serial1_rx_cb, tx_callback_t serial1_tx_cb,
	devcb_write_line serial2_int_state_cb, rx_callback_t serial2_rx_cb, tx_callback_t serial2_tx_cb);
extern void	fdc37c669_init(void);
extern void	fdc37c932fr_init(void);
extern void	fdc37c935_init(void);
extern void	pc87306_init(void);
extern void	um8669f_init(void);
extern void	w83877f_init(uint8_t reg16init);


#endif	/*EMU_SIO_H*/
