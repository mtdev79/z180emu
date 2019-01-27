// license:BSD-3-Clause
// copyright-holders:Joakim Larsson Edstrom 

// Copyright (c) Michal Tomek 2018-2019
// based on Z80SCC 

/***************************************************************************

    Z180-ASCI DUART

****************************************************************************/

#ifndef __Z180ASCI_H
#define __Z180ASCI_H

#pragma once

#define M_RX_FIFO_SZ 4       // receive FIFO size 

struct z180asci_channel {
	char *m_tag;

	int m_cka;

	// Register state
	uint8_t m_cntla;
	uint8_t m_cntlb;
	uint8_t m_stat;
	uint8_t m_asext;
	uint16_t m_astc;

	unsigned int m_brg_const;
	uint16_t m_brg_timer;
	unsigned int m_brg_rate;

	// receiver state
	uint8_t m_rx_data_fifo[M_RX_FIFO_SZ];    // receive data FIFO 
	uint8_t m_rx_error_fifo[M_RX_FIFO_SZ];   // receive error FIFO
	int m_rx_fifo_rp;       // receive FIFO read pointer
	int m_rx_fifo_wp;       // receive FIFO write pointer

	//int m_rx_clock;         // receive clock pulse count
	UINT8 rx_bits_rem;
	UINT8 rx_data;			// RSR

	//int m_rxd;

	// transmitter state
	uint8_t m_tdr;

	//int m_tx_clock;             // transmit clock pulse count
	UINT8 tx_bits_rem;
	UINT8 tx_data;			// TSR

	UINT8 m_bit_count;

	int m_cts;      // clear to send
	int m_dcd;		// data carrier detect (Ch0 only)
	int m_rts;      // request to send (Ch0 only)

	int m_index;	// channel index
	struct z180asci_device *m_uart;

};

// ======================> z180asci_device


typedef void (*tx_callback_t)(device_t *device, int channel, UINT8 data);
typedef int (*rx_callback_t)(device_t *device, int channel);

struct z180asci_device {
	char *m_tag;
	//UINT32 m_type;
	UINT32 m_clock;
	void *m_owner;

	struct z180asci_channel *m_chan0;
	struct z180asci_channel *m_chan1;

	// internal state

	// byte rx/tx callbacks
	// Note: bit rx/tx callbacks are not implemented
	tx_callback_t tx_callback;
	rx_callback_t rx_callback;
    
	// interrupt line callback
	//devcb_write_line    m_out_int_cb;

};

struct z180asci_device *z180asci_device_create(void *owner, char *tag, /*UINT32 type,*/ UINT32 clock,
	rx_callback_t rx_callback,tx_callback_t tx_callback);
void z180asci_device_reset(struct z180asci_device *device);
void z180asci_channel_device_timer(struct z180asci_channel *ch /*, emu_timer *timer, device_timer_id id, int param, void *ptr*/);
uint8_t z180asci_channel_register_read(struct z180asci_channel *ch, uint8_t reg);
void z180asci_channel_register_write(struct z180asci_channel *ch, uint8_t reg, uint8_t data);

#endif // __Z180ASCI_H
