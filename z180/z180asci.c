// license:BSD-3-Clause
// copyright-holders:Joakim Larsson Edstrom

// Copyright (c) Michal Tomek 2018-2019
// based on Z80SCC

/***************************************************************************

    Z180-ASCI DUART

****************************************************************************/

#include <stdlib.h>
#include <string.h>
#include <ctype.h>
#include "z180.h"

#define STAT_TIE 0x1
#define STAT_TDRE 0x2
#define STAT_DCD0 0x4
#define STAT_CTS1E 0x4
#define STAT_RIE 0x8
#define STAT_FE 0x10
#define STAT_PE 0x20
#define STAT_OVRN 0x40
#define STAT_RDRF 0x80

#define CNTLA_MOD0 0x1
#define CNTLA_MOD1 0x2
#define CNTLA_MOD2 0x4
#define CNTLA_MPBR 0x8
#define CNTLA_RTS0 0x10
#define CNTLA_CKA1D 0x10
#define CNTLA_TE 0x20
#define CNTLA_RE 0x40
#define CNTLA_MPE 0x80

#define CNTLB_SS0 0x1
#define CNTLB_SS1 0x2
#define CNTLB_SS2 0x4
#define CNTLB_SS 0x7
#define CNTLB_DR 0x8
#define CNTLB_PEO 0x10
#define CNTLB_CTS 0x20
#define CNTLB_PS 0x20
#define CNTLB_MP 0x40
#define CNTLB_MPBT 0x80

#define ASEXT_SB 0x1
#define ASEXT_BD 0x2
#define ASEXT_BFE 0x4
#define ASEXT_BRGM 0x8
#define ASEXT_X1BCLK 0x10
#define ASEXT_CTS0D	0x20
#define ASEXT_DCD0D	0x40
#define ASEXT_RDRFI	0x80


#ifdef _MSC_VER
#define FUNCNAME __func__
#else
#define FUNCNAME __PRETTY_FUNCTION__
#endif

#define CHAN0_TAG   "ch0"
#define CHAN1_TAG   "ch1"

//**************************************************************************
//  FORWARD DECLARATIONS
//**************************************************************************
#define receive_register_reset(ch) ch->rx_bits_rem = 0
#define transmit_register_reset(ch) ch->tx_bits_rem = 0
#define set_tra_rate(ch,x) /**/
#define set_rcv_rate(ch,x) /**/
#define is_transmit_register_empty(ch) (ch->tx_bits_rem == 0)
#define transmit_register_setup(ch,data) ch->tx_data = data; ch->tx_bits_rem = ch->m_bit_count	/* load character into shift register */
#define transmit_register_get_data_bit(ch) 0
#define receive_register_extract(ch) /**/
#define get_received_char(ch) ch->rx_data 

void z180asci_channel_device_start(struct z180asci_channel *ch);
void z180asci_channel_device_reset(struct z180asci_channel *ch);
void z180asci_channel_tra_callback(struct z180asci_channel *ch);
void z180asci_channel_rcv_callback(struct z180asci_channel *ch);
void z180asci_channel_tra_complete(struct z180asci_channel *ch);
void z180asci_channel_rcv_complete(struct z180asci_channel *ch);
void z180asci_channel_set_rts(struct z180asci_channel *ch, int state);
void z180asci_channel_m_rx_fifo_rp_step(struct z180asci_channel *ch);
void z180asci_channel_receive_data(struct z180asci_channel *ch, uint8_t data);
uint8_t z180asci_channel_data_read(struct z180asci_channel *ch);
void z180asci_channel_data_write(struct z180asci_channel *ch, uint8_t data);
void z180asci_channel_update_serial(struct z180asci_channel *ch);


struct z180asci_device *z180asci_device_create(void *owner, char *tag, /*UINT32 type,*/ UINT32 clock,
	rx_callback_t rx_callback,tx_callback_t tx_callback) {

	struct z180asci_device *d = malloc(sizeof(struct z180asci_device));
	memset(d,0,sizeof(struct z180asci_device));
	d->m_owner = owner;
	//d->m_type = type;
	d->m_clock = clock;
	d->m_tag = tag;

	d->m_chan0 = malloc(sizeof(struct z180asci_channel));
	memset(d->m_chan0,0,sizeof(struct z180asci_channel));
	d->m_chan1 = malloc(sizeof(struct z180asci_channel));
	memset(d->m_chan1,0,sizeof(struct z180asci_channel));

	//d->m_out_int_cb = out_int_cb;
	d->tx_callback = tx_callback;
	d->rx_callback = rx_callback;

	d->m_chan0->m_tag = CHAN0_TAG;
	d->m_chan0->m_index = 0;
	d->m_chan0->m_uart = d;
	d->m_chan1->m_tag = CHAN1_TAG;
	d->m_chan1->m_index = 1;
	d->m_chan1->m_uart = d;
	z180asci_channel_device_start(d->m_chan0);
	z180asci_channel_device_start(d->m_chan1);
	z180asci_device_reset(d);

	return d;
}


void z180asci_device_reset(struct z180asci_device *device)
{
	z180asci_channel_device_reset(device->m_chan0);
	z180asci_channel_device_reset(device->m_chan1);

	// reset common registers

	// reset external lines
}

/*
 * Interrupts
*/

//-------------------------------------------------
//  check_interrupts -
//-------------------------------------------------
void z180asci_channel_check_interrupts(struct z180asci_channel *ch)
{
	LOG("%s Channel %d %s \n",ch->m_uart->m_tag, ch->m_index, FUNCNAME);

	if (cpu_get_state_z180(ch->m_uart->m_owner, Z180_IFF1)) {
		z180_set_asci_irq(ch->m_uart->m_owner, ch->m_index, 
			(ch->m_stat & (STAT_TDRE|STAT_TIE)) == (STAT_TDRE|STAT_TIE)
			||((ch->m_stat & STAT_RIE) && ((ch->m_index == 0 && !ch->m_dcd)
			   ||(ch->m_stat & (STAT_RDRF|STAT_OVRN|STAT_PE|STAT_FE))))
		);
	}
	//device->m_out_int_cb(device, state); // we don't have an external interrupt line to assert
}


//-------------------------------------------------
//  start - channel startup
//-------------------------------------------------
void z180asci_channel_device_start(struct z180asci_channel *ch)
{
	LOG("%s\n", FUNCNAME);
	ch->m_cka = 0;

		ch->m_brg_rate = 0;
		ch->m_brg_const = 2;

}

//-------------------------------------------------
//  reset - reset channel status
//-------------------------------------------------
void z180asci_channel_device_reset(struct z180asci_channel *ch)
{
	LOG("%s\n", FUNCNAME);

	// reset registers
	ch->m_stat = 2; // TDRE=1
	ch->m_cntla = ch->m_index ? 0:0x10;
	ch->m_cntlb = 7;
	ch->m_asext = 0;
	ch->m_astc = 0;

	// stop receiver and transmitter
	ch->tx_bits_rem = 0;
	ch->rx_bits_rem = 0;

	// reset external lines
	ch->m_cts = 0;
	if (ch->m_index==0)
	{
		ch->m_rts = 0;
		ch->m_dcd = 0;
		//out_rts_cb(!(ch->m_rts = ch->m_cntla & CNTLA_RTS0 ? 1:0));
		//out_dcd_cb(m_dcd);
	}

	// reset interrupts
}

void z180asci_channel_device_timer(struct z180asci_channel *ch /*, emu_timer *timer, device_timer_id id, int param, void *ptr*/)
{
	//LOG("m_brg_timer %s:%d\n",ch->m_tag,ch->m_brg_timer);
	if (!--ch->m_brg_timer) {
		z180asci_channel_rcv_callback(ch);
		z180asci_channel_tra_callback(ch);
		ch->m_brg_timer = ch->m_brg_const;
	}
}


//-------------------------------------------------
//  tra_callback -
//-------------------------------------------------
void z180asci_channel_tra_callback(struct z180asci_channel *ch)
{
	if (!(ch->m_cntla & CNTLA_TE))
	{
		LOG("%s \"%s \"Channel %d transmit mark 1\n", FUNCNAME, ch->m_uart->m_tag, ch->m_index);
		// transmit mark
		//out_txd_cb(1);
	}
	else if ((ch->m_asext & (ASEXT_BD|ASEXT_SB)) == (ASEXT_BD|ASEXT_SB))
	{
		LOG("%s \"%s \"Channel %d send break 0\n", FUNCNAME, ch->m_uart->m_tag, ch->m_index);
		// transmit break
		//out_txd_cb(0);
	}
	else if (!is_transmit_register_empty(ch))
	{
		int db = transmit_register_get_data_bit(ch);

		LOG("%s \"%s \"Channel %d transmit data bit\n", FUNCNAME, ch->m_uart->m_tag, ch->m_index/*, db*/);
		// transmit data
		//out_txd_cb(db);
		if (!--ch->tx_bits_rem)
			z180asci_channel_tra_complete(ch);
	}
	/*else
	{
		LOG("%s \"%s \"Channel %d Failed to transmit\n", FUNCNAME, ch->m_uart->m_tag, ch->m_index);
	}*/
}

//-------------------------------------------------
//  tra_complete -
//-------------------------------------------------
void z180asci_channel_tra_complete(struct z180asci_channel *ch)
{
	if (ch->m_uart->tx_callback)
		ch->m_uart->tx_callback(ch->m_uart,ch->m_index,ch->tx_data);

	if ((ch->m_cntla & CNTLA_TE) && (ch->m_asext & (ASEXT_BD|ASEXT_SB)) != (ASEXT_BD|ASEXT_SB))
	{
		if ( !(ch->m_stat & STAT_TDRE) )
		{
			LOG("%s %s %d done sending, loading data from TDR:%02x '%c'\n", FUNCNAME, ch->m_uart->m_tag, ch->m_index,
				   ch->m_tdr, isascii(ch->m_tdr) ? ch->m_tdr : ' ');
			transmit_register_setup(ch,ch->m_tdr); // Reload the shift register
			ch->m_stat |= STAT_TDRE; // Now here is room in the TDR again
		}
		else
		{
			LOG("%s %s %d done sending\n", FUNCNAME, ch->m_uart->m_tag, ch->m_index);

			// when the RTS bit is reset, the _RTS output goes high after the transmitter empties
			if (!ch->m_rts) // TODO: Clean up RTS handling
				z180asci_channel_set_rts(ch,1);
		}

		if (ch->m_stat & STAT_TIE)
		{
			if(ch->m_stat & STAT_TDRE)  // Check TDRE bit and interrupt if slot available
				z180asci_channel_check_interrupts(ch);
		}
	}
	else if ((ch->m_asext & (ASEXT_BD|ASEXT_SB)) == (ASEXT_BD|ASEXT_SB))
	{
		LOG("%s \"%s \"Channel %d Transmit Break 0\n", FUNCNAME, ch->m_uart->m_tag, ch->m_index);
		// transmit break
		//out_txd_cb(0);
	}
	else
	{
		LOG("%s \"%s \"Channel %d Transmit Mark 1\n", FUNCNAME, ch->m_uart->m_tag, ch->m_index);
		// transmit mark
		//out_txd_cb(1);
	}
}


//-------------------------------------------------
//  rcv_callback -
//-------------------------------------------------
void z180asci_channel_rcv_callback(struct z180asci_channel *ch)
{
	int c = -1;
	if (ch->m_cntla & CNTLA_RE)
	{
		if (ch->rx_bits_rem > 0) {
			LOG("%s \"%s \"Channel %d receive data bit\n", FUNCNAME, ch->m_uart->m_tag, ch->m_index);
			//receive_register_update_bit(ch->m_rxd);
			if (!--ch->rx_bits_rem) {
				z180asci_channel_rcv_complete(ch);
			}
		} else {
			if (ch->m_uart->rx_callback)
				c = ch->m_uart->rx_callback(ch->m_uart,ch->m_index);
			if (c!=-1) {
				ch->rx_data = c;
				ch->rx_bits_rem = ch->m_bit_count;
			}
		}
	}
	/*else
	{
		LOG("%s \"%s \"Channel %d Received Data Bit but receiver is disabled\n", FUNCNAME, ch->m_uart->m_tag, ch->m_index);
	}*/
}


//-------------------------------------------------
//  rcv_complete -
//-------------------------------------------------
void z180asci_channel_rcv_complete(struct z180asci_channel *ch)
{
	uint8_t data;

	receive_register_extract(ch);
	data = get_received_char(ch);
	LOG("%s \"%s \"Channel %d Received Data %c\n", FUNCNAME, ch->m_uart->m_tag, ch->m_index, data);
	z180asci_channel_receive_data(ch,data);
}


//-------------------------------------------------
//  get_clock_mode - get clock divisor
//-------------------------------------------------
int z180asci_channel_get_clock_mode(struct z180asci_channel *ch)
{
	if (ch->m_asext & ASEXT_X1BCLK)
		return 1;
	else if (ch->m_cntlb & CNTLB_DR)
		return 64;
	else
		return 16;
}

void z180asci_channel_set_rts(struct z180asci_channel *ch, int state)
{
	LOG("%s(%d) \"%s\": %d \n", FUNCNAME, state, ch->m_uart->m_tag, ch->m_index);
	//out_rts_cb(state);
}

void z180asci_channel_update_rts(struct z180asci_channel *ch)
{
//    LOG("%s(%d) \"%s\": %d \n", FUNCNAME, state, ch->m_uart->m_tag, ch->m_index);
		if (ch->m_index == 1)
		{
			if (ch->m_cntla & CNTLA_RTS0)
			{
					// when the RTS bit is set, the _RTS output goes low
					ch->m_rts = 1;
			}
			else
			{
					// when the RTS bit is reset, the _RTS output goes high after the transmitter empties
					ch->m_rts = 0;
			}
			z180asci_channel_set_rts(ch,!ch->m_rts);
		}
}

const char *z180asci_stop_bits_tostring(enum stop_bits_t stop_bits)
{
	switch (stop_bits)
	{
	case STOP_BITS_1:
		return "1";

	case STOP_BITS_2:
		return "2";

	default:
		return "UNKNOWN";
	}
}

void z180asci_set_data_frame(struct z180asci_channel *ch, int data_bit_count, enum parity_t parity, enum stop_bits_t stop_bits)
{
	//LOG("Start bits: %d; Data bits: %d; Parity: %s; Stop bits: %s\n", start_bit_count, data_bit_count, parity_tostring(parity), z180asci_stop_bits_tostring(stop_bits));

	int stop_bit_count;

	switch (stop_bits)
	{
	case STOP_BITS_1:
	default:
		stop_bit_count = 1;
		break;

	case STOP_BITS_2:
		stop_bit_count = 2;
		break;
	}

	//m_df_parity = parity;
	//m_df_start_bit_count = start_bit_count;

	ch->m_bit_count = 1 + data_bit_count + stop_bit_count + (parity != PARITY_NONE?1:0);

} 

//-------------------------------------------------
//  get_stop_bits - get number of stop bits
//-------------------------------------------------
enum stop_bits_t z180asci_channel_get_stop_bits(struct z180asci_channel *ch)
{
	switch (ch->m_cntla & CNTLA_MOD0)
	{
	default:
	case 0: return STOP_BITS_1;
	case 1: return STOP_BITS_2;
	}
}


//-------------------------------------------------
//  get_rx_word_length - get receive word length
//-------------------------------------------------
int z180asci_channel_get_rx_word_length(struct z180asci_channel *ch)
{
	int bits = 7;

	switch (ch->m_cntla & CNTLA_MOD2)
	{
	case 0:  bits = 7;   break;
	case 4:  bits = 8;   break;
	}

	return bits;
}

//-------------------------------------------------
//  register_read - read an ASCI register
//-------------------------------------------------
uint8_t z180asci_channel_register_read(struct z180asci_channel *ch, uint8_t reg)
{
	//if (reg > 1)
	LOG("%s %02x\n", FUNCNAME, reg);
	uint8_t data = 0;

	switch (reg)
	{
		case Z180_CNTLA0:
		case Z180_CNTLA1:
			data = ch->m_cntla;
			if (ch->m_index==0) {
				data &= ~CNTLA_RTS0;
				data |= !(ch->m_rts&1);
			}
			break;
		case Z180_CNTLB0:
		case Z180_CNTLB1:
			data = ch->m_cntlb;
			data &= ~CNTLB_CTS;
			data |= !(ch->m_cts&1);
			break;
		case Z180_STAT0:
		case Z180_STAT1:
			data = ch->m_stat;
			if (ch->m_index==0) {
				data &= ~STAT_DCD0;
				data |= !(ch->m_dcd&1);
			}
			break;
		case Z180_ASEXT0:
		case Z180_ASEXT1:
			data = ch->m_asext;
			break;
		case Z180_ASTC0L:
		case Z180_ASTC1L:
			data = ch->m_astc & 0xff;
			break;
		case Z180_ASTC0H:
		case Z180_ASTC1H:
			data = ch->m_astc >> 8;
			break;
		case Z180_TDR0:
		case Z180_TDR1:
			data = ch->m_tdr;
			break;
		case Z180_RDR0:
		case Z180_RDR1:
			data = z180asci_channel_data_read(ch);
			break;
	default:
		LOG("%s: %s %d : Unsupported ASCI register:%02x\n", ch->m_uart->m_tag, FUNCNAME, ch->m_index, reg);
	}
	return data;
}

//-------------------------------------------------
//  register_write - write an ASCI register
//-------------------------------------------------
void z180asci_channel_register_write(struct z180asci_channel *ch, uint8_t reg, uint8_t data)
{
	switch (reg)
	{
		case Z180_CNTLA0:
		case Z180_CNTLA1:
			ch->m_cntla = data;
			z180asci_channel_update_serial(ch);
			break;
		case Z180_CNTLB0:
		case Z180_CNTLB1:
			ch->m_cntlb = data;
			z180asci_channel_update_serial(ch);
			break;
		case Z180_STAT0:
 			ch->m_stat &= ~0x9;
			ch->m_stat |= (data & 0x9);
			break;
		case Z180_STAT1:
 			ch->m_stat &= ~0xd;
			ch->m_stat |= (data & 0xd);
			break;
		case Z180_ASEXT0:
		case Z180_ASEXT1:
			ch->m_asext = data;
			z180asci_channel_update_serial(ch);
			break;
		case Z180_ASTC0L:
		case Z180_ASTC1L:
			ch->m_astc &= 0xff00;
			ch->m_astc |= data;
			z180asci_channel_update_serial(ch);
			break;
		case Z180_ASTC0H:
		case Z180_ASTC1H:
			ch->m_astc &= 0xff;
			ch->m_astc |= (data << 8);
			z180asci_channel_update_serial(ch);
			break;
		case Z180_TDR0:
		case Z180_TDR1:
			z180asci_channel_data_write(ch,data);
			break;
		case Z180_RDR0:
		case Z180_RDR1:
			if (!(ch->m_stat & STAT_RDRF))	// loopback as described on p.117 of UM0050
				z180asci_channel_receive_data(ch,data);
			break;
	default:
		LOG("%s: %s %d : Unsupported ASCI register:%02x\n", ch->m_uart->m_tag, FUNCNAME, ch->m_index, reg);
	}
}


//-------------------------------------------------
//  data_read - read data register from fifo
//-------------------------------------------------
uint8_t z180asci_channel_data_read(struct z180asci_channel *ch)
{
	uint8_t data = 0;

	LOG("%s \"%s\": %d : Data Register Read: ", FUNCNAME, ch->m_uart->m_tag, ch->m_index);

	if (ch->m_rx_fifo_wp != ch->m_rx_fifo_rp)
	{

		// load data from the FIFO
		data = ch->m_rx_data_fifo[ch->m_rx_fifo_rp];

		// load error status from the FIFO
		ch->m_stat = (ch->m_stat & ~(STAT_OVRN | STAT_PE | STAT_FE)) | ch->m_rx_error_fifo[ch->m_rx_fifo_rp];

		// trigger interrupt and lock the fifo if an error is present
		if (ch->m_stat & (STAT_OVRN | STAT_PE | STAT_FE))
		{
			LOG("Rx Error %02x\n", ch->m_stat & (STAT_OVRN | STAT_PE | STAT_FE));
			z180asci_channel_check_interrupts(ch);
		}
		else
		{
			// decrease RX FIFO pointer
			z180asci_channel_m_rx_fifo_rp_step(ch);

			// if RX FIFO empty reset RX interrupt status
			if (ch->m_rx_fifo_wp == ch->m_rx_fifo_rp)
			{
				LOG("Rx FIFO empty, resetting status and interrupt state");
				z180asci_channel_check_interrupts(ch);
			}
		}
	}
	else
	{
		LOG("data_read: Attempt to read out character from empty FIFO\n");
	}

	LOG("  '%c' %02x\n", isascii(data) ? data : ' ', data);
	return data;
}

/* Step read pointer */
void z180asci_channel_m_rx_fifo_rp_step(struct z180asci_channel *ch)
{
		ch->m_rx_fifo_rp++;
		if (ch->m_rx_fifo_rp >= M_RX_FIFO_SZ)
		{
				ch->m_rx_fifo_rp = 0;
		}

		// check if RX FIFO is empty
		if (ch->m_rx_fifo_rp == ch->m_rx_fifo_wp)
		{
				// no more characters available in the FIFO
				ch->m_stat &= ~STAT_RDRF;
		}
}

//-------------------------------------------------
//  data_write - write data register
//-------------------------------------------------
void z180asci_channel_data_write(struct z180asci_channel *ch, uint8_t data)
{
	LOG("%s \"%s\": %d : Data Register Write: %02d '%c'\n", FUNCNAME, ch->m_uart->m_tag, ch->m_index, data, isprint(data) ? data : ' ');

	if ( !(ch->m_stat & STAT_TDRE) )
	{
		LOG("- TDR is full, discarding data\n");
	}
	else // ..there is still room
	{
		LOG("- TDR has room, loading data and clearing TDRE bit\n");
		ch->m_tdr = data;
		ch->m_stat &= ~STAT_TDRE;

		// Update interrupt line
		z180asci_channel_check_interrupts(ch);
	}

	/* Transmitter enabled?  */
	if (ch->m_cntla & CNTLA_TE)
	{
		LOG("- TX is enabled\n");
		if (is_transmit_register_empty(ch)) // Is the shift register loaded?
		{
			LOG("- Setting up transmitter\n");
			transmit_register_setup(ch,ch->m_tdr); // Load the shift register, reload is done in tra_complete()
			LOG("- TX shift register loaded\n");
			ch->m_stat |= STAT_TDRE; // And there is a slot in the TDR available
		}
		else
		{
			LOG("- Transmitter not empty\n");
		}
	}
	else
	{
		LOG("- Transmitter disabled\n");
	}

	z180asci_channel_check_interrupts(ch);
}


//-------------------------------------------------
//  receive_data - put received data word into fifo
//-------------------------------------------------
void z180asci_channel_receive_data(struct z180asci_channel *ch, uint8_t data)
{
	LOG("\"%s\": %d : Received Data Byte '%c'/%02x put into FIFO\n", ch->m_uart->m_tag, ch->m_index, isprint(data) ? data : ' ', data);

	if (ch->m_rx_fifo_wp + 1 == ch->m_rx_fifo_rp || ( (ch->m_rx_fifo_wp + 1 == M_RX_FIFO_SZ) && (ch->m_rx_fifo_rp == 0) ))
	{
		// receive overrun error detected
		ch->m_rx_error_fifo[ch->m_rx_fifo_wp] |= STAT_OVRN;

		// store received character but do not step the fifo
		ch->m_rx_data_fifo[ch->m_rx_fifo_wp] = data;

		LOG("Receive_data() Error %02x\n", ch->m_rx_error_fifo[ch->m_rx_fifo_wp] & (STAT_OVRN | STAT_PE | STAT_FE));
	}
	else
	{
		ch->m_rx_error_fifo[ch->m_rx_fifo_wp] &= ~STAT_OVRN;

		// store received character
		ch->m_rx_data_fifo[ch->m_rx_fifo_wp] = data;

		ch->m_rx_fifo_wp++;
		if (ch->m_rx_fifo_wp >= M_RX_FIFO_SZ)
		{
			ch->m_rx_fifo_wp = 0;
		}
	}

	ch->m_stat |= STAT_RDRF;

	z180asci_channel_check_interrupts(ch);
}
 

//-------------------------------------------------
// get_brg_const
//-------------------------------------------------
unsigned int z180asci_channel_get_brg_const(struct z180asci_channel *ch)
{
	if (ch->m_asext & ASEXT_BRGM)
		return 2 + ch->m_astc;
	else
	{
		int divisor = 1<<(ch->m_cntlb & CNTLB_SS);
		return divisor * ((ch->m_cntlb & CNTLB_PS)?30:10);
	}
}

//-------------------------------------------------
// get_brg_rate
//-------------------------------------------------
unsigned int z180asci_channel_get_brg_rate(struct z180asci_channel *ch)
{
	unsigned int rate;

	if ((ch->m_cntlb & CNTLB_SS) != 7) // Do we use the PCLK as baudrate source
	{
		rate = ch->m_uart->m_clock / ch->m_brg_const;
		LOG("   - Source bit rate (%d) = PCLK (%d) / (%d)\n", rate, ch->m_uart->m_clock, ch->m_brg_const);
	}
	else // Else we use the CKA as BRG source
	{
		//unsigned int source = (ch->m_index == 0) ? ch->m_uart->m_rxca : ch->m_uart->m_rxcb;
		//rate = source / ch->m_brg_const;
		rate = 1000000;
		LOG("   - Source bit rate = CKA / (%d)\n", ch->m_brg_const);
	}

	return (rate / (2 * z180asci_channel_get_clock_mode(ch)));
}

//-------------------------------------------------
//  z180asci_channel_update_serial -
//-------------------------------------------------
void z180asci_channel_update_serial(struct z180asci_channel *ch)
{
	int data_bit_count = z180asci_channel_get_rx_word_length(ch);
	enum stop_bits_t stop_bits = z180asci_channel_get_stop_bits(ch);
	enum parity_t parity;

	if (ch->m_cntla & CNTLA_MOD1)
	{
		if (ch->m_cntlb & CNTLB_PEO)
			parity = PARITY_ODD;
		else
			parity = PARITY_EVEN;
	}
	else
	{
		parity = PARITY_NONE;
	}

	LOG("%s \"%s \"Channel %d setting data frame %d+%d%c%d\n", FUNCNAME, ch->m_uart->m_tag, ch->m_index, 1,
		 data_bit_count, parity == PARITY_NONE ? 'N' : parity == PARITY_EVEN ? 'E' : 'O', (stop_bits + 1) / 2);

	z180asci_set_data_frame(ch, data_bit_count, parity, stop_bits);

	int clocks = z180asci_channel_get_clock_mode(ch);

	if ((ch->m_cntlb & CNTLB_SS) != 7)
	{
		LOG("- BRG enabled\n");
		ch->m_brg_const = z180asci_channel_get_brg_const(ch);
		ch->m_brg_rate = z180asci_channel_get_brg_rate(ch);

		LOG("- BRG rate %d\n", ch->m_brg_rate);
		set_rcv_rate(ch,ch->m_brg_rate);

		//if (is_transmit_register_empty(ch))
		//{
			set_tra_rate(ch,ch->m_brg_rate);
			LOG("   - Baud Rate Generator: %d clock mode: %dx\n", ch->m_brg_rate, z180asci_channel_get_clock_mode(ch));
		/*}
		else
		{
			ch->m_delayed_tx_brg_change = 1;
			LOG("   - Baud Rate Generator delay init: %d clock mode: %dx\n", ch->m_brg_rate, z180asci_channel_get_clock_mode(ch));
		}*/
	}
	else
	{
		LOG("- BRG disabled\n");
		set_rcv_rate(ch,0);
		set_tra_rate(ch,0);
	}
	// TODO: Check registers for use of RTxC and TRxC, if used as direct Tx and/or Rx clocks set them to value as programmed
	// in m_uart->txca/txcb and rxca/rxcb respectivelly
	/*if (ch->m_rxc > 0)
	{
		set_rcv_rate(ch,ch->m_rxc / clocks); // TODO Check/Fix this to get the right tx/rx clocks, seems to be missing a divider or two
		LOG("   - Receiver clock: %d mode: %d rate: %d/%xh\n", ch->m_rxc, clocks, ch->m_rxc / clocks, ch->m_rxc / clocks);
	}

	if (ch->m_txc > 0 && (ch->m_cntlb & CNTLB_SS) == 7)
	{
		set_tra_rate(ch, ch->m_txc / clocks);
		LOG("   - Transmit clock: %d mode: %d rate: %d/%xh\n", ch->m_rxc, clocks, ch->m_rxc / clocks, ch->m_rxc / clocks);
	}*/
}
