/*****************************************************************************
 *
 *   z180.c
 *   Portable Z180 emulator V0.3
 *
 *   Copyright Juergen Buchmueller, all rights reserved.
 *   You can contact me at juergen@mame.net or pullmoll@stop1984.com
 *
 *     This program is free software; you can redistribute it and/or
 *     modify it under the terms of the GNU General Public License
 *     as published by the Free Software Foundation; either version 2
 *     of the License, or (at your option) any later version.
 *
 *     This program is distributed in the hope that it will be useful,
 *     but WITHOUT ANY WARRANTY; without even the implied warranty of
 *     MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *     GNU General Public License for more details.
 *
 *     You should have received a copy of the GNU General Public License
 *     along with this program; if not, write to the Free Software
 *     Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.
 *
 *
 *****************************************************************************/

int VERBOSE;

/* Revision: 2019-01-26 Michal Tomek z180emu */

/*****************************************************************************

    TODO:
        - HALT processing is not yet perfect. The manual states that
          during HALT, all dma and internal i/o incl. timers continue to
          work. Currently, only timers are implemented. Ideally, the
          burn_cycles routine would go away and halt processing be
          implemented in cpu_execute.
 *****************************************************************************/

/*****************************************************************************

Z180 Info:

Known clock speeds (from ZiLOG): 6, 8, 10, 20 & 33MHz

ZiLOG Z180 codes:

  Speed: 10 = 10MHZ
         20 = 20MHz
         33 = 33MHz
Package: P = 60-Pin Plastic DIP
         V = 68-Pin PLCC
         F = 80-Pin QFP
   Temp: S = 0C to +70C
         E = -40C to +85C

Environmanetal Flow: C = Plastic Standard


Example from Ms.Pac-Man/Galaga - 20 year Reunion hardare (see src/mame/drivers/20pacgal.c):

   CPU is Z8S18020VSC = Z180, 20MHz, 68-Pin PLCC, 0C to +70C, Plastic Standard


Other CPUs that use a compatible Z180 core:

Hitachi HD647180 series:
  Available in QFP80, PLCC84 & DIP90 packages (the QFP80 is not pinout compatible)
  The HD647180 also has an internal ROM

 *****************************************************************************/

#include <stdlib.h>
#include <string.h>

//#include "emu.h"
//#include "debugger.h"
#include "z180.h"

/* interrupt priorities */
#define Z180_INT_TRAP   0           /* Undefined opcode */
#define Z180_INT_NMI    1           /* NMI */
#define Z180_INT_IRQ0   2           /* Execute IRQ0 */
#define Z180_INT_IRQ1   3           /* Execute IRQ1 */
#define Z180_INT_IRQ2   4           /* Execute IRQ2 */
#define Z180_INT_PRT0   5           /* Internal PRT channel 0 */
#define Z180_INT_PRT1   6           /* Internal PRT channel 1 */
#define Z180_INT_DMA0   7           /* Internal DMA channel 0 */
#define Z180_INT_DMA1   8           /* Internal DMA channel 1 */
#define Z180_INT_CSIO   9           /* Internal CSI/O */
#define Z180_INT_ASCI0  10          /* Internal ASCI channel 0 */
#define Z180_INT_ASCI1  11          /* Internal ASCI channel 1 */
#define Z180_INT_MAX    Z180_INT_ASCI1

/****************************************************************************/
/* The Z180 registers. HALT is set to 1 when the CPU is halted, the refresh */
/* register is calculated as follows: refresh=(Regs.R&127)|(Regs.R2&128)    */
/****************************************************************************/

struct memory_select;

struct z180_state
{
	union PAIR    PREPC,PC,SP,AF,BC,DE,HL,IX,IY;
	union PAIR    AF2,BC2,DE2,HL2;
	UINT8   R,R2,IFF1,IFF2,HALT,IM,I;
	UINT8   tmdr_latch;                     /* flag latched TMDR0H, TMDR1H values */
	UINT8   read_tcr_tmdr[2];               /* flag to indicate that TCR or TMDR was read */
	UINT32  iol;                            /* I/O line status bits */
	UINT8   io[64];                         /* 64 internal 8 bit registers */
	UINT8   ioadd[64];                      /* additional 64 Z181/Z182 8 bit registers */
	offs_t  mmu[16];                        /* MMU address translation cache */
	UINT8   tmdrh[2];                       /* latched TMDR0H and TMDR1H values */
	UINT16  tmdr_value[2];                  /* TMDR values used byt PRT0 and PRT1 as down counter */
	UINT8   tif[2];                         /* TIF0 and TIF1 values */
	UINT8   nmi_state;                      /* nmi line state */
	UINT8   nmi_pending;                    /* nmi pending */
	UINT8   irq_state[3];                   /* irq line states (INT0,INT1,INT2) */
	UINT8   int_pending[Z180_INT_MAX + 1];  /* interrupt pending */
	UINT8   after_EI;                       /* are we in the EI shadow? */
	UINT32  ea;
	UINT8   timer_cnt;                      /* timer counter / divide by 20 */
	UINT8   dma0_cnt;                       /* dma0 counter / divide by 20 */
	UINT8   dma1_cnt;                       /* dma1 counter / divide by 20 */
	struct z80_daisy_chain *daisy;	/* daisy chain */
	device_irq_acknowledge_callback irq_callback;
	struct z180_device *device;
	struct memory_select *memory;	/* on Z180 mapped to ram, on Z182 mapped according to ROMBR,RAMLBR */
	struct address_space *ram;		/* on all processor types */
	struct address_space *rom;		/* only on Z182 */
	struct address_space *iospace;  /* on all processor types */
	//UINT8   rtemp;
	//UINT32  ioltemp;
	int icount;
	int extra_cycles;           /* extra cpu cycles */
	UINT8 *cc[6];	/* cycle count tables */
};

struct memory_select {
	UINT8       (*read_byte)(struct z180_state *cpustate, offs_t byteaddress);
	void        (*write_byte)(struct z180_state *cpustate, offs_t byteaddress, UINT8 data);
	UINT8 (*read_raw_byte)(struct z180_state *cpustate, offs_t byteaddress/*, offs_t directxor = 0*/);
};


INLINE struct z180_state *get_safe_token(device_t *device)
{
	assert(device != NULL);
	//assert(device->type() == Z180);
	return (struct z180_state *)((struct z180_device *)device)->m_token;
}

#define CF  0x01
#define NF  0x02
#define PF  0x04
#define VF  PF
#define XF  0x08
#define HF  0x10
#define YF  0x20
#define ZF  0x40
#define SF  0x80

/* I/O line status flags */
#define Z180_CKA0     0x00000001  /* I/O asynchronous clock 0 (active high) or DREQ0 (mux) */
#define Z180_CKA1     0x00000002  /* I/O asynchronous clock 1 (active high) or TEND1 (mux) */
#define Z180_CKS      0x00000004  /* I/O serial clock (active high) */
#define Z180_CTS0     0x00000100  /* I   clear to send 0 (active low) */
#define Z180_CTS1     0x00000200  /* I   clear to send 1 (active low) or RXS (mux) */
#define Z180_DCD0     0x00000400  /* I   data carrier detect (active low) */
#define Z180_DREQ0    0x00000800  /* I   data request DMA ch 0 (active low) or CKA0 (mux) */
#define Z180_DREQ1    0x00001000  /* I   data request DMA ch 1 (active low) */
#define Z180_RXA0     0x00002000  /* I   asynchronous receive data 0 (active high) */
#define Z180_RXA1     0x00004000  /* I   asynchronous receive data 1 (active high) */
#define Z180_RXS      0x00008000  /* I   clocked serial receive data (active high) or CTS1 (mux) */
#define Z180_RTS0     0x00010000  /*   O request to send (active low) */
#define Z180_TEND0    0x00020000  /*   O transfer end 0 (active low) or CKA1 (mux) */
#define Z180_TEND1    0x00040000  /*   O transfer end 1 (active low) */
#define Z180_A18_TOUT 0x00080000  /*   O transfer out (PRT channel, active low) or A18 (mux) */
#define Z180_TXA0     0x00100000  /*   O asynchronous transmit data 0 (active high) */
#define Z180_TXA1     0x00200000  /*   O asynchronous transmit data 1 (active high) */
#define Z180_TXS      0x00400000  /*   O clocked serial transmit data (active high) */

/*
 * Prevent warnings on NetBSD.  All identifiers beginning with an underscore
 * followed by an uppercase letter are reserved by the C standard (ISO/IEC
 * 9899:1999, 7.1.3) to be used by the implementation.  It'd be best to rename
 * all such instances, but this is less intrusive and error-prone.
 */
#undef _B
#undef _C
#undef _L

#define _PPC    PREPC.d /* previous program counter */

#define _PCD    PC.d
#define _PC     PC.w.l

#define _SPD    SP.d
#define _SP     SP.w.l

#define _AFD    AF.d
#define _AF     AF.w.l
#define _A      AF.b.h
#define _F      AF.b.l

#define _BCD    BC.d
#define _BC     BC.w.l
#define _B      BC.b.h
#define _C      BC.b.l

#define _DED    DE.d
#define _DE     DE.w.l
#define _D      DE.b.h
#define _E      DE.b.l

#define _HLD    HL.d
#define _HL     HL.w.l
#define _H      HL.b.h
#define _L      HL.b.l

#define _IXD    IX.d
#define _IX     IX.w.l
#define _HX     IX.b.h
#define _LX     IX.b.l

#define _IYD    IY.d
#define _IY     IY.w.l
#define _HY     IY.b.h
#define _LY     IY.b.l

#define IO(n)       io[n]
/*#define IO_CNTLA0   IO(Z180_CNTLA0)
#8define IO_CNTLA1   IO(Z180_CNTLA1)
#define IO_CNTLB0   IO(Z180_CNTLB0)
#define IO_CNTLB1   IO(Z180_CNTLB1)
#define IO_STAT0    IO(Z180_STAT0)
#define IO_STAT1    IO(Z180_STAT1)
#define IO_TDR0     IO(Z180_TDR0)
#define IO_TDR1     IO(Z180_TDR1)
#define IO_RDR0     IO(Z180_RDR0)
#define IO_RDR1     IO(Z180_RDR1)*/
#define IO_CNTR     IO(Z180_CNTR)
#define IO_TRDR     IO(Z180_TRDR)
#define IO_TMDR0L   IO(Z180_TMDR0L)
#define IO_TMDR0H   IO(Z180_TMDR0H)
#define IO_RLDR0L   IO(Z180_RLDR0L)
#define IO_RLDR0H   IO(Z180_RLDR0H)
#define IO_TCR      IO(Z180_TCR)
#define IO_IO11     IO(Z180_IO11)
//#define IO_ASEXT0   IO(Z180_ASEXT0)
//#define IO_ASEXT1   IO(Z180_ASEXT1)
#define IO_TMDR1L   IO(Z180_TMDR1L)
#define IO_TMDR1H   IO(Z180_TMDR1H)
#define IO_RLDR1L   IO(Z180_RLDR1L)
#define IO_RLDR1H   IO(Z180_RLDR1H)
#define IO_FRC      IO(Z180_FRC)
#define IO_IO19     IO(Z180_IO19)
/*#define IO_ASTC0L   IO(Z180_ASTC0L)
#define IO_ASTC0H   IO(Z180_ASTC0H)
#define IO_ASTC1L   IO(Z180_ASTC1L)
#define IO_ASTC1H   IO(Z180_ASTC1H)*/
#define IO_CMR      IO(Z180_CMR)
#define IO_CCR      IO(Z180_CCR)
#define IO_SAR0L    IO(Z180_SAR0L)
#define IO_SAR0H    IO(Z180_SAR0H)
#define IO_SAR0B    IO(Z180_SAR0B)
#define IO_DAR0L    IO(Z180_DAR0L)
#define IO_DAR0H    IO(Z180_DAR0H)
#define IO_DAR0B    IO(Z180_DAR0B)
#define IO_BCR0L    IO(Z180_BCR0L)
#define IO_BCR0H    IO(Z180_BCR0H)
#define IO_MAR1L    IO(Z180_MAR1L)
#define IO_MAR1H    IO(Z180_MAR1H)
#define IO_MAR1B    IO(Z180_MAR1B)
#define IO_IAR1L    IO(Z180_IAR1L)
#define IO_IAR1H    IO(Z180_IAR1H)
#define IO_IAR1B    IO(Z180_IAR1B)
#define IO_BCR1L    IO(Z180_BCR1L)
#define IO_BCR1H    IO(Z180_BCR1H)
#define IO_DSTAT    IO(Z180_DSTAT)
#define IO_DMODE    IO(Z180_DMODE)
#define IO_DCNTL    IO(Z180_DCNTL)
#define IO_IL       IO(Z180_IL)
#define IO_ITC      IO(Z180_ITC)
#define IO_IO35     IO(Z180_IO35)
#define IO_RCR      IO(Z180_RCR)
#define IO_IO37     IO(Z180_IO37)
#define IO_CBR      IO(Z180_CBR)
#define IO_BBR      IO(Z180_BBR)
#define IO_CBAR     IO(Z180_CBAR)
#define IO_IO3B     IO(Z180_IO3B)
#define IO_IO3C     IO(Z180_IO3C)
#define IO_IO3D     IO(Z180_IO3D)
#define IO_OMCR     IO(Z180_OMCR)
#define IO_IOCR     IO(Z180_IOCR)

#define IOADD(n)	ioadd[n-Z182_REGSTART]
//#define IO_SCCACNT	IOADD(Z182_SCCACNT)
//#define IO_SCCAD	IOADD(Z182_SCCAD)
//#define IO_SCCBCNT	IOADD(Z182_SCCBCNT)
//#define IO_SCCBD	IOADD(Z182_SCCBD)
#define IO_SCR		IOADD(Z182_SCR)
#define IO_RAMUBR	IOADD(Z182_RAMUBR)
#define IO_RAMLBR	IOADD(Z182_RAMLBR)
#define IO_ROMBR	IOADD(Z182_ROMBR)
#define IO_WSGCSR	IOADD(Z182_WSGCSR)
#define IO_IEPMUX	IOADD(Z182_IEPMUX)
#define IO_ENHR		IOADD(Z182_ENHR)
#define IO_DDRA		IOADD(Z182_DDRA)
#define IO_DRA		IOADD(Z182_DRA)
#define IO_DDRB		IOADD(Z182_DDRB)
#define IO_DRB		IOADD(Z182_DRB)
#define IO_DDRC		IOADD(Z182_DDRC)
#define IO_DRC		IOADD(Z182_DRC)

/* 00 ASCI control register A ch 0 */
/*#define Z180_CNTLA0_MPE         0x80
#define Z180_CNTLA0_RE          0x40
#define Z180_CNTLA0_TE          0x20
#define Z180_CNTLA0_RTS0        0x10
#define Z180_CNTLA0_MPBR_EFR    0x08
#define Z180_CNTLA0_MODE_DATA   0x04
#define Z180_CNTLA0_MODE_PARITY 0x02
#define Z180_CNTLA0_MODE_STOPB  0x01

#define Z180_CNTLA0_RESET       0x10
#define Z180_CNTLA0_RMASK       0xff
#define Z180_CNTLA0_WMASK       0xff*/

/* 01 ASCI control register A ch 1 */
/*#define Z180_CNTLA1_MPE         0x80
#define Z180_CNTLA1_RE          0x40
#define Z180_CNTLA1_TE          0x20
#define Z180_CNTLA1_CKA1D       0x10
#define Z180_CNTLA1_MPBR_EFR    0x08
#define Z180_CNTLA1_MODE        0x07

#define Z180_CNTLA1_RESET       0x10
#define Z180_CNTLA1_RMASK       0xff
#define Z180_CNTLA1_WMASK       0xff*/

/* 02 ASCI control register B ch 0 */
/*#define Z180_CNTLB0_MPBT        0x80
#define Z180_CNTLB0_MP          0x40
#define Z180_CNTLB0_CTS_PS      0x20
#define Z180_CNTLB0_PEO         0x10
#define Z180_CNTLB0_DR          0x08
#define Z180_CNTLB0_SS          0x07

#define Z180_CNTLB0_RESET       0x07
#define Z180_CNTLB0_RMASK       0xff
#define Z180_CNTLB0_WMASK       0xff*/

/* 03 ASCI control register B ch 1 */
/*#define Z180_CNTLB1_MPBT        0x80
#define Z180_CNTLB1_MP          0x40
#define Z180_CNTLB1_CTS_PS      0x20
#define Z180_CNTLB1_PEO         0x10
#define Z180_CNTLB1_DR          0x08
#define Z180_CNTLB1_SS          0x07

#define Z180_CNTLB1_RESET       0x07
#define Z180_CNTLB1_RMASK       0xff
#define Z180_CNTLB1_WMASK       0xff*/

/* 04 ASCI status register 0 */
/*#define Z180_STAT0_RDRF         0x80
#define Z180_STAT0_OVRN         0x40
#define Z180_STAT0_PE           0x20
#define Z180_STAT0_FE           0x10
#define Z180_STAT0_RIE          0x08
#define Z180_STAT0_DCD0         0x04
#define Z180_STAT0_TDRE         0x02
#define Z180_STAT0_TIE          0x01

#define Z180_STAT0_RESET        0x00
#define Z180_STAT0_RMASK        0xff
#define Z180_STAT0_WMASK        0x09*/

/* 05 ASCI status register 1 */
/*#define Z180_STAT1_RDRF         0x80
#define Z180_STAT1_OVRN         0x40
#define Z180_STAT1_PE           0x20
#define Z180_STAT1_FE           0x10
#define Z180_STAT1_RIE          0x08
#define Z180_STAT1_CTS1E        0x04
#define Z180_STAT1_TDRE         0x02
#define Z180_STAT1_TIE          0x01

#define Z180_STAT1_RESET        0x02
#define Z180_STAT1_RMASK        0xff
#define Z180_STAT1_WMASK        0x0d*/

/* 06 ASCI transmit data register 0 */
//#define Z180_TDR0_TDR           0xff

#define Z180_TDR0_RESET         0x00
#define Z180_TDR0_RMASK         0xff
#define Z180_TDR0_WMASK         0xff

/* 07 ASCI transmit data register 1 */
//#define Z180_TDR1_TDR           0xff

#define Z180_TDR1_RESET         0x00
#define Z180_TDR1_RMASK         0xff
#define Z180_TDR1_WMASK         0xff

/* 08 ASCI receive register 0 */
/*#define Z180_RDR0_RDR           0xff

#define Z180_RDR0_RESET         0x00
#define Z180_RDR0_RMASK         0xff
#define Z180_RDR0_WMASK         0xff*/

/* 09 ASCI receive register 1 */
/*#define Z180_RDR1_RDR           0xff

#define Z180_RDR1_RESET         0x00
#define Z180_RDR1_RMASK         0xff
#define Z180_RDR1_WMASK         0xff*/

/* 0a CSI/O control/status register */
#define Z180_CNTR_EF            0x80
#define Z180_CNTR_EIE           0x40
#define Z180_CNTR_RE            0x20
#define Z180_CNTR_TE            0x10
#define Z180_CNTR_SS            0x07

#define Z180_CNTR_RESET         0x07
#define Z180_CNTR_RMASK         0xff
#define Z180_CNTR_WMASK         0x7f

/* 0b CSI/O transmit/receive register */
#define Z180_TRDR_RESET         0x00
#define Z180_TRDR_RMASK         0xff
#define Z180_TRDR_WMASK         0xff

/* 0c TIMER data register ch 0 L */
#define Z180_TMDR0L_RESET       0x00
#define Z180_TMDR0L_RMASK       0xff
#define Z180_TMDR0L_WMASK       0xff

/* 0d TIMER data register ch 0 H */
#define Z180_TMDR0H_RESET       0x00
#define Z180_TMDR0H_RMASK       0xff
#define Z180_TMDR0H_WMASK       0xff

/* 0e TIMER reload register ch 0 L */
#define Z180_RLDR0L_RESET       0xff
#define Z180_RLDR0L_RMASK       0xff
#define Z180_RLDR0L_WMASK       0xff

/* 0f TIMER reload register ch 0 H */
#define Z180_RLDR0H_RESET       0xff
#define Z180_RLDR0H_RMASK       0xff
#define Z180_RLDR0H_WMASK       0xff

/* 10 TIMER control register */
#define Z180_TCR_TIF1           0x80
#define Z180_TCR_TIF0           0x40
#define Z180_TCR_TIE1           0x20
#define Z180_TCR_TIE0           0x10
#define Z180_TCR_TOC1           0x08
#define Z180_TCR_TOC0           0x04
#define Z180_TCR_TDE1           0x02
#define Z180_TCR_TDE0           0x01

#define Z180_TCR_RESET          0x00
#define Z180_TCR_RMASK          0xff
#define Z180_TCR_WMASK          0x3f

/* 11 reserved */
#define Z180_IO11_RESET         0x00
#define Z180_IO11_RMASK         0xff
#define Z180_IO11_WMASK         0xff

/* 12 (Z8S180/Z8L180) ASCI extension control register 0 */
/*#define Z180_ASEXT0_RDRF        0x80
#define Z180_ASEXT0_DCD0        0x40
#define Z180_ASEXT0_CTS0        0x20
#define Z180_ASEXT0_X1_BIT_CLK0 0x10
#define Z180_ASEXT0_BRG0_MODE   0x08
#define Z180_ASEXT0_BRK_EN      0x04
#define Z180_ASEXT0_BRK_DET     0x02
#define Z180_ASEXT0_BRK_SEND    0x01

#define Z180_ASEXT0_RESET       0x00
#define Z180_ASEXT0_RMASK       0xff
#define Z180_ASEXT0_WMASK       0xfd*/

/* 13 (Z8S180/Z8L180) ASCI extension control register 0 */
/*#define Z180_ASEXT1_RDRF        0x80
#define Z180_ASEXT1_X1_BIT_CLK1 0x10
#define Z180_ASEXT1_BRG1_MODE   0x08
#define Z180_ASEXT1_BRK_EN      0x04
#define Z180_ASEXT1_BRK_DET     0x02
#define Z180_ASEXT1_BRK_SEND    0x01

#define Z180_ASEXT1_RESET       0x00
#define Z180_ASEXT1_RMASK       0xff
#define Z180_ASEXT1_WMASK       0xfd*/


/* 14 TIMER data register ch 1 L */
#define Z180_TMDR1L_RESET       0x00
#define Z180_TMDR1L_RMASK       0xff
#define Z180_TMDR1L_WMASK       0xff

/* 15 TIMER data register ch 1 H */
#define Z180_TMDR1H_RESET       0x00
#define Z180_TMDR1H_RMASK       0xff
#define Z180_TMDR1H_WMASK       0xff

/* 16 TIMER reload register ch 1 L */
#define Z180_RLDR1L_RESET       0x00
#define Z180_RLDR1L_RMASK       0xff
#define Z180_RLDR1L_WMASK       0xff

/* 17 TIMER reload register ch 1 H */
#define Z180_RLDR1H_RESET       0x00
#define Z180_RLDR1H_RMASK       0xff
#define Z180_RLDR1H_WMASK       0xff

/* 18 free running counter */
#define Z180_FRC_RESET          0x00
#define Z180_FRC_RMASK          0xff
#define Z180_FRC_WMASK          0xff

/* 19 reserved */
#define Z180_IO19_RESET         0x00
#define Z180_IO19_RMASK         0xff
#define Z180_IO19_WMASK         0xff

/* 1a ASCI time constant ch 0 L */
/*#define Z180_ASTC0L_RESET       0x00
#define Z180_ASTC0L_RMASK       0xff
#define Z180_ASTC0L_WMASK       0xff*/

/* 1b ASCI time constant ch 0 H */
/*#define Z180_ASTC0H_RESET       0x00
#define Z180_ASTC0H_RMASK       0xff
#define Z180_ASTC0H_WMASK       0xff*/

/* 1c ASCI time constant ch 1 L */
/*#define Z180_ASTC1L_RESET       0x00
#define Z180_ASTC1L_RMASK       0xff
#define Z180_ASTC1L_WMASK       0xff*/

/* 1d ASCI time constant ch 1 H */
/*#define Z180_ASTC1H_RESET       0x00
#define Z180_ASTC1H_RMASK       0xff
#define Z180_ASTC1H_WMASK       0xff*/

/* 1e clock multiplier */
#define Z180_CMR_X2             0x80

#define Z180_CMR_RESET          0x7f
#define Z180_CMR_RMASK          0x80
#define Z180_CMR_WMASK          0x80

/* 1f chip control register */
#define Z180_CCR_CLOCK_DIVIDE   0x80
#define Z180_CCR_STDBY_IDLE1    0x40
#define Z180_CCR_BREXT          0x20
#define Z180_CCR_LNPHI          0x10
#define Z180_CCR_STDBY_IDLE0    0x08
#define Z180_CCR_LNIO           0x04
#define Z180_CCR_LNCPU_CTL      0x02
#define Z180_CCR_LNAD_DATA      0x01

#define Z180_CCR_RESET          0x00
#define Z180_CCR_RMASK          0xff
#define Z180_CCR_WMASK          0xff

/* 20 DMA source address register ch 0 L */
#define Z180_SAR0L_SAR          0xff

#define Z180_SAR0L_RESET        0x00
#define Z180_SAR0L_RMASK        0xff
#define Z180_SAR0L_WMASK        0xff

/* 21 DMA source address register ch 0 H */
#define Z180_SAR0H_SAR          0xff

#define Z180_SAR0H_RESET        0x00
#define Z180_SAR0H_RMASK        0xff
#define Z180_SAR0H_WMASK        0xff

/* 22 DMA source address register ch 0 B */
#define Z180_SAR0B_SAR          0x0f

#define Z180_SAR0B_RESET        0x00
#define Z180_SAR0B_RMASK        0x0f
#define Z180_SAR0B_WMASK        0x0f

/* 23 DMA destination address register ch 0 L */
#define Z180_DAR0L_DAR          0xff

#define Z180_DAR0L_RESET        0x00
#define Z180_DAR0L_RMASK        0xff
#define Z180_DAR0L_WMASK        0xff

/* 24 DMA destination address register ch 0 H */
#define Z180_DAR0H_DAR          0xff

#define Z180_DAR0H_RESET        0x00
#define Z180_DAR0H_RMASK        0xff
#define Z180_DAR0H_WMASK        0xff

/* 25 DMA destination address register ch 0 B */
#define Z180_DAR0B_DAR          0x00

#define Z180_DAR0B_RESET        0x00
#define Z180_DAR0B_RMASK        0x0f
#define Z180_DAR0B_WMASK        0x0f

/* 26 DMA byte count register ch 0 L */
#define Z180_BCR0L_BCR          0xff

#define Z180_BCR0L_RESET        0x00
#define Z180_BCR0L_RMASK        0xff
#define Z180_BCR0L_WMASK        0xff

/* 27 DMA byte count register ch 0 H */
#define Z180_BCR0H_BCR          0xff

#define Z180_BCR0H_RESET        0x00
#define Z180_BCR0H_RMASK        0xff
#define Z180_BCR0H_WMASK        0xff

/* 28 DMA memory address register ch 1 L */
#define Z180_MAR1L_MAR          0xff

#define Z180_MAR1L_RESET        0x00
#define Z180_MAR1L_RMASK        0xff
#define Z180_MAR1L_WMASK        0xff

/* 29 DMA memory address register ch 1 H */
#define Z180_MAR1H_MAR          0xff

#define Z180_MAR1H_RESET        0x00
#define Z180_MAR1H_RMASK        0xff
#define Z180_MAR1H_WMASK        0xff

/* 2a DMA memory address register ch 1 B */
#define Z180_MAR1B_MAR          0x0f

#define Z180_MAR1B_RESET        0x00
#define Z180_MAR1B_RMASK        0x0f
#define Z180_MAR1B_WMASK        0x0f

/* 2b DMA I/O address register ch 1 L */
#define Z180_IAR1L_IAR          0xff

#define Z180_IAR1L_RESET        0x00
#define Z180_IAR1L_RMASK        0xff
#define Z180_IAR1L_WMASK        0xff

/* 2c DMA I/O address register ch 1 H */
#define Z180_IAR1H_IAR          0xff

#define Z180_IAR1H_RESET        0x00
#define Z180_IAR1H_RMASK        0xff
#define Z180_IAR1H_WMASK        0xff

/* 2d (Z8S180/Z8L180) DMA I/O address register ch 1 B */
#define Z180_IAR1B_IAR          0x0f

#define Z180_IAR1B_RESET        0x00
#define Z180_IAR1B_RMASK        0x0f
#define Z180_IAR1B_WMASK        0x0f

/* 2e DMA byte count register ch 1 L */
#define Z180_BCR1L_BCR          0xff

#define Z180_BCR1L_RESET        0x00
#define Z180_BCR1L_RMASK        0xff
#define Z180_BCR1L_WMASK        0xff

/* 2f DMA byte count register ch 1 H */
#define Z180_BCR1H_BCR          0xff

#define Z180_BCR1H_RESET        0x00
#define Z180_BCR1H_RMASK        0xff
#define Z180_BCR1H_WMASK        0xff

/* 30 DMA status register */
#define Z180_DSTAT_DE1          0x80    /* DMA enable ch 1 */
#define Z180_DSTAT_DE0          0x40    /* DMA enable ch 0 */
#define Z180_DSTAT_DWE1         0x20    /* DMA write enable ch 0 (active low) */
#define Z180_DSTAT_DWE0         0x10    /* DMA write enable ch 1 (active low) */
#define Z180_DSTAT_DIE1         0x08    /* DMA IRQ enable ch 1 */
#define Z180_DSTAT_DIE0         0x04    /* DMA IRQ enable ch 0 */
#define Z180_DSTAT_DME          0x01    /* DMA enable (read only) */

#define Z180_DSTAT_RESET        0x30
#define Z180_DSTAT_RMASK        0xfd
#define Z180_DSTAT_WMASK        0xcc

/* 31 DMA mode register */
#define Z180_DMODE_DM           0x30
#define Z180_DMODE_SM           0x0c
#define Z180_DMODE_MMOD         0x02

#define Z180_DMODE_RESET        0x00
#define Z180_DMODE_RMASK        0x3e
#define Z180_DMODE_WMASK        0x3e

/* 32 DMA/WAIT control register */
#define Z180_DCNTL_MWI1         0x80
#define Z180_DCNTL_MWI0         0x40
#define Z180_DCNTL_IWI1         0x20
#define Z180_DCNTL_IWI0         0x10
#define Z180_DCNTL_DMS1         0x08
#define Z180_DCNTL_DMS0         0x04
#define Z180_DCNTL_DIM1         0x02
#define Z180_DCNTL_DIM0         0x01

#define Z180_DCNTL_RESET        0x00
#define Z180_DCNTL_RMASK        0xff
#define Z180_DCNTL_WMASK        0xff

/* 33 INT vector low register */
#define Z180_IL_IL              0xe0

#define Z180_IL_RESET           0x00
#define Z180_IL_RMASK           0xe0
#define Z180_IL_WMASK           0xe0

/* 34 INT/TRAP control register */
#define Z180_ITC_TRAP           0x80
#define Z180_ITC_UFO            0x40
#define Z180_ITC_ITE2           0x04
#define Z180_ITC_ITE1           0x02
#define Z180_ITC_ITE0           0x01

#define Z180_ITC_RESET          0x01
#define Z180_ITC_RMASK          0xc7
#define Z180_ITC_WMASK          0x87

/* 35 reserved */
#define Z180_IO35_RESET         0x00
#define Z180_IO35_RMASK         0xff
#define Z180_IO35_WMASK         0xff

/* 36 refresh control register */
#define Z180_RCR_REFE           0x80
#define Z180_RCR_REFW           0x80
#define Z180_RCR_CYC            0x03

#define Z180_RCR_RESET          0xc0
#define Z180_RCR_RMASK          0xc3
#define Z180_RCR_WMASK          0xc3

/* 37 reserved */
#define Z180_IO37_RESET         0x00
#define Z180_IO37_RMASK         0xff
#define Z180_IO37_WMASK         0xff

/* 38 MMU common base register */
#define Z180_CBR_CB             0xff

#define Z180_CBR_RESET          0x00
#define Z180_CBR_RMASK          0xff
#define Z180_CBR_WMASK          0xff

/* 39 MMU bank base register */
#define Z180_BBR_BB             0xff

#define Z180_BBR_RESET          0x00
#define Z180_BBR_RMASK          0xff
#define Z180_BBR_WMASK          0xff

/* 3a MMU common/bank area register */
#define Z180_CBAR_CA            0xf0
#define Z180_CBAR_BA            0x0f

#define Z180_CBAR_RESET         0xf0
#define Z180_CBAR_RMASK         0xff
#define Z180_CBAR_WMASK         0xff

/* 3b reserved */
#define Z180_IO3B_RESET         0x00
#define Z180_IO3B_RMASK         0xff
#define Z180_IO3B_WMASK         0xff

/* 3c reserved */
#define Z180_IO3C_RESET         0x00
#define Z180_IO3C_RMASK         0xff
#define Z180_IO3C_WMASK         0xff

/* 3d reserved */
#define Z180_IO3D_RESET         0x00
#define Z180_IO3D_RMASK         0xff
#define Z180_IO3D_WMASK         0xff

/* 3e operation mode control register */
#define Z180_OMCR_RESET         0x00
#define Z180_OMCR_RMASK         0xff
#define Z180_OMCR_WMASK         0xff

/* 3f I/O control register */
#define Z180_IOCR_RESET         0x00
#define Z180_IOCR_RMASK         0xff
#define Z180_IOCR_WMASK         0xff


#define Z182_SCR_RESET         0x00
#define Z182_SCR_RMASK         0xff
#define Z182_SCR_WMASK         0xff

#define Z182_RAMUBR_RESET         0xff
#define Z182_RAMUBR_RMASK         0xff
#define Z182_RAMUBR_WMASK         0xff

#define Z182_RAMLBR_RESET         0xff
#define Z182_RAMLBR_RMASK         0xff
#define Z182_RAMLBR_WMASK         0xff

#define Z182_ROMBR_RESET         0xff
#define Z182_ROMBR_RMASK         0xff
#define Z182_ROMBR_WMASK         0xff

#define Z182_WSGCSR_RESET         0xff
#define Z182_WSGCSR_RMASK         0xff
#define Z182_WSGCSR_WMASK         0xff

#define Z182_IEPMUX_RESET         0x5c
#define Z182_IEPMUX_RMASK         0xff
#define Z182_IEPMUX_WMASK         0xff

#define Z182_ENHR_RESET         0x00
#define Z182_ENHR_RMASK         0xff
#define Z182_ENHR_WMASK         0xff

#define Z182_DDRA_RESET         0x00
#define Z182_DDRA_RMASK         0xff
#define Z182_DDRA_WMASK         0xff

#define Z182_DRA_RESET         0x00
#define Z182_DRA_RMASK         0xff
#define Z182_DRA_WMASK         0xff

#define Z182_DDRB_RESET         0xff
#define Z182_DDRB_RMASK         0xff
#define Z182_DDRB_WMASK         0xff

#define Z182_DRB_RESET         0x00
#define Z182_DRB_RMASK         0xff
#define Z182_DRB_WMASK         0xff

#define Z182_DDRC_RESET         0x3f
#define Z182_DDRC_RMASK         0x3f
#define Z182_DDRC_WMASK         0x3f

#define Z182_DRC_RESET         0x00
#define Z182_DRC_RMASK         0xff
#define Z182_DRC_WMASK         0xff

/***************************************************************************
    CPU PREFIXES

    order is important here - see z180tbl.h
***************************************************************************/

#define Z180_PREFIX_op          0
#define Z180_PREFIX_cb          1
#define Z180_PREFIX_dd          2
#define Z180_PREFIX_ed          3
#define Z180_PREFIX_fd          4
#define Z180_PREFIX_xycb        5

#define Z180_PREFIX_COUNT       (Z180_PREFIX_xycb + 1)



UINT8 SZ[256];       /* zero and sign flags */
UINT8 SZ_BIT[256];   /* zero, sign and parity/overflow (=zero) flags for BIT opcode */
UINT8 SZP[256];      /* zero, sign and parity flags */
UINT8 SZHV_inc[256]; /* zero, sign, half carry and overflow flags INC r8 */
UINT8 SZHV_dec[256]; /* zero, sign, half carry and overflow flags DEC r8 */

UINT8 *SZHVC_add;
UINT8 *SZHVC_sub;

UINT8 z180_readcontrol(struct z180_state *cpustate, offs_t port);
void z180_writecontrol(struct z180_state *cpustate, offs_t port, UINT8 data);
int z180_dma0(struct z180_state *cpustate, int max_cycles);
int z180_dma1(struct z180_state *cpustate);
void cpu_burn_z180(device_t *device, int cycles);
//static void cpu_set_info_z180(device_t *device, UINT32 state, cpuinfo *info);
int check_interrupts(struct z180_state *cpustate);

#include "z180ops.h"
#include "z180tbl.h"

#include "z180cb.c"
#include "z180xy.c"
#include "z180dd.c"
#include "z180fd.c"
#include "z180ed.c"
#include "z180op.c"

UINT8 z180_readcontrol(struct z180_state *cpustate, offs_t port)
{
	UINT8 data = 0;
	UINT8 pin;

	if(cpustate->device->m_type == Z180_TYPE_Z182 && (port & 0xff)>= Z182_REGSTART && (port & 0xff)<= Z182_REGEND) {
		switch (port & 0xff) {
			case Z182_SCCACNT:
				//data = cpustate->IO_SCCACNT & Z182_SCCACNT_RMASK;
				data = z80scc_channel_control_read(cpustate->device->z80scc->m_chanA);
				LOG("Z182 '%s' SCCACNT rd $%02x ($%02x)\n", cpustate->device->m_tag, data, data);
				break;
			
			case Z182_SCCAD:
				//data = cpustate->IO_SCCAD & Z182_SCCAD_RMASK;
				data = z80scc_channel_data_read(cpustate->device->z80scc->m_chanA);
				LOG("Z182 '%s' SCCAD rd $%02x ($%02x)\n", cpustate->device->m_tag, data, data);
				break;

			case Z182_SCCBCNT:
				//data = cpustate->IO_SCCBCNT & Z182_SCCBCNT_RMASK;
				data = z80scc_channel_control_read(cpustate->device->z80scc->m_chanB);
				LOG("Z182 '%s' SCCBCNT rd $%02x ($%02x)\n", cpustate->device->m_tag, data, data);
				break;

			case Z182_SCCBD:
				//data = cpustate->IO_SCCBD & Z182_SCCBD_RMASK;
				data = z80scc_channel_data_read(cpustate->device->z80scc->m_chanB);
				LOG("Z182 '%s' SCCBD rd $%02x ($%02x)\n", cpustate->device->m_tag, data, data);
				break;

			case Z182_SCR:
				data = cpustate->IO_SCR & Z182_SCR_RMASK;
				LOG("Z182 '%s' SCR rd $%02x ($%02x)\n", cpustate->device->m_tag, data, cpustate->IO_SCR);
				break;

			case Z182_RAMUBR:
				data = cpustate->IO_RAMUBR & Z182_RAMUBR_RMASK;
				LOG("Z182 '%s' RAMUBR rd $%02x ($%02x)\n", cpustate->device->m_tag, data, cpustate->IO_RAMUBR);
				break;

			case Z182_RAMLBR:
				data = cpustate->IO_RAMLBR & Z182_RAMLBR_RMASK;
				LOG("Z182 '%s' RAMLBR rd $%02x ($%02x)\n", cpustate->device->m_tag, data, cpustate->IO_RAMLBR);
				break;

			case Z182_ROMBR:
				data = cpustate->IO_ROMBR & Z182_ROMBR_RMASK;
				LOG("Z182 '%s' ROMBR rd $%02x ($%02x)\n", cpustate->device->m_tag, data, cpustate->IO_ROMBR);
				break;

			case Z182_WSGCSR:
				data = cpustate->IO_WSGCSR & Z182_WSGCSR_RMASK;
				LOG("Z182 '%s' WSGCSR rd $%02x ($%02x)\n", cpustate->device->m_tag, data, cpustate->IO_WSGCSR);
				break;

			case Z182_IEPMUX:
				data = cpustate->IO_IEPMUX & Z182_IEPMUX_RMASK;
				LOG("Z182 '%s' IEPMUX rd $%02x ($%02x)\n", cpustate->device->m_tag, data, cpustate->IO_IEPMUX);
				break;

			case Z182_ENHR:
				data = cpustate->IO_ENHR & Z182_ENHR_RMASK;
				LOG("Z182 '%s' ENHR rd $%02x ($%02x)\n", cpustate->device->m_tag, data, cpustate->IO_ENHR);
				break;

			case Z182_DDRA:
				data = cpustate->IO_DDRA & Z182_DDRA_RMASK;
				LOG("Z182 '%s' DDRA rd $%02x ($%02x)\n", cpustate->device->m_tag, data, cpustate->IO_DDRA);
				break;

			case Z182_DRA:
				// read external pin
				if (!(cpustate->IO_SCR & 2))
					pin = cpustate->device->m_parport_read_cb(cpustate->device,0);
				else
					pin = 0xff; /* MIMIC on, random byte */
				// p.183 - read works regardless of port being selected
				data = ((pin & cpustate->IO_DDRA) | (cpustate->IO_DRA & Z182_DRA_RMASK & ~cpustate->IO_DDRA));
				LOG("Z182 '%s' DRA rd $%02x ($%02x)\n", cpustate->device->m_tag, data, cpustate->IO_DRA);
				break;

			case Z182_DDRB:
				data = cpustate->IO_DDRB & Z182_DDRB_RMASK;
				LOG("Z182 '%s' DDRB rd $%02x ($%02x)\n", cpustate->device->m_tag, data, cpustate->IO_DDRB);
				break;

			case Z182_DRB:
				// read external pin
				if (cpustate->IO_SCR & 0x60) {
					pin = cpustate->device->m_parport_read_cb(cpustate->device,1);
					if (!(cpustate->IO_SCR & 0x40)) // ASCI 1 on, random bits
						pin |= 0xe0;
					if (!(cpustate->IO_SCR & 0x20)) // ASCI 0 on, random bits
						pin |= 0x1f;
				}
				else
					pin = 0xff; /* ASCI 0,1 on, random byte */
				// p.183 - read works regardless of port being selected
				data = ((pin & cpustate->IO_DDRB) | (cpustate->IO_DRB & Z182_DRB_RMASK & ~cpustate->IO_DDRB));
				LOG("Z182 '%s' DRB rd $%02x ($%02x)\n", cpustate->device->m_tag, data, cpustate->IO_DRB);
				break;

			case Z182_DDRC:
				data = cpustate->IO_DDRC & Z182_DDRC_RMASK;
				LOG("Z182 '%s' DDRC rd $%02x ($%02x)\n", cpustate->device->m_tag, data, cpustate->IO_DDRC);
				break;

			case Z182_DRC:
				// read external pin
				if (cpustate->IO_SCR & 0x80)
					pin = cpustate->device->m_parport_read_cb(cpustate->device,2);
				else
					pin = 0xff; /* ESCC A on, random byte */
				// p.183 - read works regardless of port being selected
				data = ((pin & cpustate->IO_DDRC) | (cpustate->IO_DRC & Z182_DRC_RMASK & ~cpustate->IO_DDRC));
				// add interrupt pin bits
				data |= (!cpustate->irq_state[1]<<6)|(!cpustate->irq_state[2]<<7);
				LOG("Z182 '%s' DRC rd $%02x ($%02x)\n", cpustate->device->m_tag, data, cpustate->IO_DRC);
				break;

			default:
				LOG("Z182 '%s' bogus read control reg %02X\n", cpustate->device->m_tag, port & 0xff);
				break;
		}
	}
	else if((port & (cpustate->IO_IOCR & 0xc0)) == (cpustate->IO_IOCR & 0xc0)) {

		/* remap internal I/O registers */
			port = port - (cpustate->IO_IOCR & 0xc0);

		/* read the internal register */
		switch (port)
		{
		case Z180_CNTLA0:
			//data = cpustate->IO_CNTLA0 & Z180_CNTLA0_RMASK;
			data = z180asci_channel_register_read(cpustate->device->z180asci->m_chan0,Z180_CNTLA0);
			LOG("Z180 '%s' CNTLA0 rd $%02x ($%02x)\n", cpustate->device->m_tag, data, data);
			break;

		case Z180_CNTLA1:
			//data = cpustate->IO_CNTLA1 & Z180_CNTLA1_RMASK;
			data = z180asci_channel_register_read(cpustate->device->z180asci->m_chan1,Z180_CNTLA1);
			LOG("Z180 '%s' CNTLA1 rd $%02x ($%02x)\n", cpustate->device->m_tag, data, data);
			break;

		case Z180_CNTLB0:
			//data = cpustate->IO_CNTLB0 & Z180_CNTLB0_RMASK;
			data = z180asci_channel_register_read(cpustate->device->z180asci->m_chan0,Z180_CNTLB0);
			LOG("Z180 '%s' CNTLB0 rd $%02x ($%02x)\n", cpustate->device->m_tag, data, data);
			break;

		case Z180_CNTLB1:
			//data = cpustate->IO_CNTLB1 & Z180_CNTLB1_RMASK;
			data = z180asci_channel_register_read(cpustate->device->z180asci->m_chan1,Z180_CNTLB1);
			LOG("Z180 '%s' CNTLB1 rd $%02x ($%02x)\n", cpustate->device->m_tag, data, data);
			break;

		case Z180_STAT0:
			//data = cpustate->IO_STAT0 & Z180_STAT0_RMASK;
			data = z180asci_channel_register_read(cpustate->device->z180asci->m_chan0,Z180_STAT0);
	//data |= 0x02; // kludge for 20pacgal
			LOG("Z180 '%s' STAT0  rd $%02x ($%02x)\n", cpustate->device->m_tag, data, data);
			break;

		case Z180_STAT1:
			//data = cpustate->IO_STAT1 & Z180_STAT1_RMASK;
			data = z180asci_channel_register_read(cpustate->device->z180asci->m_chan1,Z180_STAT1);
			LOG("Z180 '%s' STAT1  rd $%02x ($%02x)\n", cpustate->device->m_tag, data, data);
			break;

		case Z180_TDR0:
			//data = cpustate->IO_TDR0 & Z180_TDR0_RMASK;
			data = z180asci_channel_register_read(cpustate->device->z180asci->m_chan0,Z180_TDR0);
			LOG("Z180 '%s' TDR0   rd $%02x ($%02x)\n", cpustate->device->m_tag, data, data);
			break;

		case Z180_TDR1:
			//data = cpustate->IO_TDR1 & Z180_TDR1_RMASK;
			data = z180asci_channel_register_read(cpustate->device->z180asci->m_chan1,Z180_TDR1);
			LOG("Z180 '%s' TDR1   rd $%02x ($%02x)\n", cpustate->device->m_tag, data, data);
			break;

		case Z180_RDR0:
			//data = cpustate->IO_RDR0 & Z180_RDR0_RMASK;
			data = z180asci_channel_register_read(cpustate->device->z180asci->m_chan0,Z180_RDR0);
			LOG("Z180 '%s' RDR0   rd $%02x ($%02x)\n", cpustate->device->m_tag, data, data);
			break;

		case Z180_RDR1:
			//data = cpustate->IO_RDR1 & Z180_RDR1_RMASK;
			data = z180asci_channel_register_read(cpustate->device->z180asci->m_chan1,Z180_RDR1);
			LOG("Z180 '%s' RDR1   rd $%02x ($%02x)\n", cpustate->device->m_tag, data, data);
			break;

		case Z180_CNTR:
			data = cpustate->IO_CNTR & Z180_CNTR_RMASK;
			data &= ~0x10; // Super Famicom Box sets the TE bit then wants it to be toggled after 8 bits transmitted
                        data &= ~0x20; // Clear the RE
			LOG("Z180 '%s' CNTR   rd $%02x ($%02x)\n", cpustate->device->m_tag, data, cpustate->io[port]);
			break;

		case Z180_TRDR:
                        if (cpustate->device->z180csi_rx_cb)
                            data = cpustate->device->z180csi_rx_cb(cpustate->device,0);
                        else {
                            logerror("Z180 '%s' TRDR   rd $%02x ($%02x)\n", cpustate->device->m_tag, data, cpustate->io[port]);
			    data = cpustate->IO_TRDR & Z180_TRDR_RMASK;
                        }
			break;

		case Z180_TMDR0L:
			data = cpustate->tmdr_value[0] & Z180_TMDR0L_RMASK;
			LOG("Z180 '%s' TMDR0L rd $%02x ($%02x)\n", cpustate->device->m_tag, data, cpustate->io[port]);
			/* if timer is counting, latch the MSB and set the latch flag */
			if ((cpustate->IO_TCR & Z180_TCR_TDE0) == 0)
			{
				cpustate->tmdr_latch |= 1;
				cpustate->tmdrh[0] = (cpustate->tmdr_value[0] & 0xff00) >> 8;
			}

			if(cpustate->read_tcr_tmdr[0])
			{
				cpustate->tif[0] = 0; // reset TIF0
				cpustate->read_tcr_tmdr[0] = 0;
			}
			else
			{
				cpustate->read_tcr_tmdr[0] = 1;
			}
			break;

		case Z180_TMDR0H:
			/* read latched value? */
			if (cpustate->tmdr_latch & 1)
			{
				cpustate->tmdr_latch &= ~1;
				data = cpustate->tmdrh[0];
			}
			else
			{
				data = (cpustate->tmdr_value[0] & 0xff00) >> 8;
			}

			if(cpustate->read_tcr_tmdr[0])
			{
				cpustate->tif[0] = 0; // reset TIF0
				cpustate->read_tcr_tmdr[0] = 0;
			}
			else
			{
				cpustate->read_tcr_tmdr[0] = 1;
			}
			LOG("Z180 '%s' TMDR0H rd $%02x ($%02x)\n", cpustate->device->m_tag, data, cpustate->io[port]);
			break;

		case Z180_RLDR0L:
			data = cpustate->IO_RLDR0L & Z180_RLDR0L_RMASK;
			LOG("Z180 '%s' RLDR0L rd $%02x ($%02x)\n", cpustate->device->m_tag, data, cpustate->io[port]);
			break;

		case Z180_RLDR0H:
			data = cpustate->IO_RLDR0H & Z180_RLDR0H_RMASK;
			LOG("Z180 '%s' RLDR0H rd $%02x ($%02x)\n", cpustate->device->m_tag, data, cpustate->io[port]);
			break;

		case Z180_TCR:
			data = (cpustate->IO_TCR & Z180_TCR_RMASK) | (cpustate->tif[0] << 6) | (cpustate->tif[1] << 7);

			if(cpustate->read_tcr_tmdr[0])
			{
				cpustate->tif[0] = 0; // reset TIF0
				cpustate->read_tcr_tmdr[0] = 0;
			}
			else
			{
				cpustate->read_tcr_tmdr[0] = 1;
			}

			if(cpustate->read_tcr_tmdr[1])
			{
				cpustate->tif[1] = 0; // reset TIF1
				cpustate->read_tcr_tmdr[1] = 0;
			}
			else
			{
				cpustate->read_tcr_tmdr[1] = 1;
			}

			LOG("Z180 '%s' TCR    rd $%02x ($%02x)\n", cpustate->device->m_tag, data, cpustate->io[port]);
			break;

		case Z180_IO11:
			data = cpustate->IO_IO11 & Z180_IO11_RMASK;
			LOG("Z180 '%s' IO11   rd $%02x ($%02x)\n", cpustate->device->m_tag, data, cpustate->io[port]);
			break;

		case Z180_ASEXT0:
			//data = cpustate->IO_ASEXT0 & Z180_ASEXT0_RMASK;
			data = z180asci_channel_register_read(cpustate->device->z180asci->m_chan0,Z180_ASEXT0);
			LOG("Z180 '%s' ASEXT0 rd $%02x ($%02x)\n", cpustate->device->m_tag, data, data);
			break;

		case Z180_ASEXT1:
			//data = cpustate->IO_ASEXT1 & Z180_ASEXT1_RMASK;
			data = z180asci_channel_register_read(cpustate->device->z180asci->m_chan1,Z180_ASEXT1);
			LOG("Z180 '%s' ASEXT1 rd $%02x ($%02x)\n", cpustate->device->m_tag, data, data);
			break;

		case Z180_TMDR1L:
			data = cpustate->tmdr_value[1] & Z180_TMDR1L_RMASK;
			LOG("Z180 '%s' TMDR1L rd $%02x ($%02x)\n", cpustate->device->m_tag, data, cpustate->io[port]);
			/* if timer is counting, latch the MSB and set the latch flag */
			if ((cpustate->IO_TCR & Z180_TCR_TDE1) == 0)
			{
				cpustate->tmdr_latch |= 2;
				cpustate->tmdrh[1] = (cpustate->tmdr_value[1] & 0xff00) >> 8;
			}

			if(cpustate->read_tcr_tmdr[1])
			{
				cpustate->tif[1] = 0; // reset TIF1
				cpustate->read_tcr_tmdr[1] = 0;
			}
			else
			{
				cpustate->read_tcr_tmdr[1] = 1;
			}
			break;

		case Z180_TMDR1H:
			/* read latched value? */
			if (cpustate->tmdr_latch & 2)
			{
				cpustate->tmdr_latch &= ~2;
				data = cpustate->tmdrh[1];
			}
			else
			{
				data = (cpustate->tmdr_value[1] & 0xff00) >> 8;
			}

			if(cpustate->read_tcr_tmdr[1])
			{
				cpustate->tif[1] = 0; // reset TIF1
				cpustate->read_tcr_tmdr[1] = 0;
			}
			else
			{
				cpustate->read_tcr_tmdr[1] = 1;
			}
			LOG("Z180 '%s' TMDR1H rd $%02x ($%02x)\n", cpustate->device->m_tag, data, cpustate->io[port]);
			break;

		case Z180_RLDR1L:
			data = cpustate->IO_RLDR1L & Z180_RLDR1L_RMASK;
			LOG("Z180 '%s' RLDR1L rd $%02x ($%02x)\n", cpustate->device->m_tag, data, cpustate->io[port]);
			break;

		case Z180_RLDR1H:
			data = cpustate->IO_RLDR1H & Z180_RLDR1H_RMASK;
			LOG("Z180 '%s' RLDR1H rd $%02x ($%02x)\n", cpustate->device->m_tag, data, cpustate->io[port]);
			break;

		case Z180_FRC:
			data = cpustate->IO_FRC & Z180_FRC_RMASK;
			LOG("Z180 '%s' FRC    rd $%02x ($%02x)\n", cpustate->device->m_tag, data, cpustate->io[port]);
			break;

		case Z180_IO19:
			data = cpustate->IO_IO19 & Z180_IO19_RMASK;
			LOG("Z180 '%s' IO19   rd $%02x ($%02x)\n", cpustate->device->m_tag, data, cpustate->io[port]);
			break;

		case Z180_ASTC0L:
			//data = cpustate->IO_ASTC0L & Z180_ASTC0L_RMASK;
			data = z180asci_channel_register_read(cpustate->device->z180asci->m_chan0,Z180_ASTC0L);
			LOG("Z180 '%s' ASTC0L rd $%02x ($%02x)\n", cpustate->device->m_tag, data, data);
			break;

		case Z180_ASTC0H:
			//data = cpustate->IO_ASTC0H & Z180_ASTC0H_RMASK;
			data = z180asci_channel_register_read(cpustate->device->z180asci->m_chan0,Z180_ASTC0H);
			LOG("Z180 '%s' ASTC0H rd $%02x ($%02x)\n", cpustate->device->m_tag, data, data);
			break;

		case Z180_ASTC1L:
			//data = cpustate->IO_ASTC1L & Z180_ASTC1L_RMASK;
			data = z180asci_channel_register_read(cpustate->device->z180asci->m_chan1,Z180_ASTC1L);
			LOG("Z180 '%s' ASTC1L rd $%02x ($%02x)\n", cpustate->device->m_tag, data, data);
			break;

		case Z180_ASTC1H:
			//data = cpustate->IO_ASTC1H & Z180_ASTC1H_RMASK;
			data = z180asci_channel_register_read(cpustate->device->z180asci->m_chan1,Z180_ASTC1H);
			LOG("Z180 '%s' ASTC1H rd $%02x ($%02x)\n", cpustate->device->m_tag, data, data);
			break;

		case Z180_CMR:
			data = cpustate->IO_CMR & Z180_CMR_RMASK;
			LOG("Z180 '%s' CMR    rd $%02x ($%02x)\n", cpustate->device->m_tag, data, cpustate->io[port]);
			break;

		case Z180_CCR:
			data = cpustate->IO_CCR & Z180_CCR_RMASK;
			LOG("Z180 '%s' CCR    rd $%02x ($%02x)\n", cpustate->device->m_tag, data, cpustate->io[port]);
			break;

		case Z180_SAR0L:
			data = cpustate->IO_SAR0L & Z180_SAR0L_RMASK;
			LOG("Z180 '%s' SAR0L  rd $%02x ($%02x)\n", cpustate->device->m_tag, data, cpustate->io[port]);
			break;

		case Z180_SAR0H:
			data = cpustate->IO_SAR0H & Z180_SAR0H_RMASK;
			LOG("Z180 '%s' SAR0H  rd $%02x ($%02x)\n", cpustate->device->m_tag, data, cpustate->io[port]);
			break;

		case Z180_SAR0B:
			data = cpustate->IO_SAR0B & Z180_SAR0B_RMASK;
			LOG("Z180 '%s' SAR0B  rd $%02x ($%02x)\n", cpustate->device->m_tag, data, cpustate->io[port]);
			break;

		case Z180_DAR0L:
			data = cpustate->IO_DAR0L & Z180_DAR0L_RMASK;
			LOG("Z180 '%s' DAR0L  rd $%02x ($%02x)\n", cpustate->device->m_tag, data, cpustate->io[port]);
			break;

		case Z180_DAR0H:
			data = cpustate->IO_DAR0H & Z180_DAR0H_RMASK;
			LOG("Z180 '%s' DAR0H  rd $%02x ($%02x)\n", cpustate->device->m_tag, data, cpustate->io[port]);
			break;

		case Z180_DAR0B:
			data = cpustate->IO_DAR0B & Z180_DAR0B_RMASK;
			LOG("Z180 '%s' DAR0B  rd $%02x ($%02x)\n", cpustate->device->m_tag, data, cpustate->io[port]);
			break;

		case Z180_BCR0L:
			data = cpustate->IO_BCR0L & Z180_BCR0L_RMASK;
			LOG("Z180 '%s' BCR0L  rd $%02x ($%02x)\n", cpustate->device->m_tag, data, cpustate->io[port]);
			break;

		case Z180_BCR0H:
			data = cpustate->IO_BCR0H & Z180_BCR0H_RMASK;
			LOG("Z180 '%s' BCR0H  rd $%02x ($%02x)\n", cpustate->device->m_tag, data, cpustate->io[port]);
			break;

		case Z180_MAR1L:
			data = cpustate->IO_MAR1L & Z180_MAR1L_RMASK;
			LOG("Z180 '%s' MAR1L  rd $%02x ($%02x)\n", cpustate->device->m_tag, data, cpustate->io[port]);
			break;

		case Z180_MAR1H:
			data = cpustate->IO_MAR1H & Z180_MAR1H_RMASK;
			LOG("Z180 '%s' MAR1H  rd $%02x ($%02x)\n", cpustate->device->m_tag, data, cpustate->io[port]);
			break;

		case Z180_MAR1B:
			data = cpustate->IO_MAR1B & Z180_MAR1B_RMASK;
			LOG("Z180 '%s' MAR1B  rd $%02x ($%02x)\n", cpustate->device->m_tag, data, cpustate->io[port]);
			break;

		case Z180_IAR1L:
			data = cpustate->IO_IAR1L & Z180_IAR1L_RMASK;
			LOG("Z180 '%s' IAR1L  rd $%02x ($%02x)\n", cpustate->device->m_tag, data, cpustate->io[port]);
			break;

		case Z180_IAR1H:
			data = cpustate->IO_IAR1H & Z180_IAR1H_RMASK;
			LOG("Z180 '%s' IAR1H  rd $%02x ($%02x)\n", cpustate->device->m_tag, data, cpustate->io[port]);
			break;

		case Z180_IAR1B:
			data = cpustate->IO_IAR1B & Z180_IAR1B_RMASK;
			LOG("Z180 '%s' IAR1B  rd $%02x ($%02x)\n", cpustate->device->m_tag, data, cpustate->io[port]);
			break;

		case Z180_BCR1L:
			data = cpustate->IO_BCR1L & Z180_BCR1L_RMASK;
			LOG("Z180 '%s' BCR1L  rd $%02x ($%02x)\n", cpustate->device->m_tag, data, cpustate->io[port]);
			break;

		case Z180_BCR1H:
			data = cpustate->IO_BCR1H & Z180_BCR1H_RMASK;
			LOG("Z180 '%s' BCR1H  rd $%02x ($%02x)\n", cpustate->device->m_tag, data, cpustate->io[port]);
			break;

		case Z180_DSTAT:
			data = cpustate->IO_DSTAT & Z180_DSTAT_RMASK;
			LOG("Z180 '%s' DSTAT  rd $%02x ($%02x)\n", cpustate->device->m_tag, data, cpustate->io[port]);
			break;

		case Z180_DMODE:
			data = cpustate->IO_DMODE & Z180_DMODE_RMASK;
			LOG("Z180 '%s' DMODE  rd $%02x ($%02x)\n", cpustate->device->m_tag, data, cpustate->io[port]);
			break;

		case Z180_DCNTL:
			data = cpustate->IO_DCNTL & Z180_DCNTL_RMASK;
			LOG("Z180 '%s' DCNTL  rd $%02x ($%02x)\n", cpustate->device->m_tag, data, cpustate->io[port]);
			break;

		case Z180_IL:
			data = cpustate->IO_IL & Z180_IL_RMASK;
			LOG("Z180 '%s' IL     rd $%02x ($%02x)\n", cpustate->device->m_tag, data, cpustate->io[port]);
			break;

		case Z180_ITC:
			data = cpustate->IO_ITC & Z180_ITC_RMASK;
			LOG("Z180 '%s' ITC    rd $%02x ($%02x)\n", cpustate->device->m_tag, data, cpustate->io[port]);
			break;

		case Z180_IO35:
			data = cpustate->IO_IO35 & Z180_IO35_RMASK;
			LOG("Z180 '%s' IO35   rd $%02x ($%02x)\n", cpustate->device->m_tag, data, cpustate->io[port]);
			break;

		case Z180_RCR:
			data = cpustate->IO_RCR & Z180_RCR_RMASK;
			LOG("Z180 '%s' RCR    rd $%02x ($%02x)\n", cpustate->device->m_tag, data, cpustate->io[port]);
			break;

		case Z180_IO37:
			data = cpustate->IO_IO37 & Z180_IO37_RMASK;
			LOG("Z180 '%s' IO37   rd $%02x ($%02x)\n", cpustate->device->m_tag, data, cpustate->io[port]);
			break;

		case Z180_CBR:
			data = cpustate->IO_CBR & Z180_CBR_RMASK;
			LOG("Z180 '%s' CBR    rd $%02x ($%02x)\n", cpustate->device->m_tag, data, cpustate->io[port]);
			break;

		case Z180_BBR:
			data = cpustate->IO_BBR & Z180_BBR_RMASK;
			LOG("Z180 '%s' BBR    rd $%02x ($%02x)\n", cpustate->device->m_tag, data, cpustate->io[port]);
			break;

		case Z180_CBAR:
			data = cpustate->IO_CBAR & Z180_CBAR_RMASK;
			LOG("Z180 '%s' CBAR   rd $%02x ($%02x)\n", cpustate->device->m_tag, data, cpustate->io[port]);
			break;

		case Z180_IO3B:
			data = cpustate->IO_IO3B & Z180_IO3B_RMASK;
			LOG("Z180 '%s' IO3B   rd $%02x ($%02x)\n", cpustate->device->m_tag, data, cpustate->io[port]);
			break;

		case Z180_IO3C:
			data = cpustate->IO_IO3C & Z180_IO3C_RMASK;
			LOG("Z180 '%s' IO3C   rd $%02x ($%02x)\n", cpustate->device->m_tag, data, cpustate->io[port]);
			break;

		case Z180_IO3D:
			data = cpustate->IO_IO3D & Z180_IO3D_RMASK;
			LOG("Z180 '%s' IO3D   rd $%02x ($%02x)\n", cpustate->device->m_tag, data, cpustate->io[port]);
			break;

		case Z180_OMCR:
			data = cpustate->IO_OMCR & Z180_OMCR_RMASK;
			LOG("Z180 '%s' OMCR   rd $%02x ($%02x)\n", cpustate->device->m_tag, data, cpustate->io[port]);
			break;

		case Z180_IOCR:
			data = cpustate->IO_IOCR & Z180_IOCR_RMASK;
			LOG("Z180 '%s' IOCR   rd $%02x ($%02x)\n", cpustate->device->m_tag, data, cpustate->io[port]);
			break;

		default:
			LOG("Z180 '%s' bogus read control reg %04X\n", cpustate->device->m_tag, port);
			break;
		}
	}
	else
		LOG("Z180 unimplemented read control reg: %04X\n",port );

	return data;
}

void z180_writecontrol(struct z180_state *cpustate, offs_t port, UINT8 data)
{
	if(cpustate->device->m_type == Z180_TYPE_Z182 && (port & 0xff)>= Z182_REGSTART && (port & 0xff)<= Z182_REGEND) {
		switch (port & 0xff) {
			case Z182_SCCACNT:
				LOG("Z182 '%s' SCCACNT wr $%02x ($%02x)\n", cpustate->device->m_tag, data,  data);
				//cpustate->IO_SCCACNT = (cpustate->IO_SCCACNT & ~Z182_SCCACNT_WMASK) | (data & Z182_SCCACNT_WMASK);
				z80scc_channel_control_write(cpustate->device->z80scc->m_chanA, data);
				break;

			case Z182_SCCAD:
				LOG("Z182 '%s' SCCAD wr $%02x ($%02x)\n", cpustate->device->m_tag, data,  data);
				//cpustate->IO_SCCAD = (cpustate->IO_SCCAD & ~Z182_SCCAD_WMASK) | (data & Z182_SCCAD_WMASK);
				z80scc_channel_data_write(cpustate->device->z80scc->m_chanA, data);
				break;

			case Z182_SCCBCNT:
				LOG("Z182 '%s' SCCBCNT wr $%02x ($%02x)\n", cpustate->device->m_tag, data,  data);
				//cpustate->IO_SCCBCNT = (cpustate->IO_SCCBCNT & ~Z182_SCCBCNT_WMASK) | (data & Z182_SCCBCNT_WMASK);
				z80scc_channel_control_write(cpustate->device->z80scc->m_chanB, data);
				break;

			case Z182_SCCBD:
				LOG("Z182 '%s' SCCBD wr $%02x ($%02x)\n", cpustate->device->m_tag, data,  data);
				//cpustate->IO_SCCBD = (cpustate->IO_SCCBD & ~Z182_SCCBD_WMASK) | (data & Z182_SCCBD_WMASK);
				z80scc_channel_data_write(cpustate->device->z80scc->m_chanB, data);
				break;

			case Z182_SCR:
				LOG("Z182 '%s' SCR wr $%02x ($%02x)\n", cpustate->device->m_tag, data,  data & Z182_SCR_WMASK);
				cpustate->IO_SCR = (cpustate->IO_SCR & ~Z182_SCR_WMASK) | (data & Z182_SCR_WMASK);
				break;

			case Z182_RAMUBR:
				LOG("Z182 '%s' RAMUBR wr $%02x ($%02x)\n", cpustate->device->m_tag, data,  data & Z182_RAMUBR_WMASK);
				cpustate->IO_RAMUBR = (cpustate->IO_RAMUBR & ~Z182_RAMUBR_WMASK) | (data & Z182_RAMUBR_WMASK);
				break;

			case Z182_RAMLBR:
				LOG("Z182 '%s' RAMLBR wr $%02x ($%02x)\n", cpustate->device->m_tag, data,  data & Z182_RAMLBR_WMASK);
				cpustate->IO_RAMLBR = (cpustate->IO_RAMLBR & ~Z182_RAMLBR_WMASK) | (data & Z182_RAMLBR_WMASK);
				break;

			case Z182_ROMBR:
				LOG("Z182 '%s' ROMBR wr $%02x ($%02x)\n", cpustate->device->m_tag, data,  data & Z182_ROMBR_WMASK);
				cpustate->IO_ROMBR = (cpustate->IO_ROMBR & ~Z182_ROMBR_WMASK) | (data & Z182_ROMBR_WMASK);
				break;

			case Z182_WSGCSR:
				LOG("Z182 '%s' WSGCSR wr $%02x ($%02x)\n", cpustate->device->m_tag, data,  data & Z182_WSGCSR_WMASK);
				cpustate->IO_WSGCSR = (cpustate->IO_WSGCSR & ~Z182_WSGCSR_WMASK) | (data & Z182_WSGCSR_WMASK);
				break;

			case Z182_IEPMUX:
				LOG("Z182 '%s' IEPMUX wr $%02x ($%02x)\n", cpustate->device->m_tag, data,  data & Z182_IEPMUX_WMASK);
				cpustate->IO_IEPMUX = (cpustate->IO_IEPMUX & ~Z182_IEPMUX_WMASK) | (data & Z182_IEPMUX_WMASK);
				break;

			case Z182_ENHR:
				LOG("Z182 '%s' ENHR wr $%02x ($%02x)\n", cpustate->device->m_tag, data,  data & Z182_ENHR_WMASK);
				cpustate->IO_ENHR = (cpustate->IO_ENHR & ~Z182_ENHR_WMASK) | (data & Z182_ENHR_WMASK);
				break;

			case Z182_DDRA:
				LOG("Z182 '%s' DDRA wr $%02x ($%02x)\n", cpustate->device->m_tag, data,  data & Z182_DDRA_WMASK);
				cpustate->IO_DDRA = (cpustate->IO_DDRA & ~Z182_DDRA_WMASK) | (data & Z182_DDRA_WMASK);
				// shadow write when DDRA changed
				if (!(cpustate->IO_SCR & 2))
					cpustate->device->m_parport_write_cb(cpustate->device,0,cpustate->IO_DRA & ~cpustate->IO_DDRA);
				break;

			case Z182_DRA:
				LOG("Z182 '%s' DRA wr $%02x ($%02x)\n", cpustate->device->m_tag, data,  data & Z182_DRA_WMASK);
				cpustate->IO_DRA = (cpustate->IO_DRA & ~Z182_DRA_WMASK) | (data & Z182_DRA_WMASK);
				if (!(cpustate->IO_SCR & 2))
					cpustate->device->m_parport_write_cb(cpustate->device,0,cpustate->IO_DRA & ~cpustate->IO_DDRA);
				break;

			case Z182_DDRB:
				LOG("Z182 '%s' DDRB wr $%02x ($%02x)\n", cpustate->device->m_tag, data,  data & Z182_DDRB_WMASK);
				cpustate->IO_DDRB = (cpustate->IO_DDRB & ~Z182_DDRB_WMASK) | (data & Z182_DDRB_WMASK);
				// shadow write when DDRB changed
				if (cpustate->IO_SCR & 0x60)
					cpustate->device->m_parport_write_cb(cpustate->device,1,
						((cpustate->IO_SCR & 0x40)?(cpustate->IO_DRB & ~cpustate->IO_DDRB) &0xe0:0) | 
						((cpustate->IO_SCR & 0x20)?(cpustate->IO_DRB & ~cpustate->IO_DDRB) &0x1f:0));
				break;

			case Z182_DRB:
				LOG("Z182 '%s' DRB wr $%02x ($%02x)\n", cpustate->device->m_tag, data,  data & Z182_DRB_WMASK);
				cpustate->IO_DRB = (cpustate->IO_DRB & ~Z182_DRB_WMASK) | (data & Z182_DRB_WMASK);
				if (cpustate->IO_SCR & 0x60)
					cpustate->device->m_parport_write_cb(cpustate->device,1,
						((cpustate->IO_SCR & 0x40)?(cpustate->IO_DRB & ~cpustate->IO_DDRB) &0xe0:0) | 
						((cpustate->IO_SCR & 0x20)?(cpustate->IO_DRB & ~cpustate->IO_DDRB) &0x1f:0));
				break;

			case Z182_DDRC:
				LOG("Z182 '%s' DDRC wr $%02x ($%02x)\n", cpustate->device->m_tag, data,  data & Z182_DDRC_WMASK);
				cpustate->IO_DDRC = (cpustate->IO_DDRC & ~Z182_DDRC_WMASK) | (data & Z182_DDRC_WMASK);
				// shadow write when DDRC changed
				if (cpustate->IO_SCR & 0x80)
					cpustate->device->m_parport_write_cb(cpustate->device,2, cpustate->IO_DRC & ~cpustate->IO_DDRC);
				break;

			case Z182_DRC:
				LOG("Z182 '%s' DRC wr $%02x ($%02x)\n", cpustate->device->m_tag, data,  data & Z182_DRC_WMASK);
				cpustate->IO_DRC = (cpustate->IO_DRC & ~Z182_DRC_WMASK) | (data & Z182_DRC_WMASK);
				if (cpustate->IO_SCR & 0x80)
					cpustate->device->m_parport_write_cb(cpustate->device,2, cpustate->IO_DRC & ~cpustate->IO_DDRC);
				break;

			default:
				LOG("Z182 '%s' bogus write control reg %02X:$%02X\n", cpustate->device->m_tag, port &0xff, data);
				break;
		}
	}
	else if((port & (cpustate->IO_IOCR & 0xc0)) == (cpustate->IO_IOCR & 0xc0)) {

		/* remap internal I/O registers */
		port = port - (cpustate->IO_IOCR & 0xc0);

		/* store the data in the internal register */
		switch (port)
		{
		case Z180_CNTLA0:
			LOG("Z180 '%s' CNTLA0 wr $%02x ($%02x)\n", cpustate->device->m_tag, data,  data);
			//cpustate->IO_CNTLA0 = (cpustate->IO_CNTLA0 & ~Z180_CNTLA0_WMASK) | (data & Z180_CNTLA0_WMASK);
			z180asci_channel_register_write(cpustate->device->z180asci->m_chan0,Z180_CNTLA0,data);
			break;

		case Z180_CNTLA1:
			LOG("Z180 '%s' CNTLA1 wr $%02x ($%02x)\n", cpustate->device->m_tag, data,  data);
			//cpustate->IO_CNTLA1 = (cpustate->IO_CNTLA1 & ~Z180_CNTLA1_WMASK) | (data & Z180_CNTLA1_WMASK);
			z180asci_channel_register_write(cpustate->device->z180asci->m_chan1,Z180_CNTLA1,data);
			break;

		case Z180_CNTLB0:
			LOG("Z180 '%s' CNTLB0 wr $%02x ($%02x)\n", cpustate->device->m_tag, data,  data);
			//cpustate->IO_CNTLB0 = (cpustate->IO_CNTLB0 & ~Z180_CNTLB0_WMASK) | (data & Z180_CNTLB0_WMASK);
			z180asci_channel_register_write(cpustate->device->z180asci->m_chan0,Z180_CNTLB0,data);
			break;

		case Z180_CNTLB1:
			LOG("Z180 '%s' CNTLB1 wr $%02x ($%02x)\n", cpustate->device->m_tag, data,  data);
			//cpustate->IO_CNTLB1 = (cpustate->IO_CNTLB1 & ~Z180_CNTLB1_WMASK) | (data & Z180_CNTLB1_WMASK);
			z180asci_channel_register_write(cpustate->device->z180asci->m_chan1,Z180_CNTLB1,data);
			break;

		case Z180_STAT0:
			LOG("Z180 '%s' STAT0  wr $%02x ($%02x)\n", cpustate->device->m_tag, data,  data);
			//cpustate->IO_STAT0 = (cpustate->IO_STAT0 & ~Z180_STAT0_WMASK) | (data & Z180_STAT0_WMASK);
			z180asci_channel_register_write(cpustate->device->z180asci->m_chan0,Z180_STAT0,data);
			break;

		case Z180_STAT1:
			LOG("Z180 '%s' STAT1  wr $%02x ($%02x)\n", cpustate->device->m_tag, data,  data);
			//cpustate->IO_STAT1 = (cpustate->IO_STAT1 & ~Z180_STAT1_WMASK) | (data & Z180_STAT1_WMASK);
			z180asci_channel_register_write(cpustate->device->z180asci->m_chan1,Z180_STAT1,data);
			break;

		case Z180_TDR0:
			LOG("Z180 '%s' TDR0   wr $%02x ($%02x)\n", cpustate->device->m_tag, data,  data);
			//cpustate->IO_TDR0 = (cpustate->IO_TDR0 & ~Z180_TDR0_WMASK) | (data & Z180_TDR0_WMASK);
			z180asci_channel_register_write(cpustate->device->z180asci->m_chan0,Z180_TDR0,data);
			break;

		case Z180_TDR1:
			LOG("Z180 '%s' TDR1   wr $%02x ($%02x)\n", cpustate->device->m_tag, data,  data);
			//cpustate->IO_TDR1 = (cpustate->IO_TDR1 & ~Z180_TDR1_WMASK) | (data & Z180_TDR1_WMASK);
			z180asci_channel_register_write(cpustate->device->z180asci->m_chan1,Z180_TDR1,data);
			break;

		case Z180_RDR0:
			LOG("Z180 '%s' RDR0   wr $%02x ($%02x)\n", cpustate->device->m_tag, data,  data);
			//cpustate->IO_RDR0 = (cpustate->IO_RDR0 & ~Z180_RDR0_WMASK) | (data & Z180_RDR0_WMASK);
			z180asci_channel_register_write(cpustate->device->z180asci->m_chan0,Z180_RDR0,data);
			break;

		case Z180_RDR1:
			LOG("Z180 '%s' RDR1   wr $%02x ($%02x)\n", cpustate->device->m_tag, data,  data);
			//cpustate->IO_RDR1 = (cpustate->IO_RDR1 & ~Z180_RDR1_WMASK) | (data & Z180_RDR1_WMASK);
			z180asci_channel_register_write(cpustate->device->z180asci->m_chan1,Z180_RDR1,data);
			break;

		case Z180_CNTR:
			LOG("Z180 '%s' CNTR   wr $%02x ($%02x)\n", cpustate->device->m_tag, data,  data & Z180_CNTR_WMASK);
			cpustate->IO_CNTR = (cpustate->IO_CNTR & ~Z180_CNTR_WMASK) | (data & Z180_CNTR_WMASK);
			break;

		case Z180_TRDR:
			cpustate->IO_TRDR = (cpustate->IO_TRDR & ~Z180_TRDR_WMASK) | (data & Z180_TRDR_WMASK);
                        if (cpustate->device->z180csi_tx_cb)
                            cpustate->device->z180csi_tx_cb(cpustate->device,0,data);
                        else
                            LOG("Z180 '%s' TRDR   wr $%02x ($%02x)\n", cpustate->device->m_tag, data,  data & Z180_TRDR_WMASK);
			break;

		case Z180_TMDR0L:
			LOG("Z180 '%s' TMDR0L wr $%02x ($%02x)\n", cpustate->device->m_tag, data,  data & Z180_TMDR0L_WMASK);
			cpustate->IO_TMDR0L = data & Z180_TMDR0L_WMASK;
			cpustate->tmdr_value[0] = (cpustate->tmdr_value[0] & 0xff00) | cpustate->IO_TMDR0L;
			break;

		case Z180_TMDR0H:
			LOG("Z180 '%s' TMDR0H wr $%02x ($%02x)\n", cpustate->device->m_tag, data,  data & Z180_TMDR0H_WMASK);
			cpustate->IO_TMDR0H = data & Z180_TMDR0H_WMASK;
			cpustate->tmdr_value[0] = (cpustate->tmdr_value[0] & 0x00ff) | (cpustate->IO_TMDR0H << 8);
			break;

		case Z180_RLDR0L:
			LOG("Z180 '%s' RLDR0L wr $%02x ($%02x)\n", cpustate->device->m_tag, data,  data & Z180_RLDR0L_WMASK);
			cpustate->IO_RLDR0L = (cpustate->IO_RLDR0L & ~Z180_RLDR0L_WMASK) | (data & Z180_RLDR0L_WMASK);
			break;

		case Z180_RLDR0H:
			LOG("Z180 '%s' RLDR0H wr $%02x ($%02x)\n", cpustate->device->m_tag, data,  data & Z180_RLDR0H_WMASK);
			cpustate->IO_RLDR0H = (cpustate->IO_RLDR0H & ~Z180_RLDR0H_WMASK) | (data & Z180_RLDR0H_WMASK);
			break;

		case Z180_TCR:
			LOG("Z180 '%s' TCR    wr $%02x ($%02x)\n", cpustate->device->m_tag, data,  data & Z180_TCR_WMASK);
			{
				UINT16 old = cpustate->IO_TCR;
				/* Force reload on state change */
				cpustate->IO_TCR = (cpustate->IO_TCR & ~Z180_TCR_WMASK) | (data & Z180_TCR_WMASK);
				if (!(old & Z180_TCR_TDE0) && (cpustate->IO_TCR & Z180_TCR_TDE0))
					cpustate->tmdr_value[0] = 0; //cpustate->IO_RLDR0L | (cpustate->IO_RLDR0H << 8);
				if (!(old & Z180_TCR_TDE1) && (cpustate->IO_TCR & Z180_TCR_TDE1))
					cpustate->tmdr_value[1] = 0; //cpustate->IO_RLDR1L | (cpustate->IO_RLDR1H << 8);
			}

			break;

		case Z180_IO11:
			LOG("Z180 '%s' IO11   wr $%02x ($%02x)\n", cpustate->device->m_tag, data,  data & Z180_IO11_WMASK);
			cpustate->IO_IO11 = (cpustate->IO_IO11 & ~Z180_IO11_WMASK) | (data & Z180_IO11_WMASK);
			break;

		case Z180_ASEXT0:
			LOG("Z180 '%s' ASEXT0 wr $%02x ($%02x)\n", cpustate->device->m_tag, data,  data);
			//cpustate->IO_ASEXT0 = (cpustate->IO_ASEXT0 & ~Z180_ASEXT0_WMASK) | (data & Z180_ASEXT0_WMASK);
			z180asci_channel_register_write(cpustate->device->z180asci->m_chan0,Z180_ASEXT0,data);
			break;

		case Z180_ASEXT1:
			LOG("Z180 '%s' ASEXT1 wr $%02x ($%02x)\n", cpustate->device->m_tag, data,  data);
			//cpustate->IO_ASEXT1 = (cpustate->IO_ASEXT1 & ~Z180_ASEXT1_WMASK) | (data & Z180_ASEXT1_WMASK);
			z180asci_channel_register_write(cpustate->device->z180asci->m_chan1,Z180_ASEXT1,data);
			break;

		case Z180_TMDR1L:
			LOG("Z180 '%s' TMDR1L wr $%02x ($%02x)\n", cpustate->device->m_tag, data,  data & Z180_TMDR1L_WMASK);
			cpustate->IO_TMDR1L = data & Z180_TMDR1L_WMASK;
			cpustate->tmdr_value[1] = (cpustate->tmdr_value[1] & 0xff00) | cpustate->IO_TMDR1L;
			break;

		case Z180_TMDR1H:
			LOG("Z180 '%s' TMDR1H wr $%02x ($%02x)\n", cpustate->device->m_tag, data,  data & Z180_TMDR1H_WMASK);
			cpustate->IO_TMDR1H = data & Z180_TMDR1H_WMASK;
			cpustate->tmdr_value[1] = (cpustate->tmdr_value[1] & 0x00ff) | cpustate->IO_TMDR1H;
			break;

		case Z180_RLDR1L:
			LOG("Z180 '%s' RLDR1L wr $%02x ($%02x)\n", cpustate->device->m_tag, data,  data & Z180_RLDR1L_WMASK);
			cpustate->IO_RLDR1L = (cpustate->IO_RLDR1L & ~Z180_RLDR1L_WMASK) | (data & Z180_RLDR1L_WMASK);
			break;

		case Z180_RLDR1H:
			LOG("Z180 '%s' RLDR1H wr $%02x ($%02x)\n", cpustate->device->m_tag, data,  data & Z180_RLDR1H_WMASK);
			cpustate->IO_RLDR1H = (cpustate->IO_RLDR1H & ~Z180_RLDR1H_WMASK) | (data & Z180_RLDR1H_WMASK);
			break;

		case Z180_FRC:
			LOG("Z180 '%s' FRC    wr $%02x ($%02x)\n", cpustate->device->m_tag, data,  data & Z180_FRC_WMASK);
			cpustate->IO_FRC = (cpustate->IO_FRC & ~Z180_FRC_WMASK) | (data & Z180_FRC_WMASK);
			break;

		case Z180_IO19:
			LOG("Z180 '%s' IO19   wr $%02x ($%02x)\n", cpustate->device->m_tag, data,  data & Z180_IO19_WMASK);
			cpustate->IO_IO19 = (cpustate->IO_IO19 & ~Z180_IO19_WMASK) | (data & Z180_IO19_WMASK);
			break;

		case Z180_ASTC0L:
			LOG("Z180 '%s' ASTC0L wr $%02x ($%02x)\n", cpustate->device->m_tag, data,  data);
			//cpustate->IO_ASTC0L = (cpustate->IO_ASTC0L & ~Z180_ASTC0L_WMASK) | (data & Z180_ASTC0L_WMASK);
			z180asci_channel_register_write(cpustate->device->z180asci->m_chan0,Z180_ASTC0L,data);
			break;

		case Z180_ASTC0H:
			LOG("Z180 '%s' ASTC0H wr $%02x ($%02x)\n", cpustate->device->m_tag, data,  data);
			//cpustate->IO_ASTC0H = (cpustate->IO_ASTC0H & ~Z180_ASTC0H_WMASK) | (data & Z180_ASTC0H_WMASK);
			z180asci_channel_register_write(cpustate->device->z180asci->m_chan0,Z180_ASTC0H,data);
			break;

		case Z180_ASTC1L:
			LOG("Z180 '%s' ASTC1L wr $%02x ($%02x)\n", cpustate->device->m_tag, data,  data);
			//cpustate->IO_ASTC1L = (cpustate->IO_ASTC1L & ~Z180_ASTC1L_WMASK) | (data & Z180_ASTC1L_WMASK);
			z180asci_channel_register_write(cpustate->device->z180asci->m_chan1,Z180_ASTC1L,data);
			break;

		case Z180_ASTC1H:
			LOG("Z180 '%s' ASTC1H wr $%02x ($%02x)\n", cpustate->device->m_tag, data,  data);
			//cpustate->IO_ASTC1H = (cpustate->IO_ASTC1H & ~Z180_ASTC1H_WMASK) | (data & Z180_ASTC1H_WMASK);
			z180asci_channel_register_write(cpustate->device->z180asci->m_chan1,Z180_ASTC1H,data);
			break;

		case Z180_CMR:
			LOG("Z180 '%s' CMR    wr $%02x ($%02x)\n", cpustate->device->m_tag, data,  data & Z180_CMR_WMASK);
			cpustate->IO_CMR = (cpustate->IO_CMR & ~Z180_CMR_WMASK) | (data & Z180_CMR_WMASK);
			break;

		case Z180_CCR:
			LOG("Z180 '%s' CCR    wr $%02x ($%02x)\n", cpustate->device->m_tag, data,  data & Z180_CCR_WMASK);
			cpustate->IO_CCR = (cpustate->IO_CCR & ~Z180_CCR_WMASK) | (data & Z180_CCR_WMASK);
			break;

		case Z180_SAR0L:
			LOG("Z180 '%s' SAR0L  wr $%02x ($%02x)\n", cpustate->device->m_tag, data,  data & Z180_SAR0L_WMASK);
			cpustate->IO_SAR0L = (cpustate->IO_SAR0L & ~Z180_SAR0L_WMASK) | (data & Z180_SAR0L_WMASK);
			break;

		case Z180_SAR0H:
			LOG("Z180 '%s' SAR0H  wr $%02x ($%02x)\n", cpustate->device->m_tag, data,  data & Z180_SAR0H_WMASK);
			cpustate->IO_SAR0H = (cpustate->IO_SAR0H & ~Z180_SAR0H_WMASK) | (data & Z180_SAR0H_WMASK);
			break;

		case Z180_SAR0B:
			LOG("Z180 '%s' SAR0B  wr $%02x ($%02x)\n", cpustate->device->m_tag, data,  data & Z180_SAR0B_WMASK);
			cpustate->IO_SAR0B = (cpustate->IO_SAR0B & ~Z180_SAR0B_WMASK) | (data & Z180_SAR0B_WMASK);
			break;

		case Z180_DAR0L:
			LOG("Z180 '%s' DAR0L  wr $%02x ($%02x)\n", cpustate->device->m_tag, data,  data & Z180_DAR0L_WMASK);
			cpustate->IO_DAR0L = (cpustate->IO_DAR0L & ~Z180_DAR0L_WMASK) | (data & Z180_DAR0L_WMASK);
			break;

		case Z180_DAR0H:
			LOG("Z180 '%s' DAR0H  wr $%02x ($%02x)\n", cpustate->device->m_tag, data,  data & Z180_DAR0H_WMASK);
			cpustate->IO_DAR0H = (cpustate->IO_DAR0H & ~Z180_DAR0H_WMASK) | (data & Z180_DAR0H_WMASK);
			break;

		case Z180_DAR0B:
			LOG("Z180 '%s' DAR0B  wr $%02x ($%02x)\n", cpustate->device->m_tag, data,  data & Z180_DAR0B_WMASK);
			cpustate->IO_DAR0B = (cpustate->IO_DAR0B & ~Z180_DAR0B_WMASK) | (data & Z180_DAR0B_WMASK);
			break;

		case Z180_BCR0L:
			LOG("Z180 '%s' BCR0L  wr $%02x ($%02x)\n", cpustate->device->m_tag, data,  data & Z180_BCR0L_WMASK);
			cpustate->IO_BCR0L = (cpustate->IO_BCR0L & ~Z180_BCR0L_WMASK) | (data & Z180_BCR0L_WMASK);
			break;

		case Z180_BCR0H:
			LOG("Z180 '%s' BCR0H  wr $%02x ($%02x)\n", cpustate->device->m_tag, data,  data & Z180_BCR0H_WMASK);
			cpustate->IO_BCR0H = (cpustate->IO_BCR0H & ~Z180_BCR0H_WMASK) | (data & Z180_BCR0H_WMASK);
			break;

		case Z180_MAR1L:
			LOG("Z180 '%s' MAR1L  wr $%02x ($%02x)\n", cpustate->device->m_tag, data,  data & Z180_MAR1L_WMASK);
			cpustate->IO_MAR1L = (cpustate->IO_MAR1L & ~Z180_MAR1L_WMASK) | (data & Z180_MAR1L_WMASK);
			break;

		case Z180_MAR1H:
			LOG("Z180 '%s' MAR1H  wr $%02x ($%02x)\n", cpustate->device->m_tag, data,  data & Z180_MAR1H_WMASK);
			cpustate->IO_MAR1H = (cpustate->IO_MAR1H & ~Z180_MAR1H_WMASK) | (data & Z180_MAR1H_WMASK);
			break;

		case Z180_MAR1B:
			LOG("Z180 '%s' MAR1B  wr $%02x ($%02x)\n", cpustate->device->m_tag, data,  data & Z180_MAR1B_WMASK);
			cpustate->IO_MAR1B = (cpustate->IO_MAR1B & ~Z180_MAR1B_WMASK) | (data & Z180_MAR1B_WMASK);
			break;

		case Z180_IAR1L:
			LOG("Z180 '%s' IAR1L  wr $%02x ($%02x)\n", cpustate->device->m_tag, data,  data & Z180_IAR1L_WMASK);
			cpustate->IO_IAR1L = (cpustate->IO_IAR1L & ~Z180_IAR1L_WMASK) | (data & Z180_IAR1L_WMASK);
			break;

		case Z180_IAR1H:
			LOG("Z180 '%s' IAR1H  wr $%02x ($%02x)\n", cpustate->device->m_tag, data,  data & Z180_IAR1H_WMASK);
			cpustate->IO_IAR1H = (cpustate->IO_IAR1H & ~Z180_IAR1H_WMASK) | (data & Z180_IAR1H_WMASK);
			break;

		case Z180_IAR1B:
			LOG("Z180 '%s' IAR1B  wr $%02x ($%02x)\n", cpustate->device->m_tag, data,  data & Z180_IAR1B_WMASK);
			cpustate->IO_IAR1B = (cpustate->IO_IAR1B & ~Z180_IAR1B_WMASK) | (data & Z180_IAR1B_WMASK);
			break;

		case Z180_BCR1L:
			LOG("Z180 '%s' BCR1L  wr $%02x ($%02x)\n", cpustate->device->m_tag, data,  data & Z180_BCR1L_WMASK);
			cpustate->IO_BCR1L = (cpustate->IO_BCR1L & ~Z180_BCR1L_WMASK) | (data & Z180_BCR1L_WMASK);
			break;

		case Z180_BCR1H:
			LOG("Z180 '%s' BCR1H  wr $%02x ($%02x)\n", cpustate->device->m_tag, data,  data & Z180_BCR1H_WMASK);
			cpustate->IO_BCR1H = (cpustate->IO_BCR1H & ~Z180_BCR1H_WMASK) | (data & Z180_BCR1H_WMASK);
			break;

		case Z180_DSTAT:
			LOG("Z180 '%s' DSTAT  wr $%02x ($%02x)\n", cpustate->device->m_tag, data,  data & Z180_DSTAT_WMASK);
			cpustate->IO_DSTAT = (cpustate->IO_DSTAT & ~Z180_DSTAT_WMASK) | (data & Z180_DSTAT_WMASK);
			if ((data & (Z180_DSTAT_DE1 | Z180_DSTAT_DWE1)) == Z180_DSTAT_DE1)
				cpustate->IO_DSTAT |= Z180_DSTAT_DME;  /* DMA enable */
			if ((data & (Z180_DSTAT_DE0 | Z180_DSTAT_DWE0)) == Z180_DSTAT_DE0)
				cpustate->IO_DSTAT |= Z180_DSTAT_DME;  /* DMA enable */
			break;

		case Z180_DMODE:
			LOG("Z180 '%s' DMODE  wr $%02x ($%02x)\n", cpustate->device->m_tag, data,  data & Z180_DMODE_WMASK);
			cpustate->IO_DMODE = (cpustate->IO_DMODE & ~Z180_DMODE_WMASK) | (data & Z180_DMODE_WMASK);
			break;

		case Z180_DCNTL:
			LOG("Z180 '%s' DCNTL  wr $%02x ($%02x)\n", cpustate->device->m_tag, data,  data & Z180_DCNTL_WMASK);
			cpustate->IO_DCNTL = (cpustate->IO_DCNTL & ~Z180_DCNTL_WMASK) | (data & Z180_DCNTL_WMASK);
			break;

		case Z180_IL:
			LOG("Z180 '%s' IL     wr $%02x ($%02x)\n", cpustate->device->m_tag, data,  data & Z180_IL_WMASK);
			cpustate->IO_IL = (cpustate->IO_IL & ~Z180_IL_WMASK) | (data & Z180_IL_WMASK);
			break;

		case Z180_ITC:
			LOG("Z180 '%s' ITC    wr $%02x ($%02x)\n", cpustate->device->m_tag, data, data & (Z180_ITC_WMASK & ~Z180_ITC_TRAP));
			if (data & Z180_ITC_TRAP) // no change
				cpustate->IO_ITC = (cpustate->IO_ITC & ~(Z180_ITC_WMASK & ~Z180_ITC_TRAP)) | (data & (Z180_ITC_WMASK & ~Z180_ITC_TRAP));
			else // reset TRAP
				cpustate->IO_ITC = (cpustate->IO_ITC & ~Z180_ITC_WMASK) | (data & (Z180_ITC_WMASK & ~Z180_ITC_TRAP));
			break;

		case Z180_IO35:
			LOG("Z180 '%s' IO35   wr $%02x ($%02x)\n", cpustate->device->m_tag, data,  data & Z180_IO35_WMASK);
			cpustate->IO_IO35 = (cpustate->IO_IO35 & ~Z180_IO35_WMASK) | (data & Z180_IO35_WMASK);
			break;

		case Z180_RCR:
			LOG("Z180 '%s' RCR    wr $%02x ($%02x)\n", cpustate->device->m_tag, data,  data & Z180_RCR_WMASK);
			cpustate->IO_RCR = (cpustate->IO_RCR & ~Z180_RCR_WMASK) | (data & Z180_RCR_WMASK);
			break;

		case Z180_IO37:
			LOG("Z180 '%s' IO37   wr $%02x ($%02x)\n", cpustate->device->m_tag, data,  data & Z180_IO37_WMASK);
			cpustate->IO_IO37 = (cpustate->IO_IO37 & ~Z180_IO37_WMASK) | (data & Z180_IO37_WMASK);
			break;

		case Z180_CBR:
			LOG("Z180 '%s' CBR    wr $%02x ($%02x)\n", cpustate->device->m_tag, data,  data & Z180_CBR_WMASK);
			cpustate->IO_CBR = (cpustate->IO_CBR & ~Z180_CBR_WMASK) | (data & Z180_CBR_WMASK);
			z180_mmu(cpustate);
			break;

		case Z180_BBR:
			LOG("Z180 '%s' BBR    wr $%02x ($%02x)\n", cpustate->device->m_tag, data,  data & Z180_BBR_WMASK);
			cpustate->IO_BBR = (cpustate->IO_BBR & ~Z180_BBR_WMASK) | (data & Z180_BBR_WMASK);
			z180_mmu(cpustate);
			break;

		case Z180_CBAR:
			LOG("Z180 '%s' CBAR   wr $%02x ($%02x)\n", cpustate->device->m_tag, data,  data & Z180_CBAR_WMASK);
			cpustate->IO_CBAR = (cpustate->IO_CBAR & ~Z180_CBAR_WMASK) | (data & Z180_CBAR_WMASK);
			z180_mmu(cpustate);
			break;

		case Z180_IO3B:
			LOG("Z180 '%s' IO3B   wr $%02x ($%02x)\n", cpustate->device->m_tag, data,  data & Z180_IO3B_WMASK);
			cpustate->IO_IO3B = (cpustate->IO_IO3B & ~Z180_IO3B_WMASK) | (data & Z180_IO3B_WMASK);
			break;

		case Z180_IO3C:
			LOG("Z180 '%s' IO3C   wr $%02x ($%02x)\n", cpustate->device->m_tag, data,  data & Z180_IO3C_WMASK);
			cpustate->IO_IO3C = (cpustate->IO_IO3C & ~Z180_IO3C_WMASK) | (data & Z180_IO3C_WMASK);
			break;

		case Z180_IO3D:
			LOG("Z180 '%s' IO3D   wr $%02x ($%02x)\n", cpustate->device->m_tag, data,  data & Z180_IO3D_WMASK);
			cpustate->IO_IO3D = (cpustate->IO_IO3D & ~Z180_IO3D_WMASK) | (data & Z180_IO3D_WMASK);
			break;

		case Z180_OMCR:
			LOG("Z180 '%s' OMCR   wr $%02x ($%02x)\n", cpustate->device->m_tag, data,  data & Z180_OMCR_WMASK);
			cpustate->IO_OMCR = (cpustate->IO_OMCR & ~Z180_OMCR_WMASK) | (data & Z180_OMCR_WMASK);
			break;

		case Z180_IOCR:
			LOG("Z180 '%s' IOCR   wr $%02x ($%02x)\n", cpustate->device->m_tag, data,  data & Z180_IOCR_WMASK);
			if (cpustate->device->m_type == Z180_TYPE_Z182)
				if ((data & 0xc0) == 0xc0)  /* b11... reserved */
					data &= ~0xc0;
			cpustate->IO_IOCR = (cpustate->IO_IOCR & ~Z180_IOCR_WMASK) | (data & Z180_IOCR_WMASK);
			break;

		default:
			LOG("Z180 bogus write control reg %04X:$%02X\n", cpustate->device->m_tag, port, data);
			break;
		}
	}
	else
		LOG("Z180 unimplemented write control reg: %04X:%04X \n",port,data );

}

int z180_dma0(struct z180_state *cpustate, int max_cycles)
{
	if (!(cpustate->IO_DSTAT & Z180_DSTAT_DE0))
	{
		return 0;
	} 

	offs_t sar0 = 65536 * cpustate->IO_SAR0B + 256 * cpustate->IO_SAR0H + cpustate->IO_SAR0L;
	offs_t dar0 = 65536 * cpustate->IO_DAR0B + 256 * cpustate->IO_DAR0H + cpustate->IO_DAR0L;
	int bcr0 = 256 * cpustate->IO_BCR0H + cpustate->IO_BCR0L;

	if (bcr0 == 0)
	{
		bcr0 = 0x10000;
	}

	int count = (cpustate->IO_DMODE & Z180_DMODE_MMOD) ? bcr0 : 1;
	int cycles = 0;

	LOG("z180 DMA0 %d %d\n",bcr0,count);
	while (count > 0)
	{
		/* last transfer happening now? */
		if (bcr0 == 1)
		{
			cpustate->iol |= Z180_TEND0;
		}
		switch( cpustate->IO_DMODE & (Z180_DMODE_SM | Z180_DMODE_DM) )
		{
		case 0x00:  /* memory SAR0+1 to memory DAR0+1 */
			cpustate->memory->write_byte(cpustate, dar0++, cpustate->memory->read_byte(cpustate, sar0++));
			cycles += (cpustate->IO_DCNTL >> 6) * 2; // memory wait states
			bcr0--; 
			break;
		case 0x04:  /* memory SAR0-1 to memory DAR0+1 */
			cpustate->memory->write_byte(cpustate, dar0++, cpustate->memory->read_byte(cpustate, sar0--));
			cycles += (cpustate->IO_DCNTL >> 6) * 2; // memory wait states
			bcr0--; 
			break;
		case 0x08:  /* memory SAR0 fixed to memory DAR0+1 */
			cpustate->memory->write_byte(cpustate, dar0++, cpustate->memory->read_byte(cpustate, sar0));
			cycles += (cpustate->IO_DCNTL >> 6) * 2; // memory wait states
			bcr0--; 
			break;
		case 0x0c:  /* I/O SAR0 fixed to memory DAR0+1 */
			if (cpustate->iol & Z180_DREQ0)
			{
				cpustate->memory->write_byte(cpustate, dar0++, IN(cpustate, sar0));
				cycles += (cpustate->IO_DCNTL >> 6) * 2; // memory wait states
				bcr0--; 
				/* edge sensitive DREQ0 ? */
				if (cpustate->IO_DCNTL & Z180_DCNTL_DIM0)
				{
					cpustate->iol &= ~Z180_DREQ0;
					count = 0;
				}
			}
			break;
		case 0x10:  /* memory SAR0+1 to memory DAR0-1 */
			cpustate->memory->write_byte(cpustate, dar0--, cpustate->memory->read_byte(cpustate, sar0++));
			cycles += (cpustate->IO_DCNTL >> 6) * 2; // memory wait states
			bcr0--; 
			break;
		case 0x14:  /* memory SAR0-1 to memory DAR0-1 */
			cpustate->memory->write_byte(cpustate, dar0--, cpustate->memory->read_byte(cpustate, sar0--));
			cycles += (cpustate->IO_DCNTL >> 6) * 2; // memory wait states
			bcr0--; 
			break;
		case 0x18:  /* memory SAR0 fixed to memory DAR0-1 */
			cpustate->memory->write_byte(cpustate, dar0--, cpustate->memory->read_byte(cpustate, sar0));
			cycles += (cpustate->IO_DCNTL >> 6) * 2; // memory wait states
			bcr0--; 
			break;
		case 0x1c:  /* I/O SAR0 fixed to memory DAR0-1 */
			if (cpustate->iol & Z180_DREQ0)
			{
				cpustate->memory->write_byte(cpustate, dar0--, IN(cpustate, sar0));
				cycles += (cpustate->IO_DCNTL >> 6) * 2; // memory wait states
				bcr0--; 
				/* edge sensitive DREQ0 ? */
				if (cpustate->IO_DCNTL & Z180_DCNTL_DIM0)
				{
					cpustate->iol &= ~Z180_DREQ0;
					count = 0;
				}
			}
			break;
		case 0x20:  /* memory SAR0+1 to memory DAR0 fixed */
			cpustate->memory->write_byte(cpustate, dar0, cpustate->memory->read_byte(cpustate, sar0++));
			cycles += (cpustate->IO_DCNTL >> 6) * 2; // memory wait states
			bcr0--; 
			break;
		case 0x24:  /* memory SAR0-1 to memory DAR0 fixed */
			cpustate->memory->write_byte(cpustate, dar0, cpustate->memory->read_byte(cpustate, sar0--));
			cycles += (cpustate->IO_DCNTL >> 6) * 2; // memory wait states
			bcr0--; 
			break;
		case 0x28:  /* reserved */
			break;
		case 0x2c:  /* reserved */
			break;
		case 0x30:  /* memory SAR0+1 to I/O DAR0 fixed */
			if (cpustate->iol & Z180_DREQ0)
			{
				OUT(cpustate, dar0, cpustate->memory->read_byte(cpustate, sar0++));
				cycles += (cpustate->IO_DCNTL >> 6) * 2; // memory wait states
				bcr0--; 
				/* edge sensitive DREQ0 ? */
				if (cpustate->IO_DCNTL & Z180_DCNTL_DIM0)
				{
					cpustate->iol &= ~Z180_DREQ0;
					count = 0;
				}
			}
			break;
		case 0x34:  /* memory SAR0-1 to I/O DAR0 fixed */
			if (cpustate->iol & Z180_DREQ0)
			{
				OUT(cpustate, dar0, cpustate->memory->read_byte(cpustate, sar0--));
				cycles += (cpustate->IO_DCNTL >> 6) * 2; // memory wait states
				bcr0--; 
				/* edge sensitive DREQ0 ? */
				if (cpustate->IO_DCNTL & Z180_DCNTL_DIM0)
				{
					cpustate->iol &= ~Z180_DREQ0;
					count = 0;
				}
			}
			break;
		case 0x38:  /* reserved */
			break;
		case 0x3c:  /* reserved */
			break;
		}
		count--;
		cycles += 6;
		if (cycles > max_cycles)
			break;
	}

	cpustate->IO_SAR0L = sar0;
	cpustate->IO_SAR0H = sar0 >> 8;
	cpustate->IO_SAR0B = sar0 >> 16;
	cpustate->IO_DAR0L = dar0;
	cpustate->IO_DAR0H = dar0 >> 8;
	cpustate->IO_DAR0B = dar0 >> 16;
	cpustate->IO_BCR0L = bcr0;
	cpustate->IO_BCR0H = bcr0 >> 8;

	/* DMA terminal count? */
	if (bcr0 == 0)
	{
		cpustate->iol &= ~Z180_TEND0;
		cpustate->IO_DSTAT &= ~Z180_DSTAT_DE0;
		/* terminal count interrupt enabled? */
		if (cpustate->IO_DSTAT & Z180_DSTAT_DIE0 && cpustate->IFF1)
			cpustate->int_pending[Z180_INT_DMA0] = 1;
	}
	return cycles;
}

int z180_dma1(struct z180_state *cpustate)
{
	if (!(cpustate->IO_DSTAT & Z180_DSTAT_DE1))
	{
		return 0;
	} 

	offs_t mar1 = 65536 * cpustate->IO_MAR1B + 256 * cpustate->IO_MAR1H + cpustate->IO_MAR1L;
	offs_t iar1 = 256 * cpustate->IO_IAR1H + cpustate->IO_IAR1L;
	int bcr1 = 256 * cpustate->IO_BCR1H + cpustate->IO_BCR1L;

	if (bcr1 == 0)
	{
		bcr1 = 0x10000;
	}

	int cycles = 0;

	if ((cpustate->iol & Z180_DREQ1) == 0)
		return 0;

	/* last transfer happening now? */
	if (bcr1 == 1)
	{
		cpustate->iol |= Z180_TEND1;
	}

	switch (cpustate->IO_DCNTL & (Z180_DCNTL_DIM1 | Z180_DCNTL_DIM0))
	{
	case 0x00:  /* memory MAR1+1 to I/O IAR1 fixed */
		cpustate->iospace->write_byte(iar1, cpustate->memory->read_byte(cpustate, mar1++));
		break;
	case 0x01:  /* memory MAR1-1 to I/O IAR1 fixed */
		cpustate->iospace->write_byte(iar1, cpustate->memory->read_byte(cpustate, mar1--));
		break;
	case 0x02:  /* I/O IAR1 fixed to memory MAR1+1 */
		cpustate->memory->write_byte(cpustate, mar1++, cpustate->iospace->read_byte(iar1));
		break;
	case 0x03:  /* I/O IAR1 fixed to memory MAR1-1 */
		cpustate->memory->write_byte(cpustate, mar1--, cpustate->iospace->read_byte(iar1));
		break;
	}
	bcr1--;

	cycles += cpustate->IO_DCNTL >> 6; // memory wait states 

	/* edge sensitive DREQ1 ? */
	if (cpustate->IO_DCNTL & Z180_DCNTL_DIM1)
		cpustate->iol &= ~Z180_DREQ1;

	cpustate->IO_MAR1L = mar1;
	cpustate->IO_MAR1H = mar1 >> 8;
	cpustate->IO_MAR1B = mar1 >> 16;
	cpustate->IO_BCR1L = bcr1;
	cpustate->IO_BCR1H = bcr1 >> 8;

	/* DMA terminal count? */
	if (bcr1 == 0)
	{
		cpustate->iol &= ~Z180_TEND1;
		cpustate->IO_DSTAT &= ~Z180_DSTAT_DE1;
		if (cpustate->IO_DSTAT & Z180_DSTAT_DIE1 && cpustate->IFF1)
			cpustate->int_pending[Z180_INT_DMA1] = 1;
	}

	/* six cycles per transfer (minimum) */
	return 6 + cycles;
}

void set_irq_line(struct z180_state *cpustate, int irqline, int state);

void z180_set_irq_line(device_t *device, int irqline, int state) {
	struct z180_state *cpustate = get_safe_token(device);
	set_irq_line(cpustate,irqline,state);
}

void z180_set_asci_irq(device_t *device, int channel, int state) {
	struct z180_state *cpustate = get_safe_token(device);
	if (channel==0)
		cpustate->int_pending[Z180_INT_ASCI0] = state;
	else
		cpustate->int_pending[Z180_INT_ASCI1] = state;
}

void z180_set_dreq0(device_t *device, int state) {
	struct z180_state *cpustate = get_safe_token(device);
	if (state)
		cpustate->iol |= Z180_DREQ0;
	else
		cpustate->iol &= ~Z180_DREQ0;
}

void z180_set_dreq1(device_t *device, int state) {
	struct z180_state *cpustate = get_safe_token(device);
	if (state)
		cpustate->iol |= Z180_DREQ1;
	else
		cpustate->iol &= ~Z180_DREQ1;
}

int z180_get_tend0(device_t *device) {
	struct z180_state *cpustate = get_safe_token(device);
	return cpustate->iol & Z180_TEND0;
}

int z180_get_tend1(device_t *device) {
	struct z180_state *cpustate = get_safe_token(device);
	return cpustate->iol & Z180_TEND1;
}


/*void z180_write_iolines(struct z180_state *cpustate, UINT32 data)
{
	UINT32 changes = cpustate->iol ^ data;

	/ I/O asynchronous clock 0 (active high) or DREQ0 (mux) /
	if (changes & Z180_CKA0)
	{
		LOG("Z180 '%s' CKA0   %d\n", cpustate->device->m_tag, data & Z180_CKA0 ? 1 : 0);
		cpustate->iol = (cpustate->iol & ~Z180_CKA0) | (data & Z180_CKA0);
	}

	/ I/O asynchronous clock 1 (active high) or TEND1 (mux) /
	if (changes & Z180_CKA1)
	{
		LOG("Z180 '%s' CKA1   %d\n", cpustate->device->m_tag, data & Z180_CKA1 ? 1 : 0);
		cpustate->iol = (cpustate->iol & ~Z180_CKA1) | (data & Z180_CKA1);
	}

	/ I/O serial clock (active high) /
	if (changes & Z180_CKS)
	{
		LOG("Z180 '%s' CKS    %d\n", cpustate->device->m_tag, data & Z180_CKS ? 1 : 0);
		cpustate->iol = (cpustate->iol & ~Z180_CKS) | (data & Z180_CKS);
	}

	/ I   clear to send 0 (active low) /
	if (changes & Z180_CTS0)
	{
		LOG("Z180 '%s' CTS0   %d\n", cpustate->device->m_tag, data & Z180_CTS0 ? 1 : 0);
		cpustate->iol = (cpustate->iol & ~Z180_CTS0) | (data & Z180_CTS0);
	}

	/ I   clear to send 1 (active low) or RXS (mux) /
	if (changes & Z180_CTS1)
	{
		LOG("Z180 '%s' CTS1   %d\n", cpustate->device->m_tag, data & Z180_CTS1 ? 1 : 0);
		cpustate->iol = (cpustate->iol & ~Z180_CTS1) | (data & Z180_CTS1);
	}

	/ I   data carrier detect (active low) /
	if (changes & Z180_DCD0)
	{
		LOG("Z180 '%s' DCD0   %d\n", cpustate->device->m_tag, data & Z180_DCD0 ? 1 : 0);
		cpustate->iol = (cpustate->iol & ~Z180_DCD0) | (data & Z180_DCD0);
	}

	/ I   data request DMA ch 0 (active low) or CKA0 (mux) /
	if (changes & Z180_DREQ0)
	{
		LOG("Z180 '%s' DREQ0  %d\n", cpustate->device->m_tag, data & Z180_DREQ0 ? 1 : 0);
		cpustate->iol = (cpustate->iol & ~Z180_DREQ0) | (data & Z180_DREQ0);
	}

	/ I   data request DMA ch 1 (active low) /
	if (changes & Z180_DREQ1)
	{
		LOG("Z180 '%s' DREQ1  %d\n", cpustate->device->m_tag, data & Z180_DREQ1 ? 1 : 0);
		cpustate->iol = (cpustate->iol & ~Z180_DREQ1) | (data & Z180_DREQ1);
	}

	/ I   asynchronous receive data 0 (active high) /
	if (changes & Z180_RXA0)
	{
		LOG("Z180 '%s' RXA0   %d\n", cpustate->device->m_tag, data & Z180_RXA0 ? 1 : 0);
		cpustate->iol = (cpustate->iol & ~Z180_RXA0) | (data & Z180_RXA0);
	}

	/ I   asynchronous receive data 1 (active high) /
	if (changes & Z180_RXA1)
	{
		LOG("Z180 '%s' RXA1   %d\n", cpustate->device->m_tag, data & Z180_RXA1 ? 1 : 0);
		cpustate->iol = (cpustate->iol & ~Z180_RXA1) | (data & Z180_RXA1);
	}

	/ I   clocked serial receive data (active high) or CTS1 (mux) /
	if (changes & Z180_RXS)
	{
		LOG("Z180 '%s' RXS    %d\n", cpustate->device->m_tag, data & Z180_RXS ? 1 : 0);
		cpustate->iol = (cpustate->iol & ~Z180_RXS) | (data & Z180_RXS);
	}

	/   O request to send (active low) /
	if (changes & Z180_RTS0)
	{
		LOG("Z180 '%s' RTS0   won't change output\n", cpustate->device->m_tag);
	}

	/   O transfer end 0 (active low) or CKA1 (mux) /
	if (changes & Z180_TEND0)
	{
		LOG("Z180 '%s' TEND0  won't change output\n", cpustate->device->m_tag);
	}

	/   O transfer end 1 (active low) /
	if (changes & Z180_TEND1)
	{
		LOG("Z180 '%s' TEND1  won't change output\n", cpustate->device->m_tag);
	}

	/   O transfer out (PRT channel, active low) or A18 (mux) /
	if (changes & Z180_A18_TOUT)
	{
		LOG("Z180 '%s' TOUT   won't change output\n", cpustate->device->m_tag);
	}

	/   O asynchronous transmit data 0 (active high) /
	if (changes & Z180_TXA0)
	{
		LOG("Z180 '%s' TXA0   won't change output\n", cpustate->device->m_tag);
	}

	/   O asynchronous transmit data 1 (active high) /
	if (changes & Z180_TXA1)
	{
		LOG("Z180 '%s' TXA1   won't change output\n", cpustate->device->m_tag);
	}

	/   O clocked serial transmit data (active high) /
	if (changes & Z180_TXS)
	{
		LOG("Z180 '%s' TXS    won't change output\n", cpustate->device->m_tag);
	}
}*/

UINT8 ram_read_byte(struct z180_state *cpustate, offs_t byteaddress) {
	return cpustate->ram->read_byte(byteaddress);
}

void ram_write_byte(struct z180_state *cpustate, offs_t byteaddress, UINT8 data) {
	return cpustate->ram->write_byte(byteaddress,data);
}

UINT8 ram_read_raw_byte(struct z180_state *cpustate, offs_t byteaddress) {
	return cpustate->ram->read_raw_byte(byteaddress);
}


UINT8 memcs_read_byte(struct z180_state *cpustate, offs_t byteaddress) {
	if(!(cpustate->IO_SCR & 8) && (byteaddress >>12) <= cpustate->IO_ROMBR)
		return cpustate->rom->read_byte(byteaddress);
	if( cpustate->IO_RAMLBR <= (byteaddress >>12) && (byteaddress >>12) <= cpustate->IO_RAMUBR)
		return cpustate->ram->read_byte(byteaddress);
	return 0; /* undocumented, DBUS floating?? */
}

void memcs_write_byte(struct z180_state *cpustate, offs_t byteaddress, UINT8 data) {
	if(!(cpustate->IO_SCR & 8) && (byteaddress >>12) <= cpustate->IO_ROMBR)
		LOG("Z180: Write to ROM %05x\n",byteaddress);
	if( cpustate->IO_RAMLBR <= (byteaddress >>12) && (byteaddress >>12) <= cpustate->IO_RAMUBR)
		cpustate->ram->write_byte(byteaddress, data);
}

UINT8 memcs_read_raw_byte(struct z180_state *cpustate, offs_t byteaddress) {
	if(!(cpustate->IO_SCR & 8) && (byteaddress >>12) <= cpustate->IO_ROMBR)
		return cpustate->rom->read_raw_byte(byteaddress);
	if( cpustate->IO_RAMLBR <= (byteaddress >>12) && (byteaddress >>12) <= cpustate->IO_RAMUBR)
		return cpustate->ram->read_raw_byte(byteaddress);
	return 0; /* undocumented, DBUS floating?? */
}

struct memory_select alwaysram = {
	ram_read_byte,
	ram_write_byte,
	ram_read_raw_byte
};

struct memory_select memcs = {
	memcs_read_byte,
	memcs_write_byte,
	memcs_read_raw_byte
};

void z80scc_out_int_cb(device_t *device, int state) {
	z180_set_irq_line((struct z180_device *)((struct z80scc_device *)device)->m_owner, 0, state);
}

struct z180_device *cpu_create_z180(char *tag, UINT32 type, UINT32 clock, 
    struct address_space *ram, struct address_space *rom /* only on Z182 */, 
	struct address_space *iospace, device_irq_acknowledge_callback irqcallback, struct z80daisy_interface *daisy_init,
	rx_callback_t z180asci_rx_cb,tx_callback_t z180asci_tx_cb,
        rx_callback_t z180csi_rx_cb, tx_callback_t z180csi_tx_cb,
	rx_callback_t z80scc_rx_cb,tx_callback_t z80scc_tx_cb /* only on Z182 */,
	parport_read_callback_t parport_read_cb, parport_write_callback_t parport_write_cb /* only on Z182 */)
{
	struct z180_device *d = malloc(sizeof(struct z180_device));
	memset(d,0,sizeof(struct z180_device));
	d->m_type = type;
	d->m_clock = clock;
	d->m_tag = tag;

	struct z180_state *cpustate = malloc(sizeof(struct z180_state));
	d->m_token = cpustate;
	memset(cpustate,0,sizeof(struct z180_state));
	cpustate->device = d;

	if (type == Z180_TYPE_Z182)
		cpustate->memory = &memcs;
	else
		cpustate->memory = &alwaysram;
	cpustate->ram = ram;
	cpustate->rom = rom;
	cpustate->iospace = iospace;

	//struct z180_state *cpustate = get_safe_token(device);
	//if (device->static_config() != NULL)
	//	cpustate->daisy.init(device, (const z80_daisy_config *)device->static_config());
	if (daisy_init != NULL)
		cpustate->daisy = z80_daisy_chain_create(d,daisy_init); // allocate head and build chain pointers
	cpustate->irq_callback = irqcallback;

	if (type == Z180_TYPE_Z182)	{ // setup 85230 ESCC
		d->z80scc_tag = malloc(20);
		strcpy(d->z80scc_tag,tag);
		strcat(d->z80scc_tag,"ESCC");
		d->z80scc = z80scc_device_create(d,d->z80scc_tag,TYPE_SCC85230,clock,
			z80scc_out_int_cb,z80scc_rx_cb, z80scc_tx_cb);
		d->m_parport_read_cb = parport_read_cb;
		d->m_parport_write_cb = parport_write_cb;
	}

	d->z180asci_tag = malloc(20);
	strcpy(d->z180asci_tag,tag);
	strcat(d->z180asci_tag,"ASCI");
	d->z180asci = z180asci_device_create(d,d->z180asci_tag,clock,
			z180asci_rx_cb, z180asci_tx_cb);

        d->z180csi_rx_cb = z180csi_rx_cb;
        d->z180csi_tx_cb = z180csi_tx_cb;

	SZHVC_add = malloc(2*256*256);
	SZHVC_sub = malloc(2*256*256);

	/* set up the state table */
	/*{
		device_state_interface *state;
		device->interface(state);
		state->state_add(Z180_PC,         "PC",        cpustate->PC.w.l);
		state->state_add(STATE_GENPC,     "GENPC",     cpustate->_PCD).noshow();
		state->state_add(STATE_GENPCBASE, "GENPCBASE", cpustate->PREPC.w.l).noshow();
		state->state_add(Z180_SP,         "SP",        cpustate->_SPD);
		state->state_add(STATE_GENSP,     "GENSP",     cpustate->SP.w.l).noshow();
		state->state_add(STATE_GENFLAGS,  "GENFLAGS",  cpustate->AF.b.l).noshow().formatstr("%8s");
		state->state_add(Z180_A,          "A",         cpustate->_A).noshow();
		state->state_add(Z180_B,          "B",         cpustate->_B).noshow();
		state->state_add(Z180_C,          "C",         cpustate->_C).noshow();
		state->state_add(Z180_D,          "D",         cpustate->_D).noshow();
		state->state_add(Z180_E,          "E",         cpustate->_E).noshow();
		state->state_add(Z180_H,          "H",         cpustate->_H).noshow();
		state->state_add(Z180_L,          "L",         cpustate->_L).noshow();
		state->state_add(Z180_AF,         "AF",        cpustate->AF.w.l);
		state->state_add(Z180_BC,         "BC",        cpustate->BC.w.l);
		state->state_add(Z180_DE,         "DE",        cpustate->DE.w.l);
		state->state_add(Z180_HL,         "HL",        cpustate->HL.w.l);
		state->state_add(Z180_IX,         "IX",        cpustate->IX.w.l);
		state->state_add(Z180_IY,         "IY",        cpustate->IY.w.l);
		state->state_add(Z180_AF2,        "AF2",       cpustate->AF2.w.l);
		state->state_add(Z180_BC2,        "BC2",       cpustate->BC2.w.l);
		state->state_add(Z180_DE2,        "DE2",       cpustate->DE2.w.l);
		state->state_add(Z180_HL2,        "HL2",       cpustate->HL2.w.l);
		state->state_add(Z180_R,          "R",         cpustate->rtemp).callimport().callexport();
		state->state_add(Z180_I,          "I",         cpustate->I);
		state->state_add(Z180_IM,         "IM",        cpustate->IM).mask(0x3);
		state->state_add(Z180_IFF1,       "IFF1",      cpustate->IFF1).mask(0x1);
		state->state_add(Z180_IFF2,       "IFF2",      cpustate->IFF2).mask(0x1);
		state->state_add(Z180_HALT,       "HALT",      cpustate->HALT).mask(0x1);

		state->state_add(Z180_IOLINES,    "IOLINES",   cpustate->ioltemp).mask(0xffffff).callimport();

		state->state_add(Z180_CNTLA0,     "CNTLA0",    cpustate->IO_CNTLA0);
		state->state_add(Z180_CNTLA1,     "CNTLA1",    cpustate->IO_CNTLA1);
		state->state_add(Z180_CNTLB0,     "CNTLB0",    cpustate->IO_CNTLB0);
		state->state_add(Z180_CNTLB1,     "CNTLB1",    cpustate->IO_CNTLB1);
		state->state_add(Z180_STAT0,      "STAT0",     cpustate->IO_STAT0);
		state->state_add(Z180_STAT1,      "STAT1",     cpustate->IO_STAT1);
		state->state_add(Z180_TDR0,       "TDR0",      cpustate->IO_TDR0);
		state->state_add(Z180_TDR1,       "TDR1",      cpustate->IO_TDR1);
		state->state_add(Z180_RDR0,       "RDR0",      cpustate->IO_RDR0);
		state->state_add(Z180_RDR1,       "RDR1",      cpustate->IO_RDR1);
		state->state_add(Z180_CNTR,       "CNTR",      cpustate->IO_CNTR);
		state->state_add(Z180_TRDR,       "TRDR",      cpustate->IO_TRDR);
		state->state_add(Z180_TMDR0L,     "TMDR0L",    cpustate->IO_TMDR0L);
		state->state_add(Z180_TMDR0H,     "TMDR0H",    cpustate->IO_TMDR0H);
		state->state_add(Z180_RLDR0L,     "RLDR0L",    cpustate->IO_RLDR0L);
		state->state_add(Z180_RLDR0H,     "RLDR0H",    cpustate->IO_RLDR0H);
		state->state_add(Z180_TCR,        "TCR",       cpustate->IO_TCR);
		state->state_add(Z180_IO11,       "IO11",      cpustate->IO_IO11);
		state->state_add(Z180_ASEXT0,     "ASEXT0",    cpustate->IO_ASEXT0);
		state->state_add(Z180_ASEXT1,     "ASEXT1",    cpustate->IO_ASEXT1);
		state->state_add(Z180_TMDR1L,     "TMDR1L",    cpustate->IO_TMDR1L);
		state->state_add(Z180_TMDR1H,     "TMDR1H",    cpustate->IO_TMDR1H);
		state->state_add(Z180_RLDR1L,     "RLDR1L",    cpustate->IO_RLDR1L);
		state->state_add(Z180_RLDR1H,     "RLDR1H",    cpustate->IO_RLDR1H);
		state->state_add(Z180_FRC,        "FRC",       cpustate->IO_FRC);
		state->state_add(Z180_IO19,       "IO19",      cpustate->IO_IO19);
		state->state_add(Z180_ASTC0L,     "ASTC0L",    cpustate->IO_ASTC0L);
		state->state_add(Z180_ASTC0H,     "ASTC0H",    cpustate->IO_ASTC0H);
		state->state_add(Z180_ASTC1L,     "ASTC1L",    cpustate->IO_ASTC1L);
		state->state_add(Z180_ASTC1H,     "ASTC1H",    cpustate->IO_ASTC1H);
		state->state_add(Z180_CMR,        "CMR",       cpustate->IO_CMR);
		state->state_add(Z180_CCR,        "CCR",       cpustate->IO_CCR);
		state->state_add(Z180_SAR0L,      "SAR0L",     cpustate->IO_SAR0L);
		state->state_add(Z180_SAR0H,      "SAR0H",     cpustate->IO_SAR0H);
		state->state_add(Z180_SAR0B,      "SAR0B",     cpustate->IO_SAR0B);
		state->state_add(Z180_DAR0L,      "DAR0L",     cpustate->IO_DAR0L);
		state->state_add(Z180_DAR0H,      "DAR0H",     cpustate->IO_DAR0H);
		state->state_add(Z180_DAR0B,      "DAR0B",     cpustate->IO_DAR0B);
		state->state_add(Z180_BCR0L,      "BCR0L",     cpustate->IO_BCR0L);
		state->state_add(Z180_BCR0H,      "BCR0H",     cpustate->IO_BCR0H);
		state->state_add(Z180_MAR1L,      "MAR1L",     cpustate->IO_MAR1L);
		state->state_add(Z180_MAR1H,      "MAR1H",     cpustate->IO_MAR1H);
		state->state_add(Z180_MAR1B,      "MAR1B",     cpustate->IO_MAR1B);
		state->state_add(Z180_IAR1L,      "IAR1L",     cpustate->IO_IAR1L);
		state->state_add(Z180_IAR1H,      "IAR1H",     cpustate->IO_IAR1H);
		state->state_add(Z180_IAR1B,      "IAR1B",     cpustate->IO_IAR1B);
		state->state_add(Z180_BCR1L,      "BCR1L",     cpustate->IO_BCR1L);
		state->state_add(Z180_BCR1H,      "BCR1H",     cpustate->IO_BCR1H);
		state->state_add(Z180_DSTAT,      "DSTAT",     cpustate->IO_DSTAT);
		state->state_add(Z180_DMODE,      "DMODE",     cpustate->IO_DMODE);
		state->state_add(Z180_DCNTL,      "DCNTL",     cpustate->IO_DCNTL);
		state->state_add(Z180_IL,         "IL",        cpustate->IO_IL);
		state->state_add(Z180_ITC,        "ITC",       cpustate->IO_ITC);
		state->state_add(Z180_IO35,       "IO35",      cpustate->IO_IO35);
		state->state_add(Z180_RCR,        "RCR",       cpustate->IO_RCR);
		state->state_add(Z180_IO37,       "IO37",      cpustate->IO_IO37);
		state->state_add(Z180_CBR,        "CBR",       cpustate->IO_CBR).callimport();
		state->state_add(Z180_BBR,        "BBR",       cpustate->IO_BBR).callimport();
		state->state_add(Z180_CBAR,       "CBAR",      cpustate->IO_CBAR).callimport();
		state->state_add(Z180_IO3B,       "IO3B",      cpustate->IO_IO3B);
		state->state_add(Z180_IO3C,       "IO3C",      cpustate->IO_IO3C);
		state->state_add(Z180_IO3D,       "IO3D",      cpustate->IO_IO3D);
		state->state_add(Z180_OMCR,       "OMCR",      cpustate->IO_OMCR);
		state->state_add(Z180_IOCR,       "IOCR",      cpustate->IO_IOCR);
	}

	device->save_item(NAME(cpustate->AF.w.l));
	device->save_item(NAME(cpustate->BC.w.l));
	device->save_item(NAME(cpustate->DE.w.l));
	device->save_item(NAME(cpustate->HL.w.l));
	device->save_item(NAME(cpustate->IX.w.l));
	device->save_item(NAME(cpustate->IY.w.l));
	device->save_item(NAME(cpustate->PC.w.l));
	device->save_item(NAME(cpustate->SP.w.l));
	device->save_item(NAME(cpustate->AF2.w.l));
	device->save_item(NAME(cpustate->BC2.w.l));
	device->save_item(NAME(cpustate->DE2.w.l));
	device->save_item(NAME(cpustate->HL2.w.l));
	device->save_item(NAME(cpustate->R));
	device->save_item(NAME(cpustate->R2));
	device->save_item(NAME(cpustate->IFF1));
	device->save_item(NAME(cpustate->IFF2));
	device->save_item(NAME(cpustate->HALT));
	device->save_item(NAME(cpustate->IM));
	device->save_item(NAME(cpustate->I));
	device->save_item(NAME(cpustate->nmi_state));
	device->save_item(NAME(cpustate->nmi_pending));
	device->save_item(NAME(cpustate->irq_state));
	device->save_item(NAME(cpustate->int_pending));
	device->save_item(NAME(cpustate->timer_cnt));
	device->save_item(NAME(cpustate->dma0_cnt));
	device->save_item(NAME(cpustate->dma1_cnt));
	device->save_item(NAME(cpustate->after_EI));

	device->save_item(NAME(cpustate->tif));

	device->save_item(NAME(cpustate->read_tcr_tmdr));
	device->save_item(NAME(cpustate->tmdr_value));
	device->save_item(NAME(cpustate->tmdrh));
	device->save_item(NAME(cpustate->tmdr_latch));

	device->save_item(NAME(cpustate->io));
	device->save_item(NAME(cpustate->iol));
	device->save_item(NAME(cpustate->ioltemp));

	device->save_item(NAME(cpustate->mmu));
	*/
	return d;
}

/****************************************************************************
 * Reset registers to their initial values
 ****************************************************************************/

void cpu_reset_z180(device_t *device)
{
	struct z180_state *cpustate = get_safe_token(device);
	int i, p;
	int oldval, newval, val;
	UINT8 *padd, *padc, *psub, *psbc;
	/* allocate big flag arrays once */
	padd = &SZHVC_add[  0*256];
	padc = &SZHVC_add[256*256];
	psub = &SZHVC_sub[  0*256];
	psbc = &SZHVC_sub[256*256];
	for (oldval = 0; oldval < 256; oldval++)
	{
		for (newval = 0; newval < 256; newval++)
		{
			/* add or adc w/o carry set */
			val = newval - oldval;
			*padd = (newval) ? ((newval & 0x80) ? SF : 0) : ZF;
			*padd |= (newval & (YF | XF));  /* undocumented flag bits 5+3 */

			if( (newval & 0x0f) < (oldval & 0x0f) ) *padd |= HF;
			if( newval < oldval ) *padd |= CF;
			if( (val^oldval^0x80) & (val^newval) & 0x80 ) *padd |= VF;
			padd++;

			/* adc with carry set */
			val = newval - oldval - 1;
			*padc = (newval) ? ((newval & 0x80) ? SF : 0) : ZF;
			*padc |= (newval & (YF | XF));  /* undocumented flag bits 5+3 */
			if( (newval & 0x0f) <= (oldval & 0x0f) ) *padc |= HF;
			if( newval <= oldval ) *padc |= CF;
			if( (val^oldval^0x80) & (val^newval) & 0x80 ) *padc |= VF;
			padc++;

			/* cp, sub or sbc w/o carry set */
			val = oldval - newval;
			*psub = NF | ((newval) ? ((newval & 0x80) ? SF : 0) : ZF);
			*psub |= (newval & (YF | XF));  /* undocumented flag bits 5+3 */
			if( (newval & 0x0f) > (oldval & 0x0f) ) *psub |= HF;
			if( newval > oldval ) *psub |= CF;
			if( (val^oldval) & (oldval^newval) & 0x80 ) *psub |= VF;
			psub++;

			/* sbc with carry set */
			val = oldval - newval - 1;
			*psbc = NF | ((newval) ? ((newval & 0x80) ? SF : 0) : ZF);
			*psbc |= (newval & (YF | XF));  /* undocumented flag bits 5+3 */
			if( (newval & 0x0f) >= (oldval & 0x0f) ) *psbc |= HF;
			if( newval >= oldval ) *psbc |= CF;
			if( (val^oldval) & (oldval^newval) & 0x80 ) *psbc |= VF;
			psbc++;
		}
	}
	for (i = 0; i < 256; i++)
	{
		p = 0;
		if( i&0x01 ) ++p;
		if( i&0x02 ) ++p;
		if( i&0x04 ) ++p;
		if( i&0x08 ) ++p;
		if( i&0x10 ) ++p;
		if( i&0x20 ) ++p;
		if( i&0x40 ) ++p;
		if( i&0x80 ) ++p;
		SZ[i] = i ? i & SF : ZF;
		SZ[i] |= (i & (YF | XF));       /* undocumented flag bits 5+3 */
		SZ_BIT[i] = i ? i & SF : ZF | PF;
		SZ_BIT[i] |= (i & (YF | XF));   /* undocumented flag bits 5+3 */
		SZP[i] = SZ[i] | ((p & 1) ? 0 : PF);
		SZHV_inc[i] = SZ[i];
		if( i == 0x80 ) SZHV_inc[i] |= VF;
		if( (i & 0x0f) == 0x00 ) SZHV_inc[i] |= HF;
		SZHV_dec[i] = SZ[i] | NF;
		if( i == 0x7f ) SZHV_dec[i] |= VF;
		if( (i & 0x0f) == 0x0f ) SZHV_dec[i] |= HF;
	}

	cpustate->_PPC = 0;
	cpustate->_PCD = 0;
	cpustate->_SPD = 0;
	cpustate->_AFD = 0;
	cpustate->_BCD = 0;
	cpustate->_DED = 0;
	cpustate->_HLD = 0;
	cpustate->_IXD = 0;
	cpustate->_IYD = 0;
	cpustate->AF2.d = 0;
	cpustate->BC2.d = 0;
	cpustate->DE2.d = 0;
	cpustate->HL2.d = 0;
	cpustate->R = 0;
	cpustate->R2 = 0;
	cpustate->IFF1 = 0;
	cpustate->IFF2 = 0;
	cpustate->HALT = 0;
	cpustate->IM = 0;
	cpustate->I = 0;
	cpustate->tmdr_latch = 0;
	cpustate->read_tcr_tmdr[0] = 0;
	cpustate->read_tcr_tmdr[1] = 0;
	cpustate->iol = 0;
	memset(cpustate->io, 0, sizeof(cpustate->io));
	memset(cpustate->mmu, 0, sizeof(cpustate->mmu));
	cpustate->tmdrh[0] = 0;
	cpustate->tmdrh[1] = 0;
	cpustate->tmdr_value[0] = 0xffff;
	cpustate->tmdr_value[1] = 0xffff;
	cpustate->tif[0] = 0;
	cpustate->tif[1] = 0;
	cpustate->nmi_state = CLEAR_LINE;
	cpustate->nmi_pending = 0;
	cpustate->irq_state[0] = CLEAR_LINE;
	cpustate->irq_state[1] = CLEAR_LINE;
	cpustate->irq_state[2] = CLEAR_LINE;
	cpustate->after_EI = 0;
	cpustate->ea = 0;

	memcpy(cpustate->cc, (UINT8 *)cc_default, sizeof(cpustate->cc));
	cpustate->_IX = cpustate->_IY = 0xffff; /* IX and IY are FFFF after a reset! */
	cpustate->_F = ZF;          /* Zero flag is set */

	for (i=0; i <= Z180_INT_MAX; i++)
		cpustate->int_pending[i] = 0;

	cpustate->timer_cnt = 0;
	cpustate->dma0_cnt = 0;
	cpustate->dma1_cnt = 0;

	/* reset io registers */
	// let ASCI initialize its registers
	/*cpustate->IO_CNTLA0  = Z180_CNTLA0_RESET;
	cpustate->IO_CNTLA1  = Z180_CNTLA1_RESET;
	cpustate->IO_CNTLB0  = Z180_CNTLB0_RESET;
	cpustate->IO_CNTLB1  = Z180_CNTLB1_RESET;
	cpustate->IO_STAT0   = Z180_STAT0_RESET;
	cpustate->IO_STAT1   = Z180_STAT1_RESET;
	cpustate->IO_TDR0    = Z180_TDR0_RESET;
	cpustate->IO_TDR1    = Z180_TDR1_RESET;
	cpustate->IO_RDR0    = Z180_RDR0_RESET;
	cpustate->IO_RDR1    = Z180_RDR1_RESET;*/
	cpustate->IO_CNTR    = Z180_CNTR_RESET;
	cpustate->IO_TRDR    = Z180_TRDR_RESET;
	cpustate->IO_TMDR0L  = Z180_TMDR0L_RESET;
	cpustate->IO_TMDR0H  = Z180_TMDR0H_RESET;
	cpustate->IO_RLDR0L  = Z180_RLDR0L_RESET;
	cpustate->IO_RLDR0H  = Z180_RLDR0H_RESET;
	cpustate->IO_TCR       = Z180_TCR_RESET;
	cpustate->IO_IO11    = Z180_IO11_RESET;
	//cpustate->IO_ASEXT0  = Z180_ASEXT0_RESET;
	//cpustate->IO_ASEXT1  = Z180_ASEXT1_RESET;
	cpustate->IO_TMDR1L  = Z180_TMDR1L_RESET;
	cpustate->IO_TMDR1H  = Z180_TMDR1H_RESET;
	cpustate->IO_RLDR1L  = Z180_RLDR1L_RESET;
	cpustate->IO_RLDR1H  = Z180_RLDR1H_RESET;
	cpustate->IO_FRC       = Z180_FRC_RESET;
	cpustate->IO_IO19    = Z180_IO19_RESET;
	/*cpustate->IO_ASTC0L  = Z180_ASTC0L_RESET;
	cpustate->IO_ASTC0H  = Z180_ASTC0H_RESET;
	cpustate->IO_ASTC1L  = Z180_ASTC1L_RESET;
	cpustate->IO_ASTC1H  = Z180_ASTC1H_RESET;*/
	cpustate->IO_CMR       = Z180_CMR_RESET;
	cpustate->IO_CCR       = Z180_CCR_RESET;
	cpustate->IO_SAR0L   = Z180_SAR0L_RESET;
	cpustate->IO_SAR0H   = Z180_SAR0H_RESET;
	cpustate->IO_SAR0B   = Z180_SAR0B_RESET;
	cpustate->IO_DAR0L   = Z180_DAR0L_RESET;
	cpustate->IO_DAR0H   = Z180_DAR0H_RESET;
	cpustate->IO_DAR0B   = Z180_DAR0B_RESET;
	cpustate->IO_BCR0L   = Z180_BCR0L_RESET;
	cpustate->IO_BCR0H   = Z180_BCR0H_RESET;
	cpustate->IO_MAR1L   = Z180_MAR1L_RESET;
	cpustate->IO_MAR1H   = Z180_MAR1H_RESET;
	cpustate->IO_MAR1B   = Z180_MAR1B_RESET;
	cpustate->IO_IAR1L   = Z180_IAR1L_RESET;
	cpustate->IO_IAR1H   = Z180_IAR1H_RESET;
	cpustate->IO_IAR1B   = Z180_IAR1B_RESET;
	cpustate->IO_BCR1L   = Z180_BCR1L_RESET;
	cpustate->IO_BCR1H   = Z180_BCR1H_RESET;
	cpustate->IO_DSTAT   = Z180_DSTAT_RESET;
	cpustate->IO_DMODE   = Z180_DMODE_RESET;
	cpustate->IO_DCNTL   = Z180_DCNTL_RESET;
	cpustate->IO_IL    = Z180_IL_RESET;
	cpustate->IO_ITC       = Z180_ITC_RESET;
	cpustate->IO_IO35    = Z180_IO35_RESET;
	cpustate->IO_RCR       = Z180_RCR_RESET;
	cpustate->IO_IO37    = Z180_IO37_RESET;
	cpustate->IO_CBR       = Z180_CBR_RESET;
	cpustate->IO_BBR       = Z180_BBR_RESET;
	cpustate->IO_CBAR    = Z180_CBAR_RESET;
	cpustate->IO_IO3B    = Z180_IO3B_RESET;
	cpustate->IO_IO3C    = Z180_IO3C_RESET;
	cpustate->IO_IO3D    = Z180_IO3D_RESET;
	cpustate->IO_OMCR    = Z180_OMCR_RESET;
	cpustate->IO_IOCR    = Z180_IOCR_RESET;

	if(cpustate->device->m_type == Z180_TYPE_Z182) {
		// let ESCC initialize it's registers
		/*cpustate->IO_SCCACNT    = Z182_SCCACNT_RESET;
		cpustate->IO_SCCAD    = Z182_SCCAD_RESET;
		cpustate->IO_SCCBCNT    = Z182_SCCBCNT_RESET;
		cpustate->IO_SCCBD    = Z182_SCCBD_RESET;*/
		cpustate->IO_SCR    = Z182_SCR_RESET;
		cpustate->IO_RAMUBR    = Z182_RAMUBR_RESET;
		cpustate->IO_RAMLBR    = Z182_RAMLBR_RESET;
		cpustate->IO_ROMBR    = Z182_ROMBR_RESET;
		cpustate->IO_WSGCSR    = Z182_WSGCSR_RESET;
		cpustate->IO_IEPMUX   = Z182_IEPMUX_RESET;
		cpustate->IO_ENHR    = Z182_ENHR_RESET;
		cpustate->IO_DDRA    = Z182_DDRA_RESET;
		cpustate->IO_DRA    = Z182_DRA_RESET;
		cpustate->IO_DDRB    = Z182_DDRB_RESET;
		cpustate->IO_DRB    = Z182_DRB_RESET;
		cpustate->IO_DDRC    = Z182_DDRC_RESET;
		cpustate->IO_DRC    = Z182_DRC_RESET;
	}

	if (cpustate->daisy != NULL)
		z80_daisy_chain_post_reset(cpustate->daisy);
	z180_mmu(cpustate);
	if (((struct z180_device *)device)->m_type == Z180_TYPE_Z182)
		z80scc_device_reset(((struct z180_device *)device)->z80scc);
}

/* Handle PRT timers, decreasing them after 20 clocks and returning the new icount base that needs to be used for the next check */
void clock_timers(struct z180_state *cpustate)
{
	cpustate->timer_cnt++;
	if (cpustate->timer_cnt >= 20)
	{
		cpustate->timer_cnt = 0;
		/* Programmable Reload Timer 0 */
		if(cpustate->IO_TCR & Z180_TCR_TDE0)
		{
			if(cpustate->tmdr_value[0] == 0)
			{
				cpustate->tmdr_value[0] = cpustate->IO_RLDR0L | (cpustate->IO_RLDR0H << 8);
				cpustate->tif[0] = 1;
			}
			else
				cpustate->tmdr_value[0]--;
		}

		/* Programmable Reload Timer 1 */
		if(cpustate->IO_TCR & Z180_TCR_TDE1)
		{
			if(cpustate->tmdr_value[1] == 0)
			{
				cpustate->tmdr_value[1] = cpustate->IO_RLDR1L | (cpustate->IO_RLDR1H << 8);
				cpustate->tif[1] = 1;
			}
			else
				cpustate->tmdr_value[1]--;
		}

		if((cpustate->IO_TCR & Z180_TCR_TIE0) && cpustate->tif[0])
		{
			// check if we can take the interrupt
			if(cpustate->IFF1 && !cpustate->after_EI)
			{
				cpustate->int_pending[Z180_INT_PRT0] = 1;
			}
		}

		if((cpustate->IO_TCR & Z180_TCR_TIE1) && cpustate->tif[1])
		{
			// check if we can take the interrupt
			if(cpustate->IFF1 && !cpustate->after_EI)
			{
				cpustate->int_pending[Z180_INT_PRT1] = 1;
			}
		}

	}
}

int check_interrupts(struct z180_state *cpustate)
{
	int i;
	int cycles = 0;

	/* check for IRQs before each instruction */
	if (cpustate->int_pending[0]) // TRAP
	{
		cycles += take_interrupt(cpustate, Z180_INT_TRAP);
		cpustate->int_pending[0] = 0;
	}
	else if (cpustate->int_pending[1]) // NMI
	{
		cycles += take_interrupt(cpustate, Z180_INT_NMI);
		cpustate->int_pending[1] = 0;
	}

	else if (cpustate->IFF1 && !cpustate->after_EI)
	{
		/* maskable interrupts */
		if (cpustate->irq_state[0] != CLEAR_LINE && (cpustate->IO_ITC & Z180_ITC_ITE0) == Z180_ITC_ITE0)
			cpustate->int_pending[Z180_INT_IRQ0] = 1;

		if (cpustate->irq_state[1] != CLEAR_LINE && (cpustate->IO_ITC & Z180_ITC_ITE1) == Z180_ITC_ITE1)
			cpustate->int_pending[Z180_INT_IRQ1] = 1;

		if (cpustate->irq_state[2] != CLEAR_LINE && (cpustate->IO_ITC & Z180_ITC_ITE2) == Z180_ITC_ITE2)
			cpustate->int_pending[Z180_INT_IRQ2] = 1;

		for (i = Z180_INT_IRQ0; i <= Z180_INT_MAX; i++)
			if (cpustate->int_pending[i])
			{
				cycles += take_interrupt(cpustate, i);
				cpustate->int_pending[i] = 0;
				break;
			}
	}

	return cycles;
}

/****************************************************************************
 * Handle I/O and timers
 ****************************************************************************/

void handle_io_timers(struct z180_state *cpustate, int cycles)
{
	while (cycles-- > 0)
	{
		clock_timers(cpustate);
	}
}

/****************************************************************************
 * Execute 'cycles' T-states. Return number of T-states really executed
 ****************************************************************************/
void cpu_execute_z180(device_t *device, int icount)
{
	struct z180_state *cpustate = get_safe_token(device);
	int curcycles;
	cpustate->icount = icount;

	/* check for NMIs on the way in; they can only be set externally */
	/* via timers, and can't be dynamically enabled, so it is safe */
	/* to just check here */
	if (cpustate->nmi_pending)
	{
		LOG("Z180 '%s' take NMI\n", cpustate->device->m_tag);
		cpustate->_PPC = -1;            /* there isn't a valid previous program counter */
		LEAVE_HALT(cpustate);       /* Check if processor was halted */

		/* disable DMA transfers!! */
		cpustate->IO_DSTAT &= ~Z180_DSTAT_DME;

		cpustate->IFF2 = cpustate->IFF1;
		cpustate->IFF1 = 0;
		PUSH(cpustate,  PC );
		cpustate->_PCD = 0x0066;
		cpustate->icount -= 11;
		cpustate->nmi_pending = 0;
		handle_io_timers(cpustate, 11);
	}

again:
	/* check if any DMA transfer is running */
	if ((cpustate->IO_DSTAT & Z180_DSTAT_DME) == Z180_DSTAT_DME)
	{
		/* check if DMA channel 0 is running and also is in burst mode */
		if ((cpustate->IO_DSTAT & Z180_DSTAT_DE0) == Z180_DSTAT_DE0 &&
			(cpustate->IO_DMODE & Z180_DMODE_MMOD) == Z180_DMODE_MMOD)
		{
			debugger_instruction_hook(device, cpustate->_PCD);

			/* FIXME z180_dma0 should be handled in handle_io_timers */
			curcycles = z180_dma0(cpustate, cpustate->icount);
			cpustate->icount -= curcycles;
			handle_io_timers(cpustate, curcycles);
		}
		else
		{
			do
			{
				curcycles = check_interrupts(cpustate);
				cpustate->icount -= curcycles;
				handle_io_timers(cpustate, curcycles);
				cpustate->after_EI = 0;

				cpustate->_PPC = cpustate->_PCD;
				debugger_instruction_hook(device, cpustate->_PCD);

				if (!cpustate->HALT)
				{
					cpustate->R++;
					cpustate->extra_cycles = 0;
					curcycles = exec_op(cpustate,ROP(cpustate));
					curcycles += cpustate->extra_cycles;
				}
				else
					curcycles = 3;

				cpustate->icount -= curcycles;

				handle_io_timers(cpustate, curcycles);

				/* if channel 0 was started in burst mode, go recheck the mode */
				if ((cpustate->IO_DSTAT & Z180_DSTAT_DE0) == Z180_DSTAT_DE0 &&
					(cpustate->IO_DMODE & Z180_DMODE_MMOD) == Z180_DMODE_MMOD)
					goto again;

				/* FIXME:
				 * For simultaneous DREQ0 and DREQ1 requests, channel 0 has priority
				 * over channel 1. When channel 0 is performing a memory to/from memory
				 * transfer, channel 1 cannot operate until the channel 0 operation has
				 * terminated. If channel 1 is operating, channel 0 cannot operate until
				 * channel 1 releases control of the bus.
				 *
				 */
				curcycles = z180_dma0(cpustate, 6);
				cpustate->icount -= curcycles;
				handle_io_timers(cpustate, curcycles);

				curcycles = z180_dma1(cpustate);
				cpustate->icount -= curcycles;
				handle_io_timers(cpustate, curcycles);

				/* If DMA is done break out to the faster loop */
				if ((cpustate->IO_DSTAT & Z180_DSTAT_DME) != Z180_DSTAT_DME)
					break;
			} while( cpustate->icount > 0 );
		}
	}

	if (cpustate->icount > 0)
	{
		do
		{
			/* If DMA is started go to check the mode */
			if ((cpustate->IO_DSTAT & Z180_DSTAT_DME) == Z180_DSTAT_DME)
				goto again;

			curcycles = check_interrupts(cpustate);
			cpustate->icount -= curcycles;
			handle_io_timers(cpustate, curcycles);
			cpustate->after_EI = 0;

			cpustate->_PPC = cpustate->_PCD;
			debugger_instruction_hook(device, cpustate->_PCD);

			if (!cpustate->HALT)
			{
				cpustate->R++;
				cpustate->extra_cycles = 0;
				curcycles = exec_op(cpustate,ROP(cpustate));
				curcycles += cpustate->extra_cycles;
			}
			else
				curcycles = 3;

			cpustate->icount -= curcycles;
			handle_io_timers(cpustate, curcycles);
		} while( cpustate->icount > 0 );
	}

	//cpustate->old_icount -= cpustate->icount;
}

/****************************************************************************
 * Burn 'cycles' T-states. Adjust R register for the lost time
 ****************************************************************************/
void cpu_burn_z180(device_t *device, int cycles)
{
	/* FIXME: This is not appropriate for dma */
	struct z180_state *cpustate = get_safe_token(device);
	while ( (cycles > 0) )
	{
		handle_io_timers(cpustate, 3);
		/* NOP takes 3 cycles per instruction */
		cpustate->R += 1;
		cpustate->icount -= 3;
		cycles -= 3;
	}
}

/****************************************************************************
 * Set IRQ line state
 ****************************************************************************/
void set_irq_line(struct z180_state *cpustate, int irqline, int state)
{
	if (irqline == INPUT_LINE_NMI)
	{
		/* mark an NMI pending on the rising edge */
		if (cpustate->nmi_state == CLEAR_LINE && state != CLEAR_LINE)
			cpustate->nmi_pending = 1;
		cpustate->nmi_state = state;
	}
	else
	{
		LOG("Z180 '%s' set_irq_line %d = %d\n",cpustate->device->m_tag , irqline,state);

		/* update the IRQ state */
		cpustate->irq_state[irqline] = state;
		if (cpustate->daisy != NULL)
			cpustate->irq_state[0] = z80_daisy_chain_update_irq_state(cpustate->daisy);

		/* the main execute loop will take the interrupt */
	}
}

/* logical to physical address translation */
int cpu_translate_z180(device_t *device, enum address_spacenum space, int intention, offs_t *address)
{
	if (space == AS_PROGRAM)
	{
		struct z180_state *cpustate = get_safe_token(device);
		*address = MMU_REMAP_ADDR(cpustate, *address);
	}
	return TRUE;
}


/**************************************************************************
 * STATE IMPORT/EXPORT
 **************************************************************************/

/*
void cpu_state_import_z180(device_t *device, int device_state_entry)
{
	struct z180_state *cpustate = get_safe_token(device);

	switch (device_state_entry)
	{
		case Z180_R:
			cpustate->R = cpustate->rtemp & 0x7f;
			cpustate->R2 = cpustate->rtemp & 0x80;
			break;

		case Z180_CBR:
		case Z180_BBR:
		case Z180_CBAR:
			z180_mmu(cpustate);
			break;

		case Z180_IOLINES:
			z180_write_iolines(cpustate, cpustate->ioltemp);
			break;

		default:
			logerror("CPU_IMPORT_STATE(z180) called for unexpected value\n");
			break;
	}
}


void cpu_state_export_z180(device_t *device, int device_state_entry)
{
	struct z180_state *cpustate = get_safe_token(device);

	switch (device_state_entry)
	{
		case Z180_R:
			cpustate->rtemp = (cpustate->R & 0x7f) | (cpustate->R2 & 0x80);
			break;

		case Z180_IOLINES:
			cpustate->ioltemp = cpustate->iol;
			break;

		default:
			logerror("CPU_EXPORT_STATE(z180) called for unexpected value\n");
			break;
	}
}*/

void cpu_string_export_z180(device_t *device, int device_state_entry, char *string)
{
	struct z180_state *cpustate = get_safe_token(device);

	switch (device_state_entry)
	{
		case STATE_GENFLAGS:
			sprintf(string,"%c%c%c%c%c%c",
				cpustate->AF.b.l & 0x80 ? 'S':'.',
				cpustate->AF.b.l & 0x40 ? 'Z':'.',
				//cpustate->AF.b.l & 0x20 ? '5':'.',
				cpustate->AF.b.l & 0x10 ? 'H':'.',
				//cpustate->AF.b.l & 0x08 ? '3':'.',
				cpustate->AF.b.l & 0x04 ? 'P':'.',
				cpustate->AF.b.l & 0x02 ? 'N':'.',
				cpustate->AF.b.l & 0x01 ? 'C':'.');
			break;
	}
}

offs_t cpu_get_state_z180(device_t *device, int device_state_entry) {

	struct z180_state *cpustate = get_safe_token(device);

	switch (device_state_entry)
	{
		case Z180_PC: return cpustate->PC.w.l;
		case STATE_GENPC: return cpustate->_PCD;
		case STATE_GENPCBASE: return cpustate->PREPC.w.l;
		case Z180_SP: return cpustate->_SPD;
		case STATE_GENSP: return cpustate->SP.w.l;
		case STATE_GENFLAGS: return cpustate->_F;
		case Z180_A: return cpustate->_A;
		case Z180_B: return cpustate->_B;
		case Z180_C: return cpustate->_C;
		case Z180_D: return cpustate->_D;
		case Z180_E: return cpustate->_E;
		case Z180_H: return cpustate->_H;
		case Z180_L: return cpustate->_L;
		case Z180_AF: return cpustate->AF.w.l;
		case Z180_BC: return cpustate->BC.w.l;
		case Z180_DE: return cpustate->DE.w.l;
		case Z180_HL: return cpustate->HL.w.l;
		case Z180_IX: return cpustate->IX.w.l;
		case Z180_IY: return cpustate->IY.w.l;
		case Z180_AF2: return cpustate->AF2.w.l;
		case Z180_BC2: return cpustate->BC2.w.l;
		case Z180_DE2: return cpustate->DE2.w.l;
		case Z180_HL2: return cpustate->HL2.w.l;
		case Z180_R: return cpustate->R;
		case Z180_I: return cpustate->I;
		case Z180_IM: return cpustate->IM &0x3;
		case Z180_IFF1: return cpustate->IFF1 &0x1;
		case Z180_IFF2: return cpustate->IFF2 &0x1;
		case Z180_HALT: return cpustate->HALT &0x1;

		//case Z180_IOLINES: return cpustate->iol & 0xffffff;

		/*case Z180_CNTLA0: return cpustate->IO_CNTLA0;
		case Z180_CNTLA1: return cpustate->IO_CNTLA1;
		case Z180_CNTLB0: return cpustate->IO_CNTLB0;
		case Z180_CNTLB1: return cpustate->IO_CNTLB1;
		case Z180_STAT0: return cpustate->IO_STAT0;
		case Z180_STAT1: return cpustate->IO_STAT1;
		case Z180_TDR0: return cpustate->IO_TDR0;
		case Z180_TDR1: return cpustate->IO_TDR1;
		case Z180_RDR0: return cpustate->IO_RDR0;
		case Z180_RDR1: return cpustate->IO_RDR1;*/
		case Z180_CNTR: return cpustate->IO_CNTR;
		case Z180_TRDR: return cpustate->IO_TRDR;
		case Z180_TMDR0L: return cpustate->IO_TMDR0L;
		case Z180_TMDR0H: return cpustate->IO_TMDR0H;
		case Z180_RLDR0L: return cpustate->IO_RLDR0L;
		case Z180_RLDR0H: return cpustate->IO_RLDR0H;
		case Z180_TCR: return cpustate->IO_TCR;
		case Z180_IO11: return cpustate->IO_IO11;
		//case Z180_ASEXT0: return cpustate->IO_ASEXT0;
		//case Z180_ASEXT1: return cpustate->IO_ASEXT1;
		case Z180_TMDR1L: return cpustate->IO_TMDR1L;
		case Z180_TMDR1H: return cpustate->IO_TMDR1H;
		case Z180_RLDR1L: return cpustate->IO_RLDR1L;
		case Z180_RLDR1H: return cpustate->IO_RLDR1H;
		case Z180_FRC: return cpustate->IO_FRC;
		case Z180_IO19: return cpustate->IO_IO19;
		/*case Z180_ASTC0L: return cpustate->IO_ASTC0L;
		case Z180_ASTC0H: return cpustate->IO_ASTC0H;
		case Z180_ASTC1L: return cpustate->IO_ASTC1L;
		case Z180_ASTC1H: return cpustate->IO_ASTC1H;*/
		case Z180_CMR: return cpustate->IO_CMR;
		case Z180_CCR: return cpustate->IO_CCR;
		case Z180_SAR0L: return cpustate->IO_SAR0L;
		case Z180_SAR0H: return cpustate->IO_SAR0H;
		case Z180_SAR0B: return cpustate->IO_SAR0B;
		case Z180_DAR0L: return cpustate->IO_DAR0L;
		case Z180_DAR0H: return cpustate->IO_DAR0H;
		case Z180_DAR0B: return cpustate->IO_DAR0B;
		case Z180_BCR0L: return cpustate->IO_BCR0L;
		case Z180_BCR0H: return cpustate->IO_BCR0H;
		case Z180_MAR1L: return cpustate->IO_MAR1L;
		case Z180_MAR1H: return cpustate->IO_MAR1H;
		case Z180_MAR1B: return cpustate->IO_MAR1B;
		case Z180_IAR1L: return cpustate->IO_IAR1L;
		case Z180_IAR1H: return cpustate->IO_IAR1H;
		case Z180_IAR1B: return cpustate->IO_IAR1B;
		case Z180_BCR1L: return cpustate->IO_BCR1L;
		case Z180_BCR1H: return cpustate->IO_BCR1H;
		case Z180_DSTAT: return cpustate->IO_DSTAT;
		case Z180_DMODE: return cpustate->IO_DMODE;
		case Z180_DCNTL: return cpustate->IO_DCNTL;
		case Z180_IL: return cpustate->IO_IL;
		case Z180_ITC: return cpustate->IO_ITC;
		case Z180_IO35: return cpustate->IO_IO35;
		case Z180_RCR: return cpustate->IO_RCR;
		case Z180_IO37: return cpustate->IO_IO37;
		case Z180_CBR: return cpustate->IO_CBR;
		case Z180_BBR: return cpustate->IO_BBR;
		case Z180_CBAR: return cpustate->IO_CBAR;
		case Z180_IO3B: return cpustate->IO_IO3B;
		case Z180_IO3C: return cpustate->IO_IO3C;
		case Z180_IO3D: return cpustate->IO_IO3D;
		case Z180_OMCR: return cpustate->IO_OMCR;
		case Z180_IOCR: return cpustate->IO_IOCR;

			/*case Z182_SCCACNT:  return cpustate->IO_SCCACNT;
			case Z182_SCCAD:  return cpustate->IO_SCCAD;
			case Z182_SCCBCNT: return cpustate->IO_SCCBCNT;
			case Z182_SCCBD: return cpustate->IO_SCCBD;*/
			case Z182_SCR: return cpustate->IO_SCR;
			case Z182_RAMUBR: return cpustate->IO_RAMUBR;
			case Z182_RAMLBR: return cpustate->IO_RAMLBR;
			case Z182_ROMBR: return cpustate->IO_ROMBR;
			case Z182_WSGCSR: return cpustate->IO_WSGCSR;
			case Z182_IEPMUX: return cpustate->IO_IEPMUX;
			case Z182_ENHR: return cpustate->IO_ENHR;
			case Z182_DDRA: return cpustate->IO_DDRA;
			case Z182_DRA: return cpustate->IO_DRA;
			case Z182_DDRB: return cpustate->IO_DDRB;
			case Z182_DRB: return cpustate->IO_DRB;
			case Z182_DDRC: return cpustate->IO_DDRC;
			case Z182_DRC: return cpustate->IO_DRC;

		default:
			return 0;
	}
}

/**************************************************************************
 * Generic set_info
 **************************************************************************/

/*static CPU_SET_INFO( z180 )
{
	struct z180_state *cpustate = get_safe_token(device);
	switch (state)
	{
		/ --- the following bits of info are set as 64-bit signed integers --- /
		case CPUINFO_INT_INPUT_STATE + INPUT_LINE_NMI:  set_irq_line(cpustate, INPUT_LINE_NMI, info->i);    break;
		case CPUINFO_INT_INPUT_STATE + Z180_IRQ0:       set_irq_line(cpustate, Z180_IRQ0, info->i);         break;
		case CPUINFO_INT_INPUT_STATE + Z180_IRQ1:       set_irq_line(cpustate, Z180_IRQ1, info->i);         break;
		case CPUINFO_INT_INPUT_STATE + Z180_IRQ2:       set_irq_line(cpustate, Z180_IRQ2, info->i);         break;

		/ --- the following bits of info are set as pointers to data or functions --- /
		case CPUINFO_PTR_Z180_CYCLE_TABLE + Z180_TABLE_op:      cpustate->cc[Z180_TABLE_op] = (UINT8 *)info->p;     break;
		case CPUINFO_PTR_Z180_CYCLE_TABLE + Z180_TABLE_cb:      cpustate->cc[Z180_TABLE_cb] = (UINT8 *)info->p;     break;
		case CPUINFO_PTR_Z180_CYCLE_TABLE + Z180_TABLE_ed:      cpustate->cc[Z180_TABLE_ed] = (UINT8 *)info->p;     break;
		case CPUINFO_PTR_Z180_CYCLE_TABLE + Z180_TABLE_xy:      cpustate->cc[Z180_TABLE_xy] = (UINT8 *)info->p;     break;
		case CPUINFO_PTR_Z180_CYCLE_TABLE + Z180_TABLE_xycb:    cpustate->cc[Z180_TABLE_xycb] = (UINT8 *)info->p;   break;
		case CPUINFO_PTR_Z180_CYCLE_TABLE + Z180_TABLE_ex:      cpustate->cc[Z180_TABLE_ex] = (UINT8 *)info->p;     break;
	}
}*/


/**************************************************************************
 * Generic get_info
 **************************************************************************/

/*CPU_GET_INFO( z180 )
{
	struct z180_state *cpustate = (device != NULL && device->token() != NULL) ? get_safe_token(device) : NULL;
	switch (state)
	{
		/ --- the following bits of info are returned as 64-bit signed integers --- /
		case CPUINFO_INT_CONTEXT_SIZE:                  info->i = sizeof(z180_state);           break;
		case CPUINFO_INT_INPUT_LINES:                   info->i = 3;                            break;
		case CPUINFO_INT_DEFAULT_IRQ_VECTOR:            info->i = 0xff;                         break;
		case CPUINFO_INT_ENDIANNESS:                    info->i = ENDIANNESS_LITTLE;            break;
		case CPUINFO_INT_CLOCK_MULTIPLIER:              info->i = 1;                            break;
		case CPUINFO_INT_CLOCK_DIVIDER:                 info->i = 1;                            break;
		case CPUINFO_INT_MIN_INSTRUCTION_BYTES:         info->i = 1;                            break;
		case CPUINFO_INT_MAX_INSTRUCTION_BYTES:         info->i = 4;                            break;
		case CPUINFO_INT_MIN_CYCLES:                    info->i = 1;                            break;
		case CPUINFO_INT_MAX_CYCLES:                    info->i = 16;                           break;

		case CPUINFO_INT_DATABUS_WIDTH + AS_PROGRAM:            info->i = 8;                            break;
		case CPUINFO_INT_ADDRBUS_WIDTH + AS_PROGRAM:        info->i = 20;                           break;
		case CPUINFO_INT_ADDRBUS_SHIFT + AS_PROGRAM:        info->i = 0;                            break;
		case CPUINFO_INT_DATABUS_WIDTH + AS_IO:             info->i = 8;                            break;
		case CPUINFO_INT_ADDRBUS_WIDTH + AS_IO:             info->i = 16;                           break;
		case CPUINFO_INT_ADDRBUS_SHIFT + AS_IO:             info->i = 0;                            break;

		case CPUINFO_INT_INPUT_STATE + INPUT_LINE_NMI:  info->i = cpustate->nmi_state;          break;
		case CPUINFO_INT_INPUT_STATE + Z180_IRQ0:       info->i = cpustate->irq_state[0];       break;
		case CPUINFO_INT_INPUT_STATE + Z180_IRQ1:       info->i = cpustate->irq_state[1];       break;
		case CPUINFO_INT_INPUT_STATE + Z180_IRQ2:       info->i = cpustate->irq_state[2];       break;

		/ --- the following bits of info are returned as pointers --- /
		case CPUINFO_FCT_SET_INFO:      info->setinfo = CPU_SET_INFO_NAME(z180);                break;
		case CPUINFO_FCT_INIT:          info->init = CPU_INIT_NAME(z180);                       break;
		case CPUINFO_FCT_RESET:         info->reset = CPU_RESET_NAME(z180);                     break;
		case CPUINFO_FCT_EXECUTE:       info->execute = CPU_EXECUTE_NAME(z180);                 break;
		case CPUINFO_FCT_BURN:          info->burn = CPU_BURN_NAME(z180);                       break;
		case CPUINFO_FCT_DISASSEMBLE:   info->disassemble = CPU_DISASSEMBLE_NAME(z180);         break;
		case CPUINFO_FCT_TRANSLATE:     info->translate = CPU_TRANSLATE_NAME(z180);             break;
		case CPUINFO_FCT_IMPORT_STATE:  info->import_state = CPU_IMPORT_STATE_NAME(z180);       break;
		case CPUINFO_FCT_EXPORT_STATE:  info->export_state = CPU_EXPORT_STATE_NAME(z180);       break;
		case CPUINFO_FCT_EXPORT_STRING: info->export_string = CPU_EXPORT_STRING_NAME(z180);     break;

		/ --- the following bits of info are returned as pointers to functions --- /
		case CPUINFO_PTR_INSTRUCTION_COUNTER:           info->icount = &cpustate->icount;       break;

		case CPUINFO_PTR_Z180_CYCLE_TABLE + Z180_TABLE_op:      info->p = (void *)cpustate->cc[Z180_TABLE_op];  break;
		case CPUINFO_PTR_Z180_CYCLE_TABLE + Z180_TABLE_cb:      info->p = (void *)cpustate->cc[Z180_TABLE_cb];  break;
		case CPUINFO_PTR_Z180_CYCLE_TABLE + Z180_TABLE_ed:      info->p = (void *)cpustate->cc[Z180_TABLE_ed];  break;
		case CPUINFO_PTR_Z180_CYCLE_TABLE + Z180_TABLE_xy:      info->p = (void *)cpustate->cc[Z180_TABLE_xy];  break;
		case CPUINFO_PTR_Z180_CYCLE_TABLE + Z180_TABLE_xycb:    info->p = (void *)cpustate->cc[Z180_TABLE_xycb];    break;
		case CPUINFO_PTR_Z180_CYCLE_TABLE + Z180_TABLE_ex:      info->p = (void *)cpustate->cc[Z180_TABLE_ex];  break;

		/ --- the following bits of info are returned as NULL-terminated strings --- /
		case CPUINFO_STR_NAME:                          strcpy(info->s, "Z180");                break;
		case CPUINFO_STR_SHORTNAME:                     strcpy(info->s, "z180");                break;
		case CPUINFO_STR_FAMILY:                    strcpy(info->s, "Zilog Z8x180");        break;
		case CPUINFO_STR_VERSION:                   strcpy(info->s, "0.4");                 break;
		case CPUINFO_STR_SOURCE_FILE:                       strcpy(info->s, __FILE__);              break;
		case CPUINFO_STR_CREDITS:                   strcpy(info->s, "Copyright Juergen Buchmueller, all rights reserved."); break;
	}
}*/

//DEFINE_LEGACY_CPU_DEVICE(Z180, z180);
