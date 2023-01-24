/*
 * Simple sdcard in SPI mode emulation
 * (c)2023 Hamish Coleman <hamish@zot.org>
 *
 * Public interface definitions
 */
#ifndef SDCARD_H
#define SDCARD_H

// States are named from the SDcard's point of view (thus "TX" is the card
// intends to transmit)
//
// Remember to update sdcard_state_names too
enum sdcard_state {
    IDLE = 0,

    RX_CMD,
    RX_BLOCK,

    TX_BUSY,
    TX_R1,
    TX_R3,
    TX_R7,
    TX_IMM32,
    TX_BLOCK_R1,
    TX_BLOCK_TOKEN,
    TX_BLOCK_BUF,
    TX_RX_BLOCK_R1,
    TX_RX_BLOCK_STAT,

    NEXT,   // Pseudo state, requesting to shift to the "next" state
};

struct sdcard_device {
    int fd;
    enum sdcard_state state;
    enum sdcard_state state_next;
    int cmd_ptr;
    int resp_ptr;
    int tx_len; // size of valid data in resp to TX
    UINT8 r1; // result code to send with R1
    UINT8 cmd[8];
    UINT8 resp[512+6];
};

extern int sdcard_trace;

// TODO: technically, SPI is simultaneous readwrite
int sdcard_read(struct sdcard_device *device, int cs, UINT8 data);
int sdcard_write(struct sdcard_device *device, int cs, UINT8 data);
int sdcard_init(struct sdcard_device *sd, char *filename);

#endif
