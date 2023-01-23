/*
 * Simple sdcard in SPI mode emulation
 * (c)2023 Hamish Coleman <hamish@zot.org>
 *
 * Public interface definitions
 */
#ifndef SDCARD_H
#define SDCARD_H

// Remember to update sdcard_state_names too
enum sdcard_state {
    IDLE = 0,
    RX_CMD,
    TX_R1,
    TX_R3,
    TX_R7,
    TX_R1_TX_BLOCK,
    TX_R1_RX_BLOCK,
    RX_BLOCK,
    TX_RX_BLOCK_STAT,
};

struct sdcard_device {
    int fd;
    enum sdcard_state state;
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
