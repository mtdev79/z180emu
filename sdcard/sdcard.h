/*
 * Simple sdcard in SPI mode emulation
 * (c)2023 Hamish Coleman <hamish@zot.org>
 *
 * Public interface definitions
 */
#ifndef SDCARD_H
#define SDCARD_H

enum sdcard_state {
    IDLE = 0,
    RX_CMD,
    TX_RESP,
    PRE_WRITE_STAT,
    WRITE_BLOCK,
};

struct sdcard_device {
    int fd;
    enum sdcard_state state;
    int cmd_ptr;
    int resp_ptr;
    UINT8 cmd[8];
    UINT8 resp[512+6];
};

extern int sdcard_trace;

// TODO: technically, SPI is simultaneous readwrite
int sdcard_read(struct sdcard_device *device, UINT8 data);
int sdcard_write(struct sdcard_device *device, UINT8 data);
void sdcard_reset(struct sdcard_device *sd);
int sdcard_init(struct sdcard_device *sd, char *filename);

#endif
