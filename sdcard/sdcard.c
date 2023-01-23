/*
 * Simple sdcard in SPI mode emulation
 * (c)2023 Hamish Coleman <hamish@zot.org>
 *
 * TODO:
 * - hook up writes
 * - improve the CSD to reflect the actual size of the backing file
 */

#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include <fcntl.h>
#include <unistd.h>

typedef uint8_t UINT8;

#include "sdcard.h"

#define dprint(level, ...) do { \
    if (sdcard_trace >= level) printf (__VA_ARGS__); \
} while(0)

int sdcard_trace = 1;

void sdcard_setstate(struct sdcard_device *sd, enum sdcard_state newstate) {
    if (sd->state != newstate) {
           dprint(1,"SD: old=%i, new=%i\n", sd->state, newstate);
    }
    sd->state = newstate;
}

void sdcard_reset_ptr(struct sdcard_device *sd) {
    sd->cmd_ptr = 0;
    sd->resp_ptr = 0;
}

void sdcard_reset(struct sdcard_device *sd) {
    sdcard_setstate(sd, IDLE);
    sdcard_reset_ptr(sd);
}

int sdcard_init(struct sdcard_device *sd, char *filename) {
    memset((void *)sd, 0, sizeof(*sd));
    sdcard_reset_ptr(sd);

    if ((sd->fd = open(filename, O_RDWR))==-1) {
        return -1;
    }
    return 1;
}

void sdcard_dump(struct sdcard_device *sd) {
    printf("SD:DUMP:  s%i,t%x,r%x, ",
        sd->state,
        sd->cmd_ptr,
        sd->resp_ptr
    );

    int i;
    for (i=0; i<sizeof(sd->cmd); i++) {
        printf("%02x", sd->cmd[i]);
    }
    printf("-");
    for (i=0; i<20; i++) {
        printf("%02x", sd->resp[i]);
    }
    printf("\n");

}

int sdcard_write(struct sdcard_device *sd, int cs, UINT8 data) {
    if (!cs) {
        // Not for us, dont drive any bits in reply
        return 0;
        // TODO: detect init sequence
    }

    if (sdcard_trace>1) {
        sdcard_dump(sd);
        printf("SD:WR:    d=0x%02x\n", data);
    }

    if (sd->state == TX_RESP) {
        // got a write when we thought we needed a read
        sdcard_reset(sd);
        dprint(1,"SD: got write, expected read\n");
    }
    if (sd->state == IDLE) {
        if (data &0x80) {
            // if the first byte of a cmd doesnt have b7=0, b6=1
            // this is probably a dummy write, stay in IDLE
            dprint(1,"SD: got dummy write?\n");
            return 0xff; // in write state, return is ignored
        }
        sdcard_setstate(sd, RX_CMD);
        sdcard_reset_ptr(sd);
    } else if (sd->state == WRITE_BLOCK) {
        // FIXME - implement a buffer!
        return 0xff; // in write state, return is ignored
    }
    // else bad state?

    sd->cmd[sd->cmd_ptr] = data;
    sd->cmd_ptr = (sd->cmd_ptr & 0x07) + 1;

    if (sd->cmd_ptr != 6) {
        return 0xff; // in write state, return is ignored
    }

    UINT8 cmd = sd->cmd[0];

    // we have a whole cmd
    // should start with 0b01xxxxxx (x = cmd)
    // should end with   0bccccccc1 (c = crc)

    // TODO: check cmd[5] CRC

    int rp = 0;
    switch (cmd) {
        case 0x40: // CMD0  GO_IDLE_STATE
            dprint(1,"SD: CMD0 GO_IDLE_STATE\n");
        case 0x77: // CMD55 APP_CMD
            sd->resp[0] = 0xff;
            sd->resp[1] = 0x01;
            sdcard_setstate(sd, TX_RESP);
            sdcard_reset_ptr(sd);
            break;
        case 0x48: // CMD8  SEND_IF_COND
            dprint(1,"SD: CMD8 SEND_IF_COND\n");
            // TODO: check arg 00 00 01 aa
            sd->resp[0] = 0xff;
            sd->resp[1] = 0x01;
            sd->resp[2] = 0x00;
            sd->resp[3] = 0x00;
            sd->resp[4] = 0x01;
            sd->resp[5] = 0xaa;
            sdcard_setstate(sd, TX_RESP);
            sdcard_reset_ptr(sd);
            break;
        case 0x49: // CMD9  SEND_CSD
            dprint(1,"SD: CMD9 SEND_CSD\n");
            sd->resp[rp++] = 0xff;
            sd->resp[rp++] = 0x01;
            sd->resp[rp++] = 0xfe; // data token

            sd->resp[rp++] = 0x00;
            sd->resp[rp++] = 0x00;
            sd->resp[rp++] = 0x00;
            sd->resp[rp++] = 0x00;
            sd->resp[rp++] = 0x00;
            sd->resp[rp++] = 0x00;
            sd->resp[rp++] = 0x00;
            sd->resp[rp++] = 0x00; // LBA
            sd->resp[rp++] = 0x08; // .
            sd->resp[rp++] = 0x00; // .
            sd->resp[rp++] = 0x00; // Approx 1Gig
            sd->resp[rp++] = 0x00;
            sd->resp[rp++] = 0x00;
            sd->resp[rp++] = 0x00;
            sd->resp[rp++] = 0x00;
            sd->resp[rp++] = 0x00;
            // TODO: use real sizes

            sd->resp[rp++] = 0x05; // CRC1
            sd->resp[rp++] = 0x0a; // CRC2
            sdcard_setstate(sd, TX_RESP);
            sdcard_reset_ptr(sd);
            break;
        case 0x4a: // CMD10 SEND_CID
            dprint(1,"SD: CMD10 SEND_CID\n");
            sd->resp[rp++] = 0xff;
            sd->resp[rp++] = 0x01;
            sd->resp[rp++] = 0xfe; // data token

            sd->resp[rp++] = 0x01; // Manuf
            sd->resp[rp++] = 'A'; // App
            sd->resp[rp++] = 'A';
            sd->resp[rp++] = 'B'; // Name
            sd->resp[rp++] = 'B';
            sd->resp[rp++] = 'B';
            sd->resp[rp++] = 'B';
            sd->resp[rp++] = 'B';
            sd->resp[rp++] = 0x10; // Rev
            sd->resp[rp++] = 0x12; // Serial
            sd->resp[rp++] = 0x34;
            sd->resp[rp++] = 0x56;
            sd->resp[rp++] = 0x78;
            sd->resp[rp++] = 0x00; // 2YY
            sd->resp[rp++] = 0x01; // YM
            sd->resp[rp++] = 0x00;

            sd->resp[rp++] = 0x05; // CRC1
            sd->resp[rp++] = 0x0a; // CRC2
            sdcard_setstate(sd, TX_RESP);
            sdcard_reset_ptr(sd);
            break;
        case 0x50: // CMD16 SET_BLOCKLEN
            dprint(1,"SD: CMD16 SET_BLOCKLEN\n");
            // TODO: do a real check of the value
            if (sd->cmd[3] != 2) {
                dprint(1,"SD: unexpected blocklen\n");
                goto error;
            }
            sd->resp[0] = 0xff;
            sd->resp[1] = 0x01;
            sdcard_setstate(sd, TX_RESP);
            sdcard_reset_ptr(sd);
            break;
        case 0x51: { // CMD17 READ_SINGLE_BLOCK
            dprint(1,"SD: CMD17 READ_SINGLE_BLOCK\n");
            int block = (
                sd->cmd[1]<<24 |
                sd->cmd[2]<<16 |
                sd->cmd[3]<<8 |
                sd->cmd[4]
            );
            dprint(1,"SD:READ:  0x%04x\n", block);

            sd->resp[0] = 0xff;
            sd->resp[1] = 0x01;
            sd->resp[2] = 0xfe; // data token

            if (lseek(sd->fd, block*0x200, SEEK_SET) != -1) {
                read(sd->fd, &sd->resp[3], 0x200);
            }

            sd->resp[512+3+0] = 0x05; // CRC
            sd->resp[512+3+1] = 0x0a;
            sdcard_setstate(sd, TX_RESP);
            sdcard_reset_ptr(sd);
            break;
        }
        case 0x58: { // CMD24 WRITE_BLOCK
            dprint(1,"SD: CMD24 WRITE_BLOCK\n");
            int block = (
                sd->cmd[1]<<24 |
                sd->cmd[2]<<16 |
                sd->cmd[3]<<8 |
                sd->cmd[4]
            );
            dprint(1,"SD:WRITE: 0x%04x\n", block);

            sd->resp[rp++] = 0xff;
            sd->resp[rp++] = 0x01;

            sdcard_setstate(sd, PRE_WRITE_STAT);
            sdcard_reset_ptr(sd);
            break;
        }

        case 0x7a: // CMD58 READ_OCR
            dprint(1,"SD: CMD58 READ_OCR\n");
            sd->resp[0] = 0xff;
            sd->resp[1] = 0x01;
            sd->resp[2] = 0x40; // bit30 == HCS
            sd->resp[3] = 0x00;
            sd->resp[4] = 0x00;
            sd->resp[5] = 0x00;
            sdcard_setstate(sd, TX_RESP);
            sdcard_reset_ptr(sd);
            break;

        // TODO: gate ACMD values on a previous CMD55 APP_CMD
        case 0x69: // ACMD41 SEND_OP_COND
            dprint(1,"SD: ACMD41 SEND_OP_COND\n");
            sd->resp[0] = 0xff;
            sd->resp[1] = 0x00;
            sdcard_setstate(sd, TX_RESP);
            sdcard_reset_ptr(sd);
            break;
        case 0x73: // ACMD51 SEND_SCR
            dprint(1,"SD: ACMD51 SEND_SCR\n");
            sd->resp[rp++] = 0xff;
            sd->resp[rp++] = 0x01;
            sd->resp[rp++] = 0xfe; // data token
            sd->resp[rp++] = 0x00;
            sd->resp[rp++] = 0x00;
            sd->resp[rp++] = 0x30; // SD_SEC_2 == SDHC
            sd->resp[rp++] = 0x00;
            sd->resp[rp++] = 0x00;
            sd->resp[rp++] = 0x00;
            sd->resp[rp++] = 0x00;
            sd->resp[rp++] = 0x00;
            sd->resp[rp++] = 0x05; // CRC1
            sd->resp[rp++] = 0x0a; // CRC2
            sdcard_setstate(sd, TX_RESP);
            sdcard_reset_ptr(sd);
            break;

error:
        default:
            sd->resp[0] = 0xff;
            sd->resp[1] = 0x05; // illegal command
            sdcard_setstate(sd, TX_RESP);
            sdcard_reset_ptr(sd);

            dprint(1,"unknown\n");
            dprint(1,"SD:CMD:   cmd=0x%02x\n",cmd);

            return 0xff; // in write state, return is ignored
    }
    if (sdcard_trace>1) {
        printf("\n");
    }
}

int sdcard_read(struct sdcard_device *sd, int cs, UINT8 data) {
    if (!cs) {
        // Not for us, dont drive any bits in reply
        return 0;
    }

    UINT8 result;
    if (sdcard_trace>1) {
        sdcard_dump(sd);
    }

    if (sd->state == RX_CMD) {
        // got a read when we expected more writes
        sdcard_reset(sd);
        dprint(1,"SD: read when RX_CMD\n");
    }
    if (sd->state == WRITE_BLOCK) {
        // got the read after the end of a write block
        sdcard_setstate(sd, TX_RESP);

        dprint(1,"SD: FIXME write data accepted\n");
        result = 0x05; // Data accepted
        goto out;
    }
    if (sd->state == IDLE) {
        // in idle mode, the card just outputs ones?
        dprint(1,"SD: read while IDLE\n");
        result = 0xff;
        goto out;
    }

    result = sd->resp[sd->resp_ptr];
    sd->resp_ptr = sd->resp_ptr + 1;
    if (sd->resp_ptr >= sizeof(sd->resp)) {
        sd->resp_ptr = 0;
    }

    if (sd->state == PRE_WRITE_STAT && sd->resp_ptr == 2) {
        sdcard_setstate(sd, WRITE_BLOCK);
    }

out:
    if (sdcard_trace>1) {
        printf("SD:RD:    r=0x%02x\n", result);
    }
    return result;
}

