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

char *sdcard_state_names[] = {
    [IDLE] = "IDLE",

    [RX_CMD] = "RX_CMD",
    [RX_BLOCK] = "RX_BLOCK",

    [TX_BUSY] = "TX_BUSY",
    [TX_R1] = "TX_R1",
    [TX_R3] = "TX_R3",
    [TX_R7] = "TX_R7",
    [TX_IMM32] = "TX_IMM32",
    [TX_BLOCK_R1] = "TX_BLOCK_R1",
    [TX_BLOCK_TOKEN] = "TX_BLOCK_TOKEN",
    [TX_BLOCK_BUF] = "TX_BLOCK_BUF",
    [TX_RX_BLOCK_R1] = "TX_RX_BLOCK_R1",
    [TX_RX_BLOCK_STAT] = "TX_RX_BLOCK_STAT",

    [NEXT] = "next",
};

void sdcard_setstate(struct sdcard_device *sd, enum sdcard_state newstate) {
    if (newstate == NEXT) {
        newstate = sd->state_next;
        sd->state_next = IDLE;
    }
    if (sd->state != newstate) {
        dprint(1,"SD: %s -> %s\n",
            sdcard_state_names[sd->state],
            sdcard_state_names[newstate]
        );
    }
    sd->state = newstate;
}

void sdcard_reset_ptr(struct sdcard_device *sd) {
    sd->cmd_ptr = 0;
    sd->resp_ptr = 0;
    sd->tx_len = 0;
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

#define R1_0        0x00
#define R1_OK       0x01
#define R1_ILL_CMD  0x05
void sdcard_resp_r1(struct sdcard_device *sd, UINT8 r1) {
    sd->resp_ptr = 0;
    // TODO: sd->tx_len = 0;
    sd->r1 = r1;
    sd->state_next = TX_R1;
    sdcard_setstate(sd, TX_BUSY);
}

void sdcard_resp_reply32(struct sdcard_device *sd, int reply) {
    sd->resp[0] = (reply >> 24) & 0xff;
    sd->resp[1] = (reply >> 16) & 0xff;
    sd->resp[2] = (reply >> 8) & 0xff;
    sd->resp[3] = reply & 0xff;
}

void sdcard_resp_r3(struct sdcard_device *sd, UINT8 r1, int value) {
    sd->resp_ptr = 0;
    sd->r1 = r1;
    // TODO: sd->tx_len = 4;
    sdcard_resp_reply32(sd, value);
    sd->state_next = TX_R3;
    sdcard_setstate(sd, TX_BUSY);
}

// TODO: r7 is almost identical to r3, coalesce them
void sdcard_resp_r7(struct sdcard_device *sd, UINT8 r1, int value) {
    sd->resp_ptr = 0;
    sd->r1 = r1;
    // TODO: sd->tx_len = 4;
    sdcard_resp_reply32(sd, value);
    sd->state_next = TX_R7;
    sdcard_setstate(sd, TX_BUSY);
}

void sdcard_resp_tx_block(struct sdcard_device *sd, UINT8 r1, int tx_len) {
    sd->resp_ptr = 0;
    sd->tx_len = tx_len;
    sd->r1 = r1;
    sd->state_next = TX_BLOCK_R1;
    sdcard_setstate(sd, TX_BUSY);
}

void sdcard_resp_rx_block(struct sdcard_device *sd) {
    sd->resp_ptr = 0;
    sd->tx_len = 0; // This overwrites the buffer, so mark it as empty
    sd->r1 = 0; // R1 should indicate not idle
    sd->state_next = TX_RX_BLOCK_R1;
    sdcard_setstate(sd, TX_BUSY);
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

    switch (sd->state) {
        case TX_R1:
        case TX_R3:
        case TX_R7:
        case TX_IMM32:
        case TX_BLOCK_R1:
        case TX_BLOCK_TOKEN:
        case TX_BLOCK_BUF:
        case TX_RX_BLOCK_R1:
        case TX_RX_BLOCK_STAT:
            // got a write when we thought we needed a read
            dprint(2,"SD: got write, expected read (data=0x%02x)\n", data);
            /* FALL THROUGH */

        case IDLE:
            if ((data & 0xc0) != 0x40) {
                // The first two bits of a command must be 0,1
                // otherwise, we stay in IDLE
                dprint(2,"SD: got dummy write?\n");
                return 0xff; // TODO: signal that we didnt drive DO
            }

            sdcard_setstate(sd, RX_CMD);
            sdcard_reset_ptr(sd);
            /* FALL THROUGH */

        case RX_CMD:
            sd->cmd[sd->cmd_ptr++] = data;
            if (sd->cmd_ptr < 6) {
                return 0xff; // in write state, return is ignored
            }
            break;

        case RX_BLOCK:
            if (sd->resp_ptr == 0) {
                // data token
                if (data != 0xfe) {
                    dprint(1,"SD: unexpected data token 0x%02x\n", data);
                    sdcard_reset(sd);
                }
            } else if (sd->resp_ptr < 514) {
                int index = sd->resp_ptr-1;
                sd->resp[index] = data;
            } else if (sd->resp_ptr == 515) {
                // ignore CRC1
            } else {
                // ignore CRC2

                // process entire block
                int block = (
                    sd->cmd[1]<<24 |
                    sd->cmd[2]<<16 |
                    sd->cmd[3]<<8 |
                    sd->cmd[4]
                );
                dprint(1,"SD:WRITE: 0x%04x RX_BUFFER\n", block);

                // TODO: wire up write command here

                sd->r1 = 0x05; // Data accepted
                sdcard_setstate(sd, TX_RX_BLOCK_STAT);
            }
            sd->resp_ptr++;
            return 0xff;

        default:
            dprint(1,"SD:WR: unexpected state %i\n", sd->state);
            return 0xff;
    }

    // If we got here, we have an entire command packet

    UINT8 cmd = sd->cmd[0];

    // should start with 0b01xxxxxx (x = cmd)
    // should end with   0bccccccc1 (c = crc)

    // TODO: check cmd[5] CRC

    int rp = 0;
    switch (cmd) {
        case 0x40:
            dprint(1,"SD:CMD0 GO_IDLE_STATE\n");
            sdcard_resp_r1(sd, R1_OK);
            break;

        case 0x48:
            dprint(1,"SD:CMD8 SEND_IF_COND\n");

            // We accept any conditions (cmd[1],cmd[2],cmd[3])
            // And echo the check byte back (cmd[4])
            // Should check CRC (eg, 48000001aa87 is valid)
            int reply;
            reply = sd->cmd[1] << 24;
            reply |= sd->cmd[2] << 16;
            reply |= sd->cmd[3] << 8;
            reply |= sd->cmd[4];

            sdcard_resp_r7(sd, R1_OK, reply);
            break;

        case 0x49:
            dprint(1,"SD:CMD9 SEND_CSD\n");

            sd->resp[rp++] = 0x40; // CSD_STRUCT and reserved bits
            sd->resp[rp++] = 0x0e; // TAAC
            sd->resp[rp++] = 0x00; // NSAC
            sd->resp[rp++] = 0x32; // TRAN_SPEED 24MHz
            sd->resp[rp++] = 0x5b; // CCC - guess
            sd->resp[rp++] = 0x59; // CCC, READ_BL_LEN
            sd->resp[rp++] = 0x00; // 4 bits flags, 4 bits reserved
            sd->resp[rp++] = 0x00; // 2 bits reserved, 6 bits LBA
            sd->resp[rp++] = 0x08; // .
            sd->resp[rp++] = 0x00; // Approx 1Gig
            sd->resp[rp++] = 0x7f;
            sd->resp[rp++] = 0x80;
            sd->resp[rp++] = 0x0a;
            sd->resp[rp++] = 0x40;
            sd->resp[rp++] = 0x00;
            sd->resp[rp++] = 0x01; // last bit is stop bit
            // TODO: use real backing store for size

            sdcard_resp_tx_block(sd, R1_0,16);
            break;

        case 0x4a:
            dprint(1,"SD:CMD10 SEND_CID\n");

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
            sd->resp[rp++] = 0x01; // m=1
            sd->resp[rp++] = 0x01; // y=2001
            sd->resp[rp++] = 0x01; // last bit is stop bit

            sdcard_resp_tx_block(sd, R1_0, 16);
            break;

        case 0x50:
            dprint(1,"SD:CMD16 SET_BLOCKLEN\n");
            // TODO: do a real check of the value
            if (sd->cmd[3] != 2) {
                dprint(1,"SD: unexpected blocklen\n");
                sdcard_resp_r1(sd, R1_ILL_CMD);
                break;
            }
            sdcard_resp_r1(sd, R1_OK);
            break;

        case 0x51: {
            dprint(1,"SD:CMD17 READ_SINGLE_BLOCK\n");
            int block = (
                sd->cmd[1]<<24 |
                sd->cmd[2]<<16 |
                sd->cmd[3]<<8 |
                sd->cmd[4]
            );
            dprint(1,"SD:READ:  0x%04x\n", block);

            if (lseek(sd->fd, block*0x200, SEEK_SET) != -1) {
                read(sd->fd, &sd->resp, 0x200);
            }

            // TODO: could use result of read as source of r1 status
            sdcard_resp_tx_block(sd, R1_0, 512);
            break;
        }

        case 0x58: {
            dprint(1,"SD:CMD24 WRITE_BLOCK\n");
            int block = (
                sd->cmd[1]<<24 |
                sd->cmd[2]<<16 |
                sd->cmd[3]<<8 |
                sd->cmd[4]
            );
            dprint(1,"SD:WRITE: 0x%04x\n", block);

            sdcard_resp_rx_block(sd);
            break;
        }

        case 0x77:
            dprint(1,"SD:CMD55 APP_CMD\n");
            sdcard_resp_r1(sd, R1_OK);
            break;

        case 0x7a:
            dprint(1,"SD:CMD58 READ_OCR\n");
            sdcard_resp_r3(sd, R1_0, 0x40000000);
            break;

        // TODO: gate ACMD values on a previous CMD55 APP_CMD
        case 0x69:
            dprint(1,"SD:ACMD41 SEND_OP_COND\n");
            sdcard_resp_r1(sd, R1_0);
            break;

        case 0x73:
            dprint(1,"SD:ACMD51 SEND_SCR\n");
            sd->resp[rp++] = 0x01; // SD_SPEC v1.10
            sd->resp[rp++] = 0xB1; // SD_SEC == SDHC, 1bit width
            sd->resp[rp++] = 0x00;
            sd->resp[rp++] = 0x00;
            sd->resp[rp++] = 0x0c; // CMD_SUPPORT 58,59 + 48/49
            sd->resp[rp++] = 0x00;
            sd->resp[rp++] = 0x00;
            sd->resp[rp++] = 0x00;
            sdcard_resp_tx_block(sd, R1_0, 8);
            break;

        default:
            sdcard_resp_r1(sd, R1_ILL_CMD);

            dprint(1,"unknown\n");
            dprint(1,"SD:CMD:   cmd=0x%02x\n",cmd);
            break;
    }
    if (sdcard_trace>1) {
        printf("\n");
    }
    return 0xff;
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

    switch (sd->state) {
        case IDLE:
            // in idle mode, the card just outputs ones?
            dprint(1,"SD: read while IDLE\n");
            result = 0xff;
            break;

        case RX_CMD:
            // got a read when we expected more writes
            sdcard_reset(sd);
            dprint(1,"SD: read when RX_CMD\n");
            break;

        case TX_BUSY:
            result = 0xff; // first resp, indicate busy
            sdcard_setstate(sd, NEXT);
            break;

        case TX_R3:
        case TX_R7:
            sd->state_next = TX_IMM32;
            /* FALL THROUGH */

        case TX_R1:
            result = sd->r1; // second, return response code
            sdcard_setstate(sd, NEXT);
            break;

        case TX_IMM32:
            result = sd->resp[sd->resp_ptr];
            sd->resp_ptr++;

            if (sd->resp_ptr==4) {
                sdcard_setstate(sd, IDLE);
            }

            break;

        // TODO? with a deeper queue of state_next values, the TX_BLOCK
        // could be simpler
        case TX_BLOCK_R1:
            result = sd->r1;
            sdcard_setstate(sd, TX_BLOCK_TOKEN);
            break;

        case TX_BLOCK_TOKEN:
            result = 0xfe; // data token
            sdcard_setstate(sd, TX_BLOCK_BUF);
            break;

        case TX_BLOCK_BUF: {
            int index = sd->resp_ptr;

            if (index < sd->tx_len) {
                // Send the actual data
                result = sd->resp[index];
            } else if (index == sd->tx_len) {
                result = 0; // CRC1;
            } else {
                result = 0; // CRC2;
                sdcard_setstate(sd, IDLE);
            }
            sd->resp_ptr++;
            break;
        }

        case TX_RX_BLOCK_R1:
            result = sd->r1;
            sdcard_reset_ptr(sd);
            sdcard_setstate(sd, RX_BLOCK);
            break;

        case TX_RX_BLOCK_STAT:
            result = sd->r1;
            sdcard_setstate(sd, IDLE);
            break;

        default:
            dprint(1,"SD:RD: unexpected state %i\n", sd->state);
            result = 0xff;
            break;
    }

    if (sdcard_trace>1) {
        printf("SD:RD:    r=0x%02x\n", result);
    }
    return result;
}

