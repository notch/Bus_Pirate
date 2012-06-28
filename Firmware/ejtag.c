/*
 * Written in 2012 by notch <notch@cryptodrunks.net>
 *
 * To the extent possible under law, the author(s) have dedicated all copyright
 * and related and neighboring rights to this software to the public domain
 * worldwide. This software is distributed without any warranty.
 *
 * You should have received a copy of the CC0 Public Domain Dedication along
 * with this software. If not, see
 * <http://creativecommons.org/publicdomain/zero/1.0/>.
 */

#include "base.h"
#include "ejtag.h"

struct _ejtagConfig ejtagConfig;


/*
 * TAP shift function from OpenOCD mode
 * in_buf even bytes are TDI, odd bytes are TMS (index starts at 0 - even)
 * lsb gets shifted first
 */
extern void binOpenOCDTapShiftFast(unsigned char *in_buf, unsigned char *out_buf, unsigned int bits, unsigned int delay);

static void ejtagReset(void) {
    unsigned char inbuf[2];
    unsigned char outbuf[2]; // OCDTapShiftFast() need at least 2 bytes in both buffers
    /* Five clocks with TMS high should move tap to Test-Logic-Reset state,
     * whatever the starting state was. Then one clock with TMS low will put
     * tap in Run-Test/Idle
     */
    ejtagConfig.currentInstr = 0;
    inbuf[0] = 0x00;
    inbuf[1] = 0x1F; /* 0b0001 1111 */
    binOpenOCDTapShiftFast(inbuf, outbuf, 6, ejtagConfig.delay);
}

static unsigned char ejtagDetectIrLength(void) {
    unsigned char inbuf[4];
    unsigned char outbuf[2];
    unsigned char i, ret;

    ejtagReset();

    // go to shift-ir
    inbuf[0] = 0x00;
    inbuf[1] = 0x03;
    binOpenOCDTapShiftFast(inbuf, outbuf, 4, ejtagConfig.delay);

    // fill register with 0's
    // FIXME: IR longer than 15 not supported so far
    inbuf[0] = 0x00;
    inbuf[1] = 0x00;
    inbuf[2] = 0x00;
    inbuf[3] = 0x00;
    binOpenOCDTapShiftFast(inbuf, outbuf, 16, ejtagConfig.delay);

    // fill register with 1's
    inbuf[0] = 0xff;
    inbuf[1] = 0x00;
    inbuf[2] = 0xff;
    inbuf[3] = 0x00;
    binOpenOCDTapShiftFast(inbuf, outbuf, 16, ejtagConfig.delay);

    // spot first 1
    for (i = 0; i < 16; i++) {
        if ((outbuf[i / 8] >> (i % 8)) & 1)
            break;
    }
    if (i == 16)
        ret = 0; // IR length bigger than 15
    else
        ret = i;

    ejtagReset();

    ejtagConfig.instrLen = ret;

    return ret;
}

static void ejtagSetInstruction(unsigned char instruction) {
    /* tms high for select-dr-scan  \
     * tms high for select-ir-scan   \ 4 bits
     * tms low for capture-ir        /
     * tms low for shift-ir         /
     * tms low while shifting the instruction                   \ instruction len bits
     * tms high for last bit of instruction to move to exit1-ir /
     * tms high for update-ir    \ 2 bits
     * tms low for run-test/idle /
     *
     * At the beginning tap should be in run-test/idle
     *
     * for now only support instruction lenght of 10 (16-4-2)
     * with long int we could simply support instrlen of 26 (32-4-2)
     * TODO: add support for instrlen > 10
     */
    unsigned int tmsdata;
    unsigned int tdodata;
    unsigned char inbuf[4];
    unsigned char outbuf[2];

    if (ejtagConfig.instrLen < 1)
        return; //FIXME: add error handling

    if (ejtagConfig.currentInstr == instruction)
        return;

    tdodata = instruction << 4;
    tmsdata = (0x03 << (4 + ejtagConfig.instrLen - 1)) | 0x03;

    inbuf[0] = (unsigned char) tdodata;
    inbuf[1] = (unsigned char) tmsdata;
    inbuf[2] = (unsigned char) (tdodata >> 8);
    inbuf[3] = (unsigned char) (tmsdata >> 8);

    binOpenOCDTapShiftFast(inbuf, outbuf, ejtagConfig.instrLen + 6, ejtagConfig.delay);
    ejtagConfig.currentInstr = instruction;

}

static void ejtagReadWrite(const unsigned char * indata, unsigned char * outdata) {
    /* tms high for select-dr-scan \
     * tms low for capture-dr       ) 3 bits
     * tms low for shift-dr        /
     * shift data with tms low                           \ sizeof(unsigned long int) 32 bits
     * tms high for last bit of data to move to exit1-dr /
     * tms high for update-dr    \ 2 bits
     * tms low for run-test/idle /
     *
     * FIXME: for now we pad the first byte with 5 zeros to align data
     * so total lenght is 5+3+32+2 = 42
     */
    int i;
    unsigned char inbuf[12], outbuf[6];

    /* prefix with 5 zeros, so real data starts on byte boundary */
    inbuf[0] = 0x00; /* 0b0 << 5 */
    inbuf[1] = 0x20; /* 0b001 << 5 */
    for (i = 0; i < 4; i++) {
        inbuf[2 * (i + 1)] = indata[i];
        inbuf[2 * (i + 1) + 1] = (i == 3) ? 0x80 : 0x00;
    }
    inbuf[10] = 0x00;
    inbuf[11] = 0x01;


    binOpenOCDTapShiftFast(inbuf, outbuf, 42, ejtagConfig.delay);

    for (i = 0; i < 4; i++)
        outdata[i] = outbuf[i + 1];
}

static void ejtagRead(unsigned char * outdata) {
    unsigned char tmp[4] = {0};
    ejtagReadWrite(tmp, outdata);
}

static void ejtagWrite(const unsigned char * indata) {
    unsigned char tmp[4];
    ejtagReadWrite(indata, tmp);
}

static void ejtagDmaRead(const unsigned char * addr, unsigned char * outdata, char size) {
    unsigned char tmp[4] __attribute__((aligned)); // align cause later we cast it to unsigned long
    unsigned char status[4] __attribute__((aligned)); // not sure if needed, but better safe than sorry
    unsigned char timeout = 5;

    unsigned char i;

    ejtagSetInstruction(EJTAG_INSTR_ADDRESS);
    ejtagWrite(addr);
    ejtagSetInstruction(EJTAG_INSTR_CONTROL);
    /*(EJTAG_BIT_PRACC | EJTAG_BIT_PROBEN | EJTAG_BIT_DMAACC | EJTAG_BIT_DSTRT | EJTAG_BIT_DRWN | EJTAG_DMA_WORD)*/
    /* 18 | 15 | 17 | 11 | 9 | 8 */
    // tmp[0] = 0x00; tmp[1] = 0x8B; tmp[2] = 0x06; tmp[3] = 0x00;
    switch (size) {
        case EJTAG_DMA_SIZE_BYTE:
            *(unsigned long *) tmp = (EJTAG_BIT_PRACC | EJTAG_BIT_PROBEN | EJTAG_BIT_DMAACC | EJTAG_BIT_DSTRT | EJTAG_BIT_DRWN | EJTAG_DMA_BYTE);
        case EJTAG_DMA_SIZE_HALFWORD:
            *(unsigned long *) tmp = (EJTAG_BIT_PRACC | EJTAG_BIT_PROBEN | EJTAG_BIT_DMAACC | EJTAG_BIT_DSTRT | EJTAG_BIT_DRWN | EJTAG_DMA_HALFWORD);
        case EJTAG_DMA_SIZE_WORD:
            *(unsigned long *) tmp = (EJTAG_BIT_PRACC | EJTAG_BIT_PROBEN | EJTAG_BIT_DMAACC | EJTAG_BIT_DSTRT | EJTAG_BIT_DRWN | EJTAG_DMA_WORD);
    }
    ejtagWrite(tmp);
    do {
        if (!timeout--)
            break;
        ejtagSetInstruction(EJTAG_INSTR_CONTROL);
        /*(EJTAG_BIT_PRACC | EJTAG_BIT_PROBEN | EJTAG_BIT_DMAACC) */
        /* 18 | 15 | 17 */
        //tmp[0] = 0x00; tmp[1] = 0x80; tmp[2] = 0x06; tmp[3] = 0x00;
        *(unsigned long *) tmp = (EJTAG_BIT_PRACC | EJTAG_BIT_PROBEN | EJTAG_BIT_DMAACC);
        ejtagReadWrite(tmp, status);
        //} while (status[1] & 0x08);
    } while ((*(unsigned long*) status) & EJTAG_BIT_DSTRT);

    ejtagSetInstruction(EJTAG_INSTR_DATA);
    ejtagRead(outdata);

    timeout = 5;
    do {
        if (!timeout--)
            break;
        ejtagSetInstruction(EJTAG_INSTR_CONTROL);
        /* (EJTAG_BIT_PRACC | EJTAG_BIT_PROBEN) */
        /* 18 | 15*/
        //tmp[0] = 0x00; tmp[1] = 0x80; tmp[2] = 0x04; tmp[3] = 0x00;
        *(unsigned long *) tmp = (EJTAG_BIT_PRACC | EJTAG_BIT_PROBEN);
        ejtagReadWrite(tmp, status);
        //} while (status[1] & 0x04);
    } while ((*(unsigned long *) status) & EJTAG_BIT_DERR);

    for (i=0; i<4; i++)
        UART1TX(outdata[i]);

    switch (size) {
        case EJTAG_DMA_SIZE_BYTE:
            if ((addr[0] & 3) == 3)
                outdata[0] = outdata[3];
            else if ((addr[0] & 3) == 2)
                outdata[0] = outdata[2];
            else if ((addr[0] & 3) == 1)
                outdata[0] = outdata[1];
            outdata[1] = 0;
            outdata[2] = 0;
            outdata[3] = 0;
            break;
        case EJTAG_DMA_SIZE_HALFWORD:
            if (addr[0] & 2) {
                outdata[0] = outdata[2];
                outdata[1] = outdata[3];
            }
            outdata[2] = 0;
            outdata[3] = 0;
            break;
        case EJTAG_DMA_SIZE_WORD:
        default:
            break;
    }
}

static void ejtagDmaWrite(const unsigned char * addr, unsigned char * data, char size) {
    unsigned char tmp[4] __attribute__((aligned));
    unsigned char status[4] __attribute__((aligned));
    unsigned char timeout = 5;

    /* prepare data */
    switch (size) {
        case EJTAG_DMA_SIZE_BYTE:
            data[1] = data[0];
            data[2] = data[0];
            data[3] = data[0];
            break;
        case EJTAG_DMA_SIZE_HALFWORD:
            data[2] = data[0];
            data[3] = data[1];
            break;
        case EJTAG_DMA_SIZE_WORD:
        default:
            break;
    }

    ejtagSetInstruction(EJTAG_INSTR_ADDRESS);
    ejtagWrite(addr);
    ejtagSetInstruction(EJTAG_INSTR_DATA);
    ejtagWrite(data);
    ejtagSetInstruction(EJTAG_INSTR_CONTROL);
    /* EJTAG_BIT_PRACC | EJTAG_BIT_PROBEN | EJTAG_BIT_DMAACC | EJTAG_BIT_DSTRT | EJTAG_DMA_WORD */
    /* 18 | 15 | 17 | 11 | 8*/
    //tmp[0] = 0x00; tmp[1] = 0x89; tmp[2] = 0x06; tmp[3] = 0x00;
    switch (size) {
        case EJTAG_DMA_SIZE_BYTE:
            *(unsigned long *) tmp = (EJTAG_BIT_PRACC | EJTAG_BIT_PROBEN | EJTAG_BIT_DMAACC | EJTAG_BIT_DSTRT | EJTAG_DMA_BYTE);
            break;
        case EJTAG_DMA_SIZE_HALFWORD:
            *(unsigned long *) tmp = (EJTAG_BIT_PRACC | EJTAG_BIT_PROBEN | EJTAG_BIT_DMAACC | EJTAG_BIT_DSTRT | EJTAG_DMA_HALFWORD);
            break;
        case EJTAG_DMA_SIZE_WORD:
            *(unsigned long *) tmp = (EJTAG_BIT_PRACC | EJTAG_BIT_PROBEN | EJTAG_BIT_DMAACC | EJTAG_BIT_DSTRT | EJTAG_DMA_WORD);
            break;
    }
    ejtagWrite(tmp);
    do {
        if (!timeout--)
            break;
        ejtagSetInstruction(EJTAG_INSTR_CONTROL);
        /* EJTAG_BIT_PRACC | EJTAG_BIT_PROBEN | EJTAG_BIT_DMAACC */
        /* 18 | 15 | 17 */
        //tmp[0] = 0x00; tmp[1] = 0x80; tmp[2] = 0x06; tmp[3] = 0x00;
        *(unsigned long *) tmp = (EJTAG_BIT_PRACC | EJTAG_BIT_PROBEN | EJTAG_BIT_DMAACC);
        ejtagReadWrite(tmp, status);
        //} while (status[1] * 0x08);
    } while ((*(unsigned long *) status) & EJTAG_BIT_DSTRT);

    timeout = 5;
    do {
        if (!timeout--)
            break;
        ejtagSetInstruction(EJTAG_INSTR_CONTROL);
        /* EJTAG_BIT_PRACC | EJTAG_BIT_PROBEN */
        /* 18 | 15*/
        //tmp[0] = 0x00; tmp[1] = 0x80; tmp[2] = 0x04; tmp[3] = 0x00;
        *(unsigned long *) tmp = (EJTAG_BIT_PRACC | EJTAG_BIT_PROBEN);
        ejtagReadWrite(tmp, status);
        //} while (status[1] & 0x04);
    } while ((*(unsigned long *) status) & EJTAG_BIT_DERR);

    return;
}

void ejtagDmaBusInit(void)
{
    unsigned char tmp[4] __attribute__((aligned));
    unsigned char status[4] __attribute__((aligned));
    unsigned char addr[4] __attribute__((aligned));
    unsigned char timeout = 5;
    unsigned char i;

    // The purpose of this is to make the processor break into debug mode on
    // reset rather than execute the reset vector
    ejtagSetInstruction( EJTAG_INSTR_EJTAGBOOT );

    ejtagSetInstruction( EJTAG_INSTR_CONTROL );
    // Reset the processor
    *(unsigned long *)tmp = ( EJTAG_BIT_PRRST | EJTAG_BIT_PERRST );
    ejtagWrite(tmp);

    // Release reset
    *(unsigned long *)tmp = 0;
    ejtagWrite(tmp);
    *(unsigned long *)tmp = ( EJTAG_BIT_PRACC | EJTAG_BIT_PROBEN | EJTAG_BIT_PROBTRAP | EJTAG_BIT_JTAGBRK | EJTAG_BIT_ROCC );
    ejtagWrite(tmp);

    /* Wait until processor is in break */
    *(unsigned long *)tmp &= ~EJTAG_BIT_JTAGBRK;
    do {
        ejtagReadWrite(tmp, status);
        if (!timeout--)
            break;
    } while ( ((*(unsigned long *)status) & EJTAG_BIT_BRKST) == 0 );
    if (timeout == 0) {
        /* TODO: handle errors*/
        UART1TX(0xFF);
    }

    // Handle the reset bit clear, if any
    if ( (*(unsigned long *)status) & EJTAG_BIT_ROCC) {
        *(unsigned long *)tmp &= ~ EJTAG_BIT_ROCC;
        ejtagWrite(tmp);
        *(unsigned long *)tmp |= EJTAG_BIT_ROCC;
        ejtagWrite(tmp);
    }

    // Clear Memory Protection Bit in DCR
    *(unsigned long *)addr = 0xff300000UL;
    ejtagDmaRead(addr, status, EJTAG_DMA_SIZE_WORD);
    status[0] &= ~(1 << 2);
    ejtagDmaWrite(addr, status, EJTAG_DMA_SIZE_WORD);

    // Clear watchdog, if any
    *(unsigned long *)addr = 0xb8000080UL;
    *(unsigned long *)tmp = 0;
    ejtagDmaWrite(addr, tmp, EJTAG_DMA_SIZE_WORD);

    // get potential flash base address
    *(unsigned long *)addr = 0xfffe2000UL;
    ejtagDmaRead(addr, tmp, EJTAG_DMA_SIZE_WORD);
    for (i=0; i<4; i++)
        UART1TX(tmp[i]);
    *(unsigned long *)addr = 0xfffe1000UL;
    ejtagDmaRead(addr, tmp, EJTAG_DMA_SIZE_WORD);
    for (i=0; i<4; i++)
        UART1TX(tmp[i]);
}

void ejtagSetup(void) {
    ejtagConfig.currentInstr = 0;
    ejtagConfig.delay = 1;
    ejtagConfig.instrLen = 0;

    EJTAG_PIN_TD0_TRIS = 1; //input from chain
    EJTAG_PIN_TCK_TRIS = 0;
    EJTAG_PIN_TDI_TRIS = 0;
    EJTAG_PIN_TMS_TRIS = 0;

    EJTAG_PIN_TDO = 0;
    EJTAG_PIN_TCK = 0;
    EJTAG_PIN_TDI = 0;
    EJTAG_PIN_TMS = 0;
}

void ejtag(void) {
    unsigned char c;
    unsigned char addr[4] __attribute__((aligned));
    unsigned char data[4] __attribute__((aligned));
    unsigned char outdata[4] __attribute__((aligned));
    unsigned int i;
    unsigned long int length, j;

    ejtagSetup();
    bpWstring("EJT1");

    while (1) {
        c = UART1RX();
        switch (c) {
            case 0:
                return;
            case 1: //EJATG_DETECT_IRLEN
                UART1TX(ejtagDetectIrLength());
                break;

            case 2: //EJTAG_CMD_SET_INSTR_LEN
                ejtagConfig.instrLen = UART1RX();
                break;
            case 3: //EJTAG_CMD_SET_DELAY
                c = UART1RX();
                ejtagConfig.delay = (c << 8) | UART1RX();
                break;
            case 4: //EJTAG_CMD_RESET
                ejtagReset();
                break;
            case 5: //EJTAG_CMD_SET_INSTR
                c = UART1RX();
                ejtagSetInstruction(c);
                break;
            case 6: //EJTAG_CMD_READWRITE
                for (i = 0; i < 4; i++) {
                    data[i] = UART1RX();
                }
                ejtagReadWrite(data, outdata);
                for (i = 0; i < 4; i++) {
                    UART1TX(outdata[i]);
                }
                break;
                ;
            case 7: //EJTAG_CMD_DMA_READ_WORD
                for (i = 0; i < 4; i++) {
                    addr[i] = UART1RX();
                }
                ejtagDmaRead(addr, outdata, EJTAG_DMA_SIZE_WORD);
                for (i = 0; i < 4; i++) {
                    UART1TX(outdata[i]);
                }
                break;
            case 8: //EJTAG_CMD_DMA_READ_HALFWORD
                for (i = 0; i < 4; i++) {
                    addr[i] = UART1RX();
                }
                ejtagDmaRead(addr, outdata, EJTAG_DMA_SIZE_HALFWORD);
                for (i = 0; i < 4; i++) {
                    UART1TX(outdata[i]);
                }
                break;
            case 9: //EJTAG_CMD_DMA_READ_BYTE
                for (i = 0; i < 4; i++) {
                    addr[i] = UART1RX();
                }
                ejtagDmaRead(addr, outdata, EJTAG_DMA_SIZE_BYTE);
                for (i = 0; i < 4; i++) {
                    UART1TX(outdata[i]);
                }
                break;
            case 10: //EJTAG_CMD_DMA_WRITE_WORD
                for (i = 0; i < 4; i++) {
                    addr[i] = UART1RX();
                }
                for (i = 0; i < 4; i++) {
                    data[i] = UART1RX();
                }
                ejtagDmaWrite(addr, data, EJTAG_DMA_SIZE_WORD);
                break;
            case 11: //EJTAG_CMD_DMA_WRITE_HALFWORD
                for (i = 0; i < 4; i++) {
                    addr[i] = UART1RX();
                }
                for (i = 0; i < 4; i++) {
                    data[i] = UART1RX();
                }
                ejtagDmaWrite(addr, data, EJTAG_DMA_SIZE_HALFWORD);
                break;
            case 12: //EJTAG_CMD_DMA_WRITE_BYTE
                for (i = 0; i < 4; i++) {
                    addr[i] = UART1RX();
                }
                for (i = 0; i < 4; i++) {
                    data[i] = UART1RX();
                }
                ejtagDmaWrite(addr, data, EJTAG_DMA_SIZE_BYTE);
                break;
            case 13: //EJTAG_READ_BATCH
                for (i = 0; i < 4; i++)
                    addr[i] = UART1RX();
                for (i = 0, length = 0; i < 4; i++) {
                    length <<= 8;
                    length |= UART1RX();
                }
                for (j = 0; j < length; j += 4) {
                    ejtagDmaRead(addr, outdata, EJTAG_DMA_SIZE_WORD);
                    for (i = 0; i < 4; i++)
                        UART1TX(outdata[i]);
                    *(unsigned long *) addr += 4;
                }
                break;
            case 14: //EJTAG_DMA_BUS_INIT
                ejtagDmaBusInit();
                break;
            default:
                break;
        }
    }
}

