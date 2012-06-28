#ifndef EJTAG_H
#define EJTAG_H

#define EJTAG_PIN_TDO	BP_MISO
#define EJTAG_PIN_TCK	BP_CLK
#define EJTAG_PIN_TDI	BP_MOSI
#define EJTAG_PIN_TMS	BP_CS

#define EJTAG_PIN_TD0_TRIS	BP_MISO_DIR
#define EJTAG_PIN_TCK_TRIS	BP_CLK_DIR
#define EJTAG_PIN_TDI_TRIS	BP_MOSI_DIR
#define EJTAG_PIN_TMS_TRIS	BP_CS_DIR

#define EJTAG_INSTR_IDCODE	0x01
#define EJTAG_INSTR_IMPCODE	0x03
#define EJTAG_INSTR_ADDRESS	0x08
#define EJTAG_INSTR_DATA	0x09
#define EJTAG_INSTR_CONTROL	0x0A
#define EJTAG_INSTR_ALL		0x0B
#define EJTAG_INSTR_EJTAGBOOT	0x0C
#define EJTAG_INSTR_NORMALBOOT	0x0D
#define EJTAG_INSTR_FASTDATA	0x0E
#define EJTAG_INSTR_TCBCONTROLA	0x10
#define EJTAG_INSTR_TCBCONTROLB	0x11
#define EJTAG_INSTR_TCBDATA	0x12
#define EJTAG_INSTR_TCBCONTROLC	0x13
#define EJTAG_INSTR_PCSAMPLE	0x14
#define EJTAG_INSTR_TCBCONTROLD	0x15
#define EJTAG_INSTR_TCBCONTROLE	0x16
#define EJTAG_INSTR_FDC		0x17
#define EJTAG_INSTR_BYPASS	0xFF

#define EJTAG_BIT_TOF		(1UL << 1 )
#define EJTAG_BIT_TIF		(1UL << 2 )
#define EJTAG_BIT_BRKST		(1UL << 3 )
#define EJTAG_BIT_DLOCK		(1UL << 5 )
#define EJTAG_BIT_DRWN		(1UL << 9 )
#define EJTAG_BIT_DERR		(1UL << 10)
#define EJTAG_BIT_DSTRT		(1UL << 11)
#define EJTAG_BIT_JTAGBRK	(1UL << 12)
#define EJTAG_BIT_PROBTRAP	(1UL << 14)
#define EJTAG_BIT_PROBEN	(1UL << 15)
#define EJTAG_BIT_PRRST		(1UL << 16)
#define EJTAG_BIT_DMAACC	(1UL << 17)
#define EJTAG_BIT_PRACC		(1UL << 18)
#define EJTAG_BIT_PRNW		(1UL << 19)
#define EJTAG_BIT_PERRST	(1UL << 20)
#define EJTAG_BIT_HALT		(1UL << 21)
#define EJTAG_BIT_DOZE		(1UL << 22)
#define EJTAG_BIT_SYNC		(1UL << 23)
#define EJTAG_BIT_DNM		(1UL << 28)
#define EJTAG_BIT_ROCC		(1UL << 31)

#define EJTAG_DMA_BYTE		0x00000000UL  //DMA tranfser size BYTE
#define EJTAG_DMA_HALFWORD	0x00000080UL  //DMA transfer size HALFWORD
#define EJTAG_DMA_WORD		0x00000100UL  //DMA transfer size WORD
#define EJTAG_DMA_TRIPLEBYTE	0x00000180UL  //DMA transfer size TRIPLEBYTE

#define EJTAG_DMA_SIZE_WORD     0
#define EJTAG_DMA_SIZE_HALFWORD 1
#define EJTAG_DMA_SIZE_BYTE     2

struct _ejtagConfig {
	unsigned int delay; /* TODO: init all*/
	unsigned char instrLen;
	unsigned char currentInstr;
};

void ejtag(void);

#endif
