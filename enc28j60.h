#ifndef _DEV_SPI_NODE_ENC28J60_H_
#define _DEV_SPI_NODE_ENC28J60_H_
/*
 * Copyright (C) 2015 by Ole Reinhardt <ole.reinhardt@embedded-it.de>
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holders nor the names of
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF
 * THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 * For additional information see http://www.ethernut.de/
 */

/*!
 * \file include/dev/spi_node_at25df.h
 * \brief Low level access for Atmel AT25DF SPI DataFlash.
 *
 * \verbatim
 * $Id$
 * \endverbatim
 */

#include <dev/spibus.h>

/**
 * ENC28J60 RX/TX Buffer size
 */
#define ENC28J60_RXTXBUF_SIZE     0x1FFF      // ENC28J60 has 8K RAM to be split up into RX/TX
#define ETHFRAME_SIZE             0x05F6

/**
 * ENC28J60 SPI-Commands
 */
#define ENC28J60_OPCODE_RCR       0b00000000  // Read Control Register
#define ENC28J60_OPCODE_RBM       0b00000001  // Read Buffer Memody
  #define ENC28J60_ARGUMENT_RBM     0b00011010  // Argument for RBM-Command
#define ENC28J60_OPCODE_WCR       0b00000010  // Write Control Register
#define ENC28J60_OPCODE_WBM       0b00000011  // Write Buffer Memory
  #define ENC28J60_ARGUMENT_WBM     0b00011010  // Argument for WBM-Command
#define ENC28J60_OPCODE_BFS       0b00000100  // Bit Field Set
#define ENC28J60_OPCODE_BFC       0b00000101  // Bit Field Clear
#define ENC28J60_OPCODE_SRC       0b00000111  // System Reset Command
  #define ENC28J60_ARGUMENT_SRC     0b00011111  // Argument for SRC

/**
 * ENC28J60 Register addresses
 */
#define BNKENCODE(b, a)           ((b << 5) | a) // Encode the bank into the register address

// Raw register addresses (no bank encoded)
#define ENC28J60_RAWREG_ECON1     0x1F
#define ENC28J60_RAWREG_ECON2     0x1E
#define ENC28J60_RAWREG_ESTAT     0x1D
#define ENC28J60_RAWREG_EIR       0x1C
#define ENC28J60_RAWREG_EIE       0x1B

// MII Interface registers
#define ENC28J60_REG_MICMD        BNKENCODE(2, 0x12)
#define ENC28J60_REG_MISTAT       BNKENCODE(3, 0x0A)
#define ENC28J60_REG_MIREGADR     BNKENCODE(2, 0x14)
#define ENC28J60_REG_MIRDL        BNKENCODE(2, 0x18)
#define ENC28J60_REG_MIRDH        BNKENCODE(2, 0x19)
#define ENC28J60_REG_MIWRL        BNKENCODE(2, 0x16)
#define ENC28J60_REG_MIWRH        BNKENCODE(2, 0x17)

// Other registers
#define ENC28J60_REG_ERDPTL       BNKENCODE(0, 0x00)
#define ENC28J60_REG_ERDPTH       BNKENCODE(0, 0x01)
#define ENC28J60_REG_EWRPTL       BNKENCODE(0, 0x02)
#define ENC28J60_REG_EWRPTH       BNKENCODE(0, 0x03)
#define ENC28J60_REG_ETXSTL       BNKENCODE(0, 0x04)
#define ENC28J60_REG_ETXSTH       BNKENCODE(0, 0x05)
#define ENC28J60_REG_ETXNDL       BNKENCODE(0, 0x06)
#define ENC28J60_REG_ETXNDH       BNKENCODE(0, 0x07)
#define ENC28J60_REG_ERXSTL       BNKENCODE(0, 0x08)
#define ENC28J60_REG_ERXSTH       BNKENCODE(0, 0x09)
#define ENC28J60_REG_ERXNDL       BNKENCODE(0, 0x0A)
#define ENC28J60_REG_ERXNDH       BNKENCODE(0, 0x0B)
#define ENC28J60_REG_ERXRDPTL     BNKENCODE(0, 0x0C)
#define ENC28J60_REG_ERXRDPTH     BNKENCODE(0, 0x0D)
#define ENC28J60_REG_ERXWRPTL     BNKENCODE(0, 0x0E)
#define ENC28J60_REG_ERXWRPTH     BNKENCODE(0, 0x0F)
#define ENC28J60_REG_EDMASTL      BNKENCODE(0, 0x10)
#define ENC28J60_REG_EDMASTH      BNKENCODE(0, 0x11)
#define ENC28J60_REG_EDMANDL      BNKENCODE(0, 0x12)
#define ENC28J60_REG_EDMANDH      BNKENCODE(0, 0x13)
#define ENC28J60_REG_EDMADSTL     BNKENCODE(0, 0x14)
#define ENC28J60_REG_EDMADSTH     BNKENCODE(0, 0x15)
#define ENC28J60_REG_EDMACSL      BNKENCODE(0, 0x16)
#define ENC28J60_REG_EDMACSH      BNKENCODE(0, 0x17)
#define ENC28J60_REG_EHT0         BNKENCODE(1, 0x00)
#define ENC28J60_REG_EHT1         BNKENCODE(1, 0x01)
#define ENC28J60_REG_EHT2         BNKENCODE(1, 0x02)
#define ENC28J60_REG_EHT3         BNKENCODE(1, 0x03)
#define ENC28J60_REG_EHT4         BNKENCODE(1, 0x04)
#define ENC28J60_REG_EHT5         BNKENCODE(1, 0x05)
#define ENC28J60_REG_EHT6         BNKENCODE(1, 0x06)
#define ENC28J60_REG_EHT7         BNKENCODE(1, 0x07)
#define ENC28J60_REG_EPMM0        BNKENCODE(1, 0x08)
#define ENC28J60_REG_EPMM1        BNKENCODE(1, 0x09)
#define ENC28J60_REG_EPMM2        BNKENCODE(1, 0x0A)
#define ENC28J60_REG_EPMM3        BNKENCODE(1, 0x0B)
#define ENC28J60_REG_EPMM4        BNKENCODE(1, 0x0C)
#define ENC28J60_REG_EPMM5        BNKENCODE(1, 0x0D)
#define ENC28J60_REG_EPMM6        BNKENCODE(1, 0x0E)
#define ENC28J60_REG_EPMM7        BNKENCODE(1, 0x0F)
#define ENC28J60_REG_EPMCSL       BNKENCODE(1, 0x10)
#define ENC28J60_REG_EPMCSH       BNKENCODE(1, 0x11)
#define ENC28J60_REG_EPMOL        BNKENCODE(1, 0x14)
#define ENC28J60_REG_EPMOH        BNKENCODE(1, 0x15)
#define ENC28J60_REG_ERXFCON      BNKENCODE(1, 0x18)
#define ENC28J60_REG_EPKTCNT      BNKENCODE(1, 0x19)
#define ENC28J60_REG_MACON1       BNKENCODE(2, 0x00)
#define ENC28J60_REG_MACON3       BNKENCODE(2, 0x02)
#define ENC28J60_REG_MACON4       BNKENCODE(2, 0x03)
#define ENC28J60_REG_MABBIPG      BNKENCODE(2, 0x04)
#define ENC28J60_REG_MAIPGL       BNKENCODE(2, 0x06)
#define ENC28J60_REG_MAIPGH       BNKENCODE(2, 0x07)
#define ENC28J60_REG_MACLCON1     BNKENCODE(2, 0x08)
#define ENC28J60_REG_MACLCON2     BNKENCODE(2, 0x09)
#define ENC28J60_REG_MAMXFLL      BNKENCODE(2, 0x0A)
#define ENC28J60_REG_MAMXFLH      BNKENCODE(2, 0x0B)
#define ENC28J60_REG_MAADR5       BNKENCODE(3, 0x00)
#define ENC28J60_REG_MAADR6       BNKENCODE(3, 0x01)
#define ENC28J60_REG_MAADR3       BNKENCODE(3, 0x02)
#define ENC28J60_REG_MAADR4       BNKENCODE(3, 0x03)
#define ENC28J60_REG_MAADR1       BNKENCODE(3, 0x04)
#define ENC28J60_REG_MAADR2       BNKENCODE(3, 0x05)
#define ENC28J60_REG_EBSTSD       BNKENCODE(3, 0x06)
#define ENC28J60_REG_EBSTCON      BNKENCODE(3, 0x07)
#define ENC28J60_REG_EBSTCSL      BNKENCODE(3, 0x08)
#define ENC28J60_REG_EBSTCSH      BNKENCODE(3, 0x09)
#define ENC28J60_REG_EREVID       BNKENCODE(3, 0x12)
#define ENC28J60_REG_ECOCON       BNKENCODE(3, 0x15)
#define ENC28J60_REG_EFLOCON      BNKENCODE(3, 0x17)
#define ENC28J60_REG_EPAUSL       BNKENCODE(3, 0x18)
#define ENC28J60_REG_EPAUSH       BNKENCODE(3, 0x19)

// For Mac initialization
#define ENC28J60_BIT_MARXEN       0x00
#define ENC28J60_BIT_RXPAUS       0x02
#define ENC28J60_BIT_TXPAUS       0x03

#define ENC28J60_BIT_FULDPX       0x00
#define ENC28J60_BIT_FRMLNEN      0x01
#define ENC28J60_BIT_TXCRCEN      0x04
#define ENC28J60_BIT_PADCFG0      0x05
#define ENC28J60_BIT_PADCFG1      0x06
#define ENC28J60_BIT_PADCFG2      0x07

#define ENC28J60_BIT_DEFER        0x06

// EIE-Register
#define ENC28J60_BIT_RXERIE       0x00
#define ENC28J60_BIT_TXERIE       0x01
#define ENC28J60_BIT_TXIE         0x03
#define ENC28J60_BIT_LINKIE       0x04
#define ENC28J60_BIT_DEMAIE       0x05
#define ENC28J60_BIT_PKTIE        0x06
#define ENC28J60_BIT_INTIE        0x07

// EIR-Register
#define ENC28J60_BIT_RXERIF       0x00
#define ENC28J60_BIT_TXERIF       0x01
#define ENC28J60_BIT_TXIF         0x03
#define ENC28J60_BIT_LINKIF       0x04
#define ENC28J60_BIT_DEMAIF       0x05
#define ENC28J60_BIT_PKTIF        0x06

#define ENC28J60_BIT_TXIF         0x03

// ECON1-Register
#define ENC28J60_BIT_BSEL0        0x00
#define ENC28J60_BIT_BSEL1        0x01
#define ENC28J60_BIT_RXEN         0x02
#define ENC28J60_BIT_TXRTS        0x03
#define ENC28J60_BIT_CSUMEN       0x04
#define ENC28J60_BIT_DMAST        0x05
#define ENC28J60_BIT_RXRST        0x06
#define ENC28J60_BIT_TXRST        0x07

// ECON2-Register
#define ENC28J60_BIT_PKTDEC       0x06

// ERXFCON-Register
#define ENC28J60_BIT_BCEN         0x00
#define ENC28J60_BIT_MCEN         0x01
#define ENC28J60_BIT_HTEN         0x02
#define ENC28J60_BIT_MPEN         0x03
#define ENC28J60_BIT_PMEN         0x04
#define ENC28J60_BIT_CRCEN        0x05
#define ENC28J60_BIT_ANDOR        0x06
#define ENC28J60_BIT_UCEN         0x07

#define ENC28J60_STATVEC_RECV_OK  23

#define ENC28J60_DEFCONTENT_EIR   0x00

extern NUTSPINODE nodeENC28J60;

extern int NutRegisterEnc28j60Spi(NUTDEVICE * dev, NUTSPIBUS * bus, int cs);

/*
 * Available drivers.
 */
extern NUTDEVICE devENC28J60;

#ifndef DEV_ETHER
#define DEV_ETHER   devENC28J60
#endif



#endif
