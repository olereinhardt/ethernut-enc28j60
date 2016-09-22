/*
 * Copyright (C) 2016 by Leon Schürmann <leon-git@lschuermann.xyz>
 * Copyright (C) 2016 by Ole Reinhardt <ole.reinhardt@embedded-it.de>
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
 * \file dev/enc28j60.c
 * \brief Driver for SPI-Connected Ethernet-chip ENC28J60
 *
 * \verbatim
 * $Id$
 * \endverbatim
 */

// TODO: Remove following line
#include <stdio.h>

#include <sys/nutdebug.h>
#include <sys/timer.h>

#include <stdlib.h>
#include <string.h>

#include <netinet/if_ether.h>
#include <net/ether.h>
#include <net/if.h>
#include <net/if_var.h>

#include <dev/gpio.h>
#include <dev/irqreg.h>

#include "enc28j60.h"

/*@{*/

#ifdef NUTDEBUG
#include <stdio.h>
#include <arpa/inet.h>
#define EMPRINTF(args,...) printf(args,##__VA_ARGS__);
#else
#define EMPRINTF(args,...)
#endif

#ifdef ENC28J60_TXBUF_SIZE
  #if ENC28J60_TXBUF_SIZE > (ENC28J60_RXTXBUF_SIZE - ETHFRAME_SIZE)
    #error The tx-buffer is to small to fit a single ethernet-packet.
  #endif
#else
  #warning The tx-buffer size should be specified. Making it as big as 1 ethernet packet for now
  #define ENC28J60_TXBUF_SIZE ETHFRAME_SIZE
#endif

#ifdef SPI_ENC28J60_CLK
  #if SPI_ENC28J60_CLK > 20000000
    #error The spi-baudrate for ENC28J60 cant be higher than 20Mhz.
  #endif
#else
  // Using 8Mhz as a safe default speed
  #define SPI_ENC28J60_CLK 100000
#endif

#ifndef NUT_THREAD_NICRXSTACK
#define NUT_THREAD_NICRXSTACK   2048
#endif


// -----------------------------------------------------------------------------

typedef struct _ENC28J60_DCB {
  IFNET * ifeth;
  NUTSPINODE * node;
  uint8_t bank;
  uint16_t tx_space_used;
  GPIO_SIGNAL * gpioIrqSig;
  uint16_t rxNextPacketPointer;
#ifdef NUT_PERFMON
#warning not yet implemented
  uint32_t ni_rx_packets;       /*!< Number of packets received. */
  uint32_t ni_tx_packets;       /*!< Number of packets sent. */
  uint32_t ni_overruns;         /*!< Number of packet overruns. */
  uint32_t ni_rx_frame_errors;  /*!< Number of frame errors. */
  uint32_t ni_rx_crc_errors;    /*!< Number of CRC errors. */
  uint32_t ni_rx_missed_errors; /*!< Number of missed packets. */
#endif
  HANDLE volatile ni_rx_rdy;    /*!< Receiver event queue. */
  HANDLE volatile ni_tx_rdy;    /*!< Transmitter event queue. */
  HANDLE ni_mutex;              /*!< Access mutex semaphore. */
  HANDLE interruptReady;
} ENC28J60_DCB;

// -----------------------------------------------------------------------------

int Enc28j60AllocateBus(NUTSPINODE * node) {
  NUTSPIBUS *bus;

  /* Sanity checks. */
  NUTASSERT(node != NULL);
  bus = (NUTSPIBUS *) node->node_bus;
  NUTASSERT(bus != NULL);
  NUTASSERT(bus->bus_alloc != NULL);
  NUTASSERT(bus->bus_transfer != NULL);
  NUTASSERT(bus->bus_release != NULL);

  return (*bus->bus_alloc) (node, 0);
}

int Enc28j60ReleaseBus(NUTSPINODE * node) {
  return (*((NUTSPIBUS *) node->node_bus)->bus_release) (node);
}

int Enc28j60ReallocateBus(NUTSPINODE * node) {
  return Enc28j60ReleaseBus(node) & Enc28j60AllocateBus(node);
}

// -----------------------------------------------------------------------------

uint8_t getEncodedBank(uint8_t byte) {
  return (byte & (3 << 5)) >> 5;
}

// -----------------------------------------------------------------------------

int Enc28j60TransmitCommand(NUTSPINODE * node, uint8_t op, uint8_t arg) {
  /**
   * Commands for ENC have the follwing scheme:
   *     <Opcode> |  <Argument>
   * 0b  1  0  1  |  1  1  0  0  0
   */

  // op can be max. 3 bits, cut everything else off
  op &= 0b00000111;
  // same for arg, just for 5 bits
  arg &= 0b00011111;

  uint8_t command = (op << 5) | arg;

  return (*((NUTSPIBUS *) node->node_bus)->bus_transfer) (node, &command, NULL, 1);
}

int Enc28j60WriteData(NUTSPINODE * node, uint8_t * data, size_t length) {
  return (*((NUTSPIBUS *) node->node_bus)->bus_transfer) (node, data, NULL, length);
}

int Enc28j60ReadData(NUTSPINODE * node, uint8_t * rx_buf, size_t length) {
  return (*((NUTSPIBUS *) node->node_bus)->bus_transfer) (node, NULL, rx_buf, length);
}

int Enc28j60SelectBank(NUTSPINODE * node, uint8_t bank) {
  ENC28J60_DCB * dcb = (ENC28J60_DCB *) node->node_dcb;
  if (bank != dcb->bank) {
    Enc28j60AllocateBus(node);
    // Bank differs from previous one, set it

    // Old register values should be kept, store them in a buffer
    // Cant use fancy ReadControlRegister as it relies on this method
    Enc28j60TransmitCommand(node, ENC28J60_OPCODE_RCR, ENC28J60_RAWREG_ECON1);
    uint8_t data[1];
    Enc28j60ReadData(node, data, 1);

    // Reallocate bus to let ENC accept a new command
    Enc28j60ReallocateBus(node);

    // Modify data to point to correct bank
    // Be safe in that the bank-value has to be in between 0 and 3
    if (bank < 0 || bank > 3) {
      printf("Error [Enc28j60SelectBank]: bank has an invalid value (%i), using 0 instead!", bank);
      bank = 0;
    }

    // Clear bank fields
    data[0] &= 0b11111100;

    // Select bank
    data[0] |= bank;

    // Write register
    Enc28j60TransmitCommand(node, ENC28J60_OPCODE_WCR, ENC28J60_RAWREG_ECON1);
    Enc28j60WriteData(node, data, 1);

    Enc28j60ReleaseBus(node);

    // Remember currently selected bank
    dcb->bank = bank;
  }
  return 0;
}

int Enc28j60ReadControlRegister(NUTSPINODE * node, uint8_t reg, uint8_t * rx_buf) {
  Enc28j60SelectBank(node, getEncodedBank(reg));

  Enc28j60AllocateBus(node);
  Enc28j60TransmitCommand(node, ENC28J60_OPCODE_RCR, reg);
  Enc28j60ReadData(node, rx_buf, 1);
  Enc28j60ReleaseBus(node);

  return 0;
}

int Enc28j60WriteControlRegister(NUTSPINODE * node, uint8_t reg, uint8_t * tx_buf) {
  Enc28j60SelectBank(node, getEncodedBank(reg));

  Enc28j60AllocateBus(node);
  Enc28j60TransmitCommand(node, ENC28J60_OPCODE_WCR, reg);
  Enc28j60WriteData(node, tx_buf, 1);
  Enc28j60ReleaseBus(node);

  return 0;
}

/*
 * Bit Field set works by doing a bitwise-OR (non-exclusive) on the register
 * with 'data'.
 */
int Enc28j60BitFieldSet(NUTSPINODE * node, uint8_t reg, uint8_t data) {
  Enc28j60SelectBank(node, getEncodedBank(reg));

  Enc28j60AllocateBus(node);
  Enc28j60TransmitCommand(node, ENC28J60_OPCODE_BFS, reg);
  Enc28j60WriteData(node, &data, 1);
  Enc28j60ReleaseBus(node);

  return 0;
}

/*
 * Bit Field clear works by inverting 'data' and doing a bitwise-AND with the
 * inverted byte onto the register.
 */
int Enc28j60BitFieldClear(NUTSPINODE * node, uint8_t reg, uint8_t data) {
  Enc28j60SelectBank(node, getEncodedBank(reg));

  Enc28j60AllocateBus(node);
  Enc28j60TransmitCommand(node, ENC28J60_OPCODE_BFC, reg);
  Enc28j60WriteData(node, &data, 1);
  Enc28j60ReleaseBus(node);

  return 0;
}

int Enc28j60ReadBufferMemory(NUTSPINODE * node, uint8_t * rx_buf, size_t length) {
  Enc28j60AllocateBus(node);

  Enc28j60TransmitCommand(node, ENC28J60_OPCODE_RBM, 0x1A);
  Enc28j60ReadData(node, rx_buf, length);

  Enc28j60ReleaseBus(node);

  return 0;
}

int Enc28j60WriteLHControlRegister(NUTSPINODE * node, uint8_t lowreg, uint16_t data) {
  uint8_t hbyte = (uint8_t) (data >> 8);
  uint8_t lbyte = (uint8_t) data;

  Enc28j60WriteControlRegister(node, lowreg, &lbyte);
  // High-byte register is always low-reg-addr + 1
  Enc28j60WriteControlRegister(node, lowreg + 1, &hbyte);

  return 0;
}

int Enc28j60ReadLHControlRegister(NUTSPINODE * node, uint8_t lowreg, uint16_t * data) {

  // Read first byte directly into data
  Enc28j60ReadControlRegister(node, lowreg, (uint8_t *) data);

  // Read second-byte (high-byte into buffer)
  Enc28j60ReadControlRegister(node, lowreg + 1, ((uint8_t *) data) + 1);


  return 0;
}

int Enc28j60ReadPhyRegister(NUTSPINODE * node, uint8_t reg, uint16_t * rx_buf) {
  // Writing the phy-address to read into the MIREGADR-register
  Enc28j60WriteControlRegister(node, ENC28J60_REG_MIREGADR, &reg);

  // Setting the MICMD.MIIRD
  uint8_t buf[1];
  Enc28j60ReadControlRegister(node, ENC28J60_REG_MICMD, buf);
  buf[0] |= 0x01;
  Enc28j60WriteControlRegister(node, ENC28J60_REG_MICMD, buf);

  // TODO: Wait 10.24µs
  NutSleep(1);

  // Wait for the operation to be completed
  _Bool completed = 0;
  while (!completed) {
    Enc28j60ReadControlRegister(node, ENC28J60_REG_MISTAT, buf);
    if ((buf[0] & 0x01) == 0x00) completed = 1;
  }

  Enc28j60ReadControlRegister(node, ENC28J60_REG_MICMD, buf);
  buf[0] &= 0xFE;
  Enc28j60WriteControlRegister(node, ENC28J60_REG_MICMD, buf);

  Enc28j60ReadLHControlRegister(node, ENC28J60_REG_MIRDL, rx_buf);

  return 0;
}

int Enc28j60WritePhyRegister(NUTSPINODE * node, uint8_t reg, uint16_t tx_buf) {
  // Writing the phy-address to read into the MIREGADR-register
  Enc28j60WriteControlRegister(node, ENC28J60_REG_MIREGADR, &reg);

  // Write the data to save. It is automatically saved when the high-byte is written
  Enc28j60WriteLHControlRegister(node, ENC28J60_REG_MIWRL, tx_buf);

  // TODO: Wait 10.24µs
  NutSleep(1);

  // Wait for the operation to be completed
  uint8_t buf[1];
  _Bool completed = 0;
  while (!completed) {
    Enc28j60ReadControlRegister(node, ENC28J60_REG_MISTAT, buf);
    if ((buf[0] & 0x01) == 0x00) completed = 1;
  }

  return 0;
}

// Bit Field operations can't be applied onto MAC, MII & PHY-registers
// This method sets bits by reading the old value and writing the new one
// This method should not be used when dealing with interrupts as some could be
// missed while reading / writing the values (Use Enc28j60SetBit instead)
int Enc28j60SetBitRW(NUTSPINODE * node, uint8_t reg, uint8_t bitpos, _Bool value) {
  uint8_t buffer = 0;
  Enc28j60ReadControlRegister(node, reg, &buffer);

  if (value)
    buffer |= (1 << bitpos);
  else
    buffer &= ~(1 << bitpos);

  Enc28j60WriteControlRegister(node, reg, &buffer);
  return 0;
}

// -----------------------------------------------------------------------------

// TODO: Just for debugging-purposes, remove in production
int Enc28j60PrintChipInfo(NUTSPINODE * node) {
  uint8_t chip_revision = 0;
  Enc28j60ReadControlRegister(node, ENC28J60_REG_EREVID, &chip_revision);
  if (chip_revision <= 0) {
    printf("ERROR: ENC28J60: No chip detected!\n");
  } else {
    printf("ENC28J60: Chip with revision %i detected.\n", chip_revision);
  }
  return 0;
}

int Enc28j60SoftReset(NUTSPINODE * node) {
  Enc28j60AllocateBus(node);
  int rc = Enc28j60TransmitCommand(node, ENC28J60_OPCODE_SRC, ENC28J60_ARGUMENT_SRC);
  Enc28j60ReleaseBus(node);
  return rc;
}

int Enc28j60InitalizeBuffers(NUTSPINODE * node) {
  // Read-buffer starts at start of 8K RAM
  Enc28j60WriteLHControlRegister(node, ENC28J60_REG_ERXSTL, 0x00);
  // Write that value as an initial value of the nextPacketPointer
  ((ENC28J60_DCB *) node->node_dcb)->rxNextPacketPointer = 0x00;

  // According to the datasheet, the ERXRDPT-registers should also be programmed to the start position
  Enc28j60WriteLHControlRegister(node, ENC28J60_REG_ERXRDPTL, 0x00);

  // Set the receive-buffer size to be the whole buffer - tx-buffer size
  // (rx-buffer ends in byte ERXND)
  Enc28j60WriteLHControlRegister(node, ENC28J60_REG_ERXNDL, ENC28J60_RXTXBUF_SIZE - ENC28J60_TXBUF_SIZE - 1);

  return 0;
}

int Enc28j60InitalizeMACRegisters (NUTSPINODE * node) {
  // Set the MARXEN-bit in MACON1 to enable MAC receiving frames
  // Also set TXPAUS & RXPAUS for full-duplex mode
  uint8_t buffer = 0;
  Enc28j60ReadControlRegister(node, ENC28J60_REG_MACON1, &buffer);
  buffer |= (1 << ENC28J60_BIT_MARXEN) |
            (1 << ENC28J60_BIT_RXPAUS) |
            (1 << ENC28J60_BIT_TXPAUS);
  Enc28j60WriteControlRegister(node, ENC28J60_REG_MACON1, &buffer);

  // TODO: Full Duplex is hardcoded for now
  // Set PADCFG, TXCRCEN and FULDPX bits of MACON3
  // TODO: For now, PADCFG is set to 001 (all short frames 0-padded to 60bytes) (Ask Ole if thats correct)
  Enc28j60ReadControlRegister(node, ENC28J60_REG_MACON3, &buffer);
  buffer |= (1 << ENC28J60_BIT_FULDPX) |
            (1 << ENC28J60_BIT_FRMLNEN) |
            (1 << ENC28J60_BIT_TXCRCEN) |
            (1 << ENC28J60_BIT_PADCFG0);
  buffer &= ~(1 << ENC28J60_BIT_PADCFG1 | 1 << ENC28J60_BIT_PADCFG2);
  Enc28j60WriteControlRegister(node, ENC28J60_REG_MACON3, &buffer);

  // Set the defer-bit in MACON4 for compliance with IEEE 802.3
  Enc28j60SetBitRW(node, ENC28J60_REG_MACON4, ENC28J60_BIT_DEFER, 1);

  // Set the maximum frame-length to receive to 1526 bytes
  Enc28j60WriteLHControlRegister(node, ENC28J60_REG_MAMXFLL, ETHFRAME_SIZE);

  // For the MABBIPG-register in full duplex, 0x15 is recommended; 0x12 in half-duplex
  uint8_t mabbipg_value = 0x15;
  Enc28j60WriteControlRegister(node, ENC28J60_REG_MABBIPG, &mabbipg_value);

  // Program MAIPGL with the recommended value 0x12
  uint8_t maipgl_value = 0x15;
  Enc28j60WriteControlRegister(node, ENC28J60_REG_MAIPGL, &maipgl_value);

  // Program MAC-address
  IFNET * ifn_eth0 = (IFNET *) ((ENC28J60_DCB *) node->node_dcb)->ifeth;
  Enc28j60WriteControlRegister(node, ENC28J60_REG_MAADR1, &ifn_eth0->if_mac[0]);
  Enc28j60WriteControlRegister(node, ENC28J60_REG_MAADR2, &ifn_eth0->if_mac[1]);
  Enc28j60WriteControlRegister(node, ENC28J60_REG_MAADR3, &ifn_eth0->if_mac[2]);
  Enc28j60WriteControlRegister(node, ENC28J60_REG_MAADR4, &ifn_eth0->if_mac[3]);
  Enc28j60WriteControlRegister(node, ENC28J60_REG_MAADR5, &ifn_eth0->if_mac[4]);
  Enc28j60WriteControlRegister(node, ENC28J60_REG_MAADR6, &ifn_eth0->if_mac[5]);

  return 0;
}

// -----------------------------------------------------------------------------

static inline int Enc28j60PutPacket(NUTDEVICE *dev, NETBUF * nb)
{
    int       rc = -1;
    uint16_t  sz;
    uint8_t  *buf;
    uint32_t  idx;
    NUTSPINODE * node = (NUTSPINODE *) ((ENC28J60_DCB *)dev->dev_dcb)->node;
    ENC28J60_DCB * dcb = (ENC28J60_DCB *) node->node_dcb;
    IFNET * ifn = (IFNET *) dcb->ifeth;

    /*
     * Calculate the number of bytes to be send. Do not send packets
     * larger than the Ethernet maximum transfer unit. The MTU
     * consist of 1500 data bytes plus the 14 byte Ethernet header
     * plus 4 bytes CRC. We check the data bytes only.
     */
    if ((sz = nb->nb_nw.sz + nb->nb_tp.sz + nb->nb_ap.sz) > ifn->if_mtu) {
        return -1;
    }

    sz += nb->nb_dl.sz;

    /*
     * Calculate the memory-position, where to put the packet.
     * It is calculated by getting the start of the TX-buffer and adding the in-use space
     * It is recommended to start at an even address, so add 1 if it's not even.
     */
    uint16_t tx_current_start = ENC28J60_RXTXBUF_SIZE - ENC28J60_TXBUF_SIZE;
    if (tx_current_start % 2 == 1)
      tx_current_start++;

    /* Disable interrupts. */
    Enc28j60SetBitRW(node, ENC28J60_RAWREG_EIE, ENC28J60_BIT_INTIE, 0);
    GpioIrqDisable(dcb->gpioIrqSig, 5);

    /*
     * Program the ETXST-Register to point to tx_current_start
     */
    Enc28j60WriteLHControlRegister(node, ENC28J60_REG_ETXSTL, tx_current_start);

    /*
     * Program the EWRPT-Register to point to tx_current start
     * as the position, where the WBM-command should start writing
     */
    Enc28j60WriteLHControlRegister(node, ENC28J60_REG_EWRPTL, tx_current_start);

    #pragma message "Redefine per packet control byte"
    // In this configuration, the values from MACON3 will be used
    uint8_t perPacketControlByte = 0x00;

    Enc28j60AllocateBus(node);

    Enc28j60TransmitCommand(node, ENC28J60_OPCODE_WBM, ENC28J60_ARGUMENT_WBM);

    Enc28j60WriteData(node, &perPacketControlByte, 1);

    Enc28j60WriteData(node, nb->nb_dl.vp, nb->nb_dl.sz);
    Enc28j60WriteData(node, nb->nb_nw.vp, nb->nb_nw.sz);
    Enc28j60WriteData(node, nb->nb_tp.vp, nb->nb_tp.sz);
    Enc28j60WriteData(node, nb->nb_ap.vp, nb->nb_ap.sz);

    NutSleep(10);

    Enc28j60ReleaseBus(node);

    // Calculate end of packet and set the corresponding Registers
    uint16_t packet_end_reg = tx_current_start + sz;
    Enc28j60WriteLHControlRegister(node, ENC28J60_REG_ETXNDL, packet_end_reg);

    /* Re-enable interrupts */
    GpioIrqEnable(dcb->gpioIrqSig, 5);
    // Somehow the following operations can't be done with BFS / BFC-operations
    Enc28j60SetBitRW(node, ENC28J60_RAWREG_EIR, ENC28J60_BIT_TXIF, 0);
    Enc28j60SetBitRW(node, ENC28J60_RAWREG_EIE, ENC28J60_BIT_TXIE, 1);
    Enc28j60SetBitRW(node, ENC28J60_RAWREG_EIE, ENC28J60_BIT_INTIE, 1);

    // Send the packet
    Enc28j60SetBitRW(node, ENC28J60_RAWREG_ECON1, ENC28J60_BIT_TXRTS, 1);

#ifdef NUT_PERFMON
      ni->ni_tx_packets++;
#endif

    rc = 0;
    return rc;
}

/*!
 * \brief Send Ethernet packet.
 *
 * \param dev Identifies the device to use.
 * \param nb  Network buffer structure containing the packet to be sent.
 *            The structure must have been allocated by a previous
 *            call NutNetBufAlloc().
 *
 * \return 0 on success, -1 in case of any errors.
 */
int Enc28j60Output(NUTDEVICE * dev, NETBUF * nb)
{
    static uint32_t mx_wait = 5000;
    static uint32_t tx_wait = 500;
    uint32_t consume_idx;
    uint32_t produce_idx;
    uint32_t num_fragments;
    int rc = -1;
    NUTSPINODE * node = (NUTSPINODE *) ((ENC28J60_DCB *)dev->dev_dcb)->node;
    ENC28J60_DCB * dcb = (ENC28J60_DCB *) node->node_dcb;
    size_t space = 0;
    size_t pkg_size;

    /*
     * After initialization we are waiting for a long time to give
     * the PHY a chance to establish an Ethernet link.
     */
    while (rc) {
        if (NutEventWait(&dcb->ni_mutex, mx_wait)) {
            break;
        }

        if (NutEventWait(&dcb->ni_tx_rdy, tx_wait)) {
          // Timeout while waiting for tx ready
          NutEventPost(&dcb->ni_mutex);
        }

        if (Enc28j60PutPacket(dev, nb) == 0) {
          /* Ethernet works. Set a long waiting time in case we
             temporarly lose the link next time. */
          rc = 0;
          NutEventPost(&dcb->ni_tx_rdy);
        }

        NutEventPost(&dcb->ni_mutex);
    }

    /*
     * Probably no Ethernet link. Significantly reduce the waiting
     * time, so following transmission will soon return an error.
     */
    if (rc) {
        mx_wait = 500;
    } else {
        /* Ethernet works. Set a long waiting time in case we
           temporarily lose the link next time. */
        mx_wait = 5000;
    }
    return rc;
}

int Enc28j60ReadPacket(NUTSPINODE * node, NETBUF ** nbp) {
  ENC28J60_DCB * dcb = (ENC28J60_DCB *) node->node_dcb;
  * nbp = NULL;
  int rc = 2;

  // Program the ERDPT-Control register to the start of the current packet
  uint16_t currentPacketStart = dcb->rxNextPacketPointer;
  Enc28j60WriteLHControlRegister(node, ENC28J60_REG_ERDPTL, currentPacketStart);

  // Read in the new value for the next packet pointer
  uint8_t rxNextPacketPointerBuf[2];
  Enc28j60ReadBufferMemory(node, rxNextPacketPointerBuf, 2);
  dcb->rxNextPacketPointer = (rxNextPacketPointerBuf[1] << 8) | rxNextPacketPointerBuf[0];

  // Read in the receive status vectors
  uint8_t statusVectorsBuf[4];
  Enc28j60ReadBufferMemory(node, statusVectorsBuf, 4);
  uint32_t statusVectors = (statusVectorsBuf[3] << 24) | (statusVectorsBuf[2] << 16) |
    (statusVectorsBuf[1] << 8) | statusVectorsBuf[0];

  if (((statusVectors >> ENC28J60_STATVEC_RECV_OK) & 1) != 1) {
    printf("Packet was not ok, freeing the buffer memory\n");
    printf("statusVectors: %02x %02x %02x %02x\n", statusVectorsBuf[3] , statusVectorsBuf[2] , statusVectorsBuf[1] , statusVectorsBuf[0]);
    // Free the buffer memory
    Enc28j60WriteLHControlRegister(node, ENC28J60_REG_ERXRDPTL, dcb->rxNextPacketPointer);
    return 1;
  }

  // Read the packet into the buffer (subtract 4 bytes CRC)
  uint16_t packetLength = (uint16_t) (statusVectors & 0xFFFF) /*- 4*/; // Packet length == first 2 bytes
  *nbp = NutNetBufAlloc(0, NBAF_DATALINK + (2 & (NUTMEM_ALIGNMENT - 1)), packetLength);

  if (*nbp != NULL) {
    uint8_t * bp = (uint8_t *) (* nbp)->nb_dl.vp;
    Enc28j60ReadBufferMemory(node, bp, packetLength);

    // Free the buffer memory
    Enc28j60WriteLHControlRegister(node, ENC28J60_REG_ERXRDPTL, dcb->rxNextPacketPointer);

    // Reduce the "waiting" packet count by 1 so that the flags will eventually be cleared
    Enc28j60BitFieldSet(node, ENC28J60_RAWREG_ECON2, (1 << ENC28J60_BIT_PKTDEC));

    rc = 0;
  }

  return rc;
}

int handleInterrupt(NUTDEVICE * dev, NETBUF * receiveBuffer) {
    NUTSPINODE * node = (NUTSPINODE *) ((ENC28J60_DCB *)dev->dev_dcb)->node;
    IFNET *ifn = (IFNET *) dev->dev_icb;
    ENC28J60_DCB * dcb = (ENC28J60_DCB *) node->node_dcb;

    /* Disable the enc-interrupts (Ethernut-interrupts have already been disabled in the interrupt-method) */
    Enc28j60BitFieldClear(node, ENC28J60_RAWREG_EIE, (1 << ENC28J60_BIT_INTIE));

    /*
     * Read all interrupt-flags into a buffer and directly reset them
     * WARNING: If an flag is set during this this time, it WILL be OVERWRITTEN!!!
     * Currently, there seems to be no better way to handle this problem.
     * If a flag is set while the current flags are being handled, it shouldn't
     * be a problem, because when the interrupt pin gets enabled, it will directly
     * fire again.
     */
    uint8_t interruptFlagsBuf;
    Enc28j60ReadControlRegister(node, ENC28J60_RAWREG_EIR, &interruptFlagsBuf);

    /*
     * Clear the interrupt flags
     * The application shouldn't clear PKTIF, as it will get automagically
     * cleared when all packets are processed
     */
    Enc28j60BitFieldClear(
      node, ENC28J60_RAWREG_EIR,
      (1 << ENC28J60_BIT_RXERIF) | (1 << ENC28J60_BIT_TXERIF) | (1 << ENC28J60_BIT_TXIF) |
      (1 << ENC28J60_BIT_LINKIF) | (1 << ENC28J60_BIT_DEMAIF)
    );

    /*
     * Check all interrupt-flags and handle them correctly
     */
    if (((interruptFlagsBuf >> ENC28J60_BIT_RXERIF) & 1) == 1) {
      // Receive error
      printf("Receive-error interrupt\n");
    }

    if (((interruptFlagsBuf >> ENC28J60_BIT_TXERIF) & 1) == 1) {
      // Transmit error
      printf("Transmit-error interrupt\n");
    }

    if (((interruptFlagsBuf >> ENC28J60_BIT_TXIF) & 1) == 1) {
      // Packet transmitted
      //printf("TXIF set, packet transmitted successfully!\n");
      NutEventPost(&dcb->ni_tx_rdy);
    }

    if (((interruptFlagsBuf >> ENC28J60_BIT_LINKIF) & 1) == 1) {
      // Link change
      printf("Link change interrupt\n");
    }

    if (((interruptFlagsBuf >> ENC28J60_BIT_DEMAIF) & 1) == 1) {
      // DMA copy or checksum-calculation has completed
      printf("DMA copy or checksum-calculation completed interrupt\n");
    }

    if (((interruptFlagsBuf >> ENC28J60_BIT_PKTIF) & 1) == 1) {
      // Receive packet pending
      for (;;) {
        if (Enc28j60ReadPacket(node, &receiveBuffer) == 0) {
          /* Discard short packets. */
          if (receiveBuffer->nb_dl.sz < 60) {
            NutNetBufFree(receiveBuffer);
          } else {
            (*ifn->if_recv) (dev, receiveBuffer);
          }
        }

        Enc28j60ReadControlRegister(node, ENC28J60_RAWREG_EIR, &interruptFlagsBuf);
        if (((interruptFlagsBuf >> ENC28J60_BIT_PKTIF) & 1) == 0) {
          break;
        }
      }
    }

    /* Enable the interrupts again */
    GpioIrqEnable(dcb->gpioIrqSig, 5);
    Enc28j60BitFieldSet(node, ENC28J60_RAWREG_EIE, (1 << ENC28J60_BIT_INTIE));

    return 0;
}

/*! \fn EmacRxThread(void *arg)
 * \brief NIC receiver thread.
 *
 */
THREAD(Enc28j60RxThread, arg)
{
    NUTDEVICE *dev = arg;
    NUTSPINODE * node = (NUTSPINODE *) ((ENC28J60_DCB *)dev->dev_dcb)->node;
    ENC28J60_DCB * dcb = (ENC28J60_DCB *) node->node_dcb;
    NETBUF * receiveBuffer = NULL;

    // Initialize the MAC-registers and programm the MAC-address
    Enc28j60InitalizeMACRegisters(node);

    /* Initialize the access mutex. */
    NutEventPost(&dcb->ni_mutex);

    /* Run at high priority. */
    NutThreadSetPriority(9);

    /* Enable enc28j60-interrupts. */
    GpioIrqEnable(dcb->gpioIrqSig, 5);
    GpioIrqSetMode(dcb->gpioIrqSig, 5, NUT_IRQMODE_FALLINGEDGE);

    /* Set interrupt-register to default values */
    // TODO: EIR is a rawreg, maybe build a BFC-Method that doesn't change the bank
    Enc28j60BitFieldClear(node, ENC28J60_RAWREG_EIR, 0xFF);

    /* Enable the gobal interrupt-enable bit and these interrupts:
     *   - RXERIE     Receive Error
     *   - TXERIE     Transmit Error
     *   - TXIE       Transmit Interrupt
     *   - LINKIE     Link status change interrupt
     *   - PKTIE      Receive Packet Pending
     */
    Enc28j60BitFieldSet(
      node, ENC28J60_RAWREG_EIE,
      (1 << ENC28J60_BIT_INTIE) | (1 << ENC28J60_BIT_RXERIE) | (1 << ENC28J60_BIT_TXERIE) |
      (1 << ENC28J60_BIT_TXIE) | (1 << ENC28J60_BIT_LINKIE) | (1 << ENC28J60_BIT_PKTIE)
    );

    /*
     * Set the rx packet-filters to sensible values:
     *  ANDOR == 0      => Filters in OR-Mode (one filter has to match)
     *  BCEN == 1       => Let Broadcast-Packets through
     *  CRCEN == 1      => Packets with an invalid CRC (probably corrupt) will be discarded
     *  UCEN == 1       => As long as the packet dest. MAC-Adress matches, the packet goes through
     */
    static uint8_t erxfconVal = (1 << ENC28J60_BIT_BCEN) | (1 << ENC28J60_BIT_CRCEN) | (1 << ENC28J60_BIT_UCEN);
    Enc28j60WriteControlRegister(node, ENC28J60_REG_ERXFCON, &erxfconVal);

    /* Enable packet reception */
    Enc28j60BitFieldSet(node, ENC28J60_RAWREG_ECON1, (1 << ENC28J60_BIT_RXEN));

    /* Initialize the tx-mutex */
    NutEventPost(&dcb->ni_tx_rdy);
    NutEventPost(&dcb->ni_rx_rdy);

    for (;;) {

      NutEventWait(&dcb->interruptReady, 50);

      // An interrupt happened, handle it!
      handleInterrupt(dev, receiveBuffer);

    }
}

static void Enc28j60Interrupt(void* args) {
  NUTDEVICE * dev = (NUTDEVICE *) args;
  ENC28J60_DCB * dcb = (ENC28J60_DCB *) dev->dev_dcb;

  // Disable interrupts temporarily to prevent the same interrupt from being fired twice
  GpioIrqDisable(dcb->gpioIrqSig, 5);

  NutEventPostFromIrq(&dcb->interruptReady);
}

int Enc28j60Init(NUTDEVICE * dev) {
  NUTSPINODE * node = (NUTSPINODE *) ((ENC28J60_DCB *)dev->dev_dcb)->node;
  ENC28J60_DCB * dcb = (ENC28J60_DCB *) node->node_dcb;

  Enc28j60SoftReset(node);
  // Give the ENC a little time to boot up
  NutSleep(100);

  // TODO: Remove / Print only if DEBUG
  Enc28j60PrintChipInfo(node);

  // Initialize the rx-buffer (and indirectly the tx too)
  Enc28j60InitalizeBuffers(node);

  // TODO: Maybe implement datasheet 6.4: Waiting For OST? Ask Ole

  /* Register enc28j60 interrupt handler. */
  // TODO: Interrupt-Pin currently hardcoded
  // WIFIMCU: D13 -> PA5
  GpioPinConfigSet(NUTGPIO_PORTA, 5, GPIO_CFG_PULLUP);
  dcb->gpioIrqSig = GpioCreateIrqHandler(NUTGPIO_PORTA, 5, Enc28j60Interrupt, dev);
  if (!dcb->gpioIrqSig) {
    EMPRINTF("ENC28J60: Registering IRQ failed\n");
    return -1;
  }

  /* Start the receiver thread. */
  if (NutThreadCreate("enc28j60rx", Enc28j60RxThread, dev,
      (NUT_THREAD_NICRXSTACK * NUT_THREAD_STACK_MULT) + NUT_THREAD_STACK_ADD) == NULL) {
      EMPRINTF("ENC28J60: Registering RX Thread failed\n");
      return -1;
  }

  return 0;
}

static int Enc28j60IOCtl(NUTDEVICE * dev, int req, void *conf)
{
  int rc = 0;
  IFNET *ifn = (IFNET *) dev->dev_icb;
  uint32_t ip;

  switch (req) {
  case SIOCSIFADDR:
      /* Set interface hardware address. */
      memcpy(ifn->if_mac, conf, sizeof(ifn->if_mac));
      break;
  case SIOCGIFADDR:
      /* Get interface hardware address. */
      memcpy(conf, ifn->if_mac, sizeof(ifn->if_mac));
      break;
  case SIOCADDMULTI:
      /* Add multicast address. */
      rc = -1;
      break;
  case SIOCDELMULTI:
      /* Delete multicast address. */
      rc = -1;
      break;
  default:
      rc = -1;
      break;
  }
  return rc;
}

// -----------------------------------------------------------------------------

/*!
 * \brief Register and initialize the ENC28J60-Spi device attached to a specified bus.
 *
 * Calls the bus controller initialization and, if successful, initializes
 * the SPI device and adds it to the system device list.
 *
 * The application should use this function to register the SPI-device,
 * as it correctly deals with any structural difficulties caused by an spi-attached
 * ethernet-chip.
 *
 * \param bus Pointer to the \ref NUTSPIBUS structure, which is provided
 *            by the bus controller driver.
 * \param cs  Zero based chip select number for this device. Must be set
 *            to 0, if only one device is attached to the bus and no chip
 *            select line is provided.
 *
 * \return 0 if the device has been registered for the first time and
 *         the initialization was successful. The function returns -1,
 *         if any device with the same name had been registered previously,
 *         if the \ref NUTDEVICE structure or the chip select is invalid or
 *         if the device or bus controller initialization failed.
 */
int NutRegisterEnc28j60Spi(NUTDEVICE * dev, NUTSPIBUS * bus, int cs)
{
    int rc;
    NUTSPINODE *node;

    NUTASSERT(dev != NULL);
    node = (NUTSPINODE *) ((ENC28J60_DCB *) dev->dev_dcb)->node;

    NUTASSERT(bus != NULL);
    NUTASSERT(bus->bus_initnode != NULL);

    /* Attach the bus controller driver and set the chip select number
       ** before calling the bus controller initialization routine. */
    node->node_bus = bus;
    node->node_cs = cs;
    rc = (*bus->bus_initnode) (node);

    /* If the bus had been successfully initialized for this device,
       ** then set up a mutex lock for the bus and register the device. */
    if (rc == 0) {
        NutEventPost(&bus->bus_mutex);
        rc = NutRegisterDevice(dev, 0, 0);
    }
    return rc;
}


// -----------------------------------------------------------------------------

/*!
 * \brief Network interface information structure.
 *
 * Used to call.
 */
static IFNET ifn_eth0 = {
    IFT_ETHER,          /*!< \brief Interface type, if_type. */
    0,                  /*!< \brief Interface flags, if_flags. */
    {0, 0, 0, 0, 0, 0}, /*!< \brief Hardware net address, if_mac. */
    0,                  /*!< \brief IP address, if_local_ip. */
    0,                  /*!< \brief Remote IP address for point to point, if_remote_ip. */
    0,                  /*!< \brief IP network mask, if_mask. */
    ETHERMTU,           /*!< \brief Maximum size of a transmission unit, if_mtu. */
    0,                  /*!< \brief Packet identifier, if_pkt_id. */
    0,                  /*!< \brief Linked list of arp entries, arpTable. */
    0,                  /*!< \brief Linked list of multicast address entries, if_mcast. */
    NutEtherInput,      /*!< \brief Routine to pass received data to, if_recv(). */
    Enc28j60Output,     /*!< \brief Driver output routine, if_send(). */
    NutEtherOutput,     /*!< \brief Media output routine, if_output(). */
    NULL                /*!< \brief Interface specific control function, if_ioctl(). */
#ifdef NUT_PERFMON
    , 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0
#endif
};

static ENC28J60_DCB dcbENC28J60 = {
  &ifn_eth0,    // => ifeth
  &nodeENC28J60,
  -1,           // Bank (default -1 to set at start) => bank
  0,            // => tx_space_used
    //
};

/*!
 * \brief ENC28J60 SPI node implementation structure.
 */
NUTSPINODE nodeENC28J60 = {
    NULL,                       /*!< \brief Pointer to the bus controller driver, node_bus. */
    NULL,                       /*!< \brief Pointer to the bus device driver specific settings, node_stat. */
    SPI_ENC28J60_CLK,           /*!< \brief Initial clock rate, node_rate. */
    SPI_MODE_0,                 /*!< \brief Initial mode, node_mode. */
    8,                          /*!< \brief Initial data bits, node_bits. */
    0,                          /*!< \brief Chip select, node_cs. */
    &dcbENC28J60                /*!< \brief Pointer to our private device control block, node_dcb. */
};

/*!
 * \brief Device information structure.
 *
 * A pointer to this structure must be passed to NutRegisterDevice()
 * to bind this Ethernet device driver to the Nut/OS kernel.
 * An application may then call NutNetIfConfig() with the name \em eth0
 * of this driver to initialize the network interface.
 *
 */
NUTDEVICE devENC28J60 = {
    0,          /* Pointer to next device. */
    {'e', 't', 'h', '0', 0, 0, 0, 0, 0},        /* Unique device name. */
    IFTYP_NET,  /* Type of device. */
    0,          /* Base address. */
    0,          /* First interrupt number. */
    &ifn_eth0,  /* Interface control block. */
    &dcbENC28J60,  /* Driver control block. */
    Enc28j60Init,  /* Driver initialization routine. */
    Enc28j60IOCtl, /* Driver specific control function. */
    0,          /* Read from device. */
    0,          /* Write to device. */
#ifdef __HARVARD_ARCH__
    0,          /* Write from program space data to device. */
#endif
    0,          /* Open a device or file. */
    0,          /* Close a device or file. */
    0,          /* Request file size. */
    0,          /* Select function, optional, not yet implemented */
};

/*@}*/
