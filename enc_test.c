/*!
 * Copyright (C) 2001-2004 by egnite Software GmbH. All rights reserved.
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
 * $Id: threads.c 4640 2012-09-24 12:05:56Z u_bonnes $
 */

/*!
 * \example threads/threads.c
 *
 * This sample demonstrates Nut/OS multithreading.
 *
 * Each thread is started with 192 bytes of stack. This is very
 * low and doesn't provide much space for local variables.
 */

#include <stdio.h>
#include <io.h>

#include <cfg/arch.h>
#include <dev/board.h>
#include <dev/gpio.h>
#include <dev/spibus.h>

#include <arch/cm3/stm/stm32xxxx.h>

#include <sys/nutdebug.h>
#include <sys/thread.h>
#include <sys/timer.h>

#include <sys/version.h>
#include <sys/confnet.h>
#include <sys/heap.h>
#include <sys/timer.h>
#include <sys/socket.h>

#include <arpa/inet.h>
#include <net/if_var.h>
#include <pro/dhcp.h>

#include "enc28j60.h"

/*
 * Main application thread.
 */
int main(void)
{
    uint32_t baud = 115200;

    /*
     * Register the UART device, open it, assign stdout to it and set
     * the baudrate.
     */
    NutRegisterDevice(&DEV_CONSOLE, 0, 0);
    freopen(DEV_CONSOLE.dev_name, "w", stdout);
    freopen(DEV_CONSOLE.dev_name, "w", stderr);
    _ioctl(_fileno(stdout), UART_SETSPEED, &baud);

    puts("\nSPI Node initialisierung\n");

    if (NutRegisterEnc28j60Spi(&devENC28J60, &spiBus4Stm32, 0)) {
      NUTPANIC("Registering devENC28J60 on spiBus4Stm32 failed\n");
    }

    puts("Ethernet Test\n");

    printf("Configure %s\n", devENC28J60.dev_name);

    if (NutDhcpIfConfig(devENC28J60.dev_name, 0, 60000)) {
        printf("First DHCP failed, code %d\n", NutDhcpError(""));
        /* No valid EEPROM contents, use hard coded MAC. */
        uint8_t my_mac[] = { 0x00, 0x06, 0x98, 0x20, 0x00, 0x00 };

        if (NutDhcpIfConfig(devENC28J60.dev_name, my_mac, 60000)) {
          printf("Second DHCP failed, code %d\n", NutDhcpError(""));
            /* No DHCP server found, use hard coded IP address. */
            uint32_t ip_addr = inet_addr("192.168.1.100");
            uint32_t ip_mask = inet_addr("255.255.255.0");

            NutNetIfConfig(devENC28J60.dev_name, my_mac, ip_addr, ip_mask);
            printf("After NetIfConfig\n");
            /* If not in a local network, we must also call
               NutIpRouteAdd() to configure the routing. */
        }
    }

    printf("%s ready\n", inet_ntoa(confnet.cdn_ip_addr));



    for (;;) {
      NutSleep(1000);
    }
    return 0;
}
