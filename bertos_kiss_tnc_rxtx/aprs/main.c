/**
 * \file
 * <!--
 * This file is part of BeRTOS.
 *
 * Bertos is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 *
 * As a special exception, you may use this file as part of a free software
 * library without restriction.  Specifically, if other files instantiate
 * templates or use macros or inline functions from this file, or you compile
 * this file and link it with other files to produce an executable, this
 * file does not by itself cause the resulting executable to be covered by
 * the GNU General Public License.  This exception does not however
 * invalidate any other reasons why the executable file might be covered by
 * the GNU General Public License.
 *
 * Copyright 2010 Develer S.r.l. (http://www.develer.com/)
 *
 * -->
 *
 * \author Francesco Sacchi <batt@develer.com>
 * \author Luca Ottaviano <lottaviano@develer.com>
 * \author Daniele Basile <asterix@develer.com>
 *
 * \brief Arduino APRS radio demo.
 *
 * This example shows how to read and decode APRS radio packets.
 * It uses the following modules:
 * afsk
 * ax25
 * ser
 *
 * You will see how to use a serial port to output messages, init the afsk demodulator and
 * how to parse input messages using ax25 module.
 */


// ###### INCLUDES #######

#include <cpu/irq.h>
#include <cfg/debug.h>

#include <net/afsk.h>
#include <net/ax25.h>

#include <drv/ser.h>
#include <drv/timer.h>

#include <stdio.h>
#include <string.h>

// Rob:
#define F_CPU 16000000
#include <util/delay.h>
//#include <struct/fifobuf.h>
#include "buildrev.h"
#include <algo/crc_ccitt.h>


// ###### GLOBALS #######

static Afsk        afsk;
static AX25Ctx     ax25;
static Serial      ser;
AX25KISSMsg kiss ;

#define ADC_CH 0


// ###### PROTOS #######

void init(void);
void kiss_tx_packet( AX25Ctx *ctx, AX25KISSMsg *k );
void kiss_init( AX25KISSMsg *k );


// ###################
// ###### SUBS #######
// ###################


void kiss_init( AX25KISSMsg *k )
{
        // Initialize KISS TX object

        uint16_t   a = CONFIG_AX25_FRAME_BUF_LEN ;
        uint8_t   *p = &k->buf[0] ;

        while ( a-- ) { *p++ = 0x00 ; }
        k->pos = 0 ;
        return ;
}

        
static void kiss_serial_poll( AX25Ctx *ctx, AX25KISSMsg *k )
{
        // Read incoming KISS data from serial port
        // Add data to KISS object's buffer
        // TX if appropriate

        int       c ;
        uint8_t   b ;

        if ( ( c = ser_getchar_nowait( &ser ) ) == EOF ) { return ; }

        // sanity checks
        // no serial input in last 2 secs?
        if (timer_clock() - k->last_tick  >  ms_to_ticks(2000L)) 
        { k->pos = 0 ; }

        // about to overflow buffer? reset
        if ( k->pos  >=  (uint16_t)( CONFIG_AX25_FRAME_BUF_LEN - 2 ) ) 
        { k->pos = 0 ; }

        // trim value to 0-255, add to buffer
        b = c & 0x00FF ;
        k->buf[ k->pos++ ] = b ;
        k->last_tick = timer_clock() ;

        // frame end? 
        if ( b == 0xC0 ) 
        {
            // Long enough?
            if ( k->pos  >=  AX25_MIN_FRAME_LEN )
            { kiss_tx_packet( ctx , k ) ; }   // transmit

            // either way, reset, and add this last 0xC0 back in, 
            // in case it is both a frame end and a frame start
            // (or just a frame start)
            k->buf[0] = 0xC0 ;
            k->pos = 1 ;
        }
        return ;
}


void kiss_tx_packet( AX25Ctx *ctx, AX25KISSMsg *k ) 
{
        // Transmit KISS packet 

        uint8_t  *buf = &k->buf[0] ;
        uint8_t   pos = k->pos ;

        ctx->crc_out = CRC_CCITT_INIT_VAL;
	kfile_putc(HDLC_FLAG, ctx->ch);

        while ( pos-- )
        {
            switch ( *buf )
            {
                case 0xC0:           // fall thru
                case 0x00:
                case 0x0A:
                case 0x0D:  break ;

                default:    ax25_putchar( ctx, *buf ) ;
                            break ;
            }
            buf++ ;
        }

        /*
	 * According to AX25 protocol,
	 * CRC is sent in reverse order!
	 */
	uint8_t crcl = (ctx->crc_out & 0xff) ^ 0xff;
	uint8_t crch = (ctx->crc_out >> 8) ^ 0xff;
	ax25_putchar(ctx, crcl);
	ax25_putchar(ctx, crch);

	ASSERT(ctx->crc_out == AX25_CRC_CORRECT);

	kfile_putc(HDLC_FLAG, ctx->ch);       
        return ; 
}


static void kiss_rx_callback( size_t frame_len ) 
{
        // function used by AX25 module to process rec'd packets
        // here, we're just pumping out the KISS data to the serial object

	// TODO: escapes? special chars?

	uint8_t  *buf = &ax25.buf[0] ;

        kfile_putc( 0xC0 , &ser.fd ) ;
        kfile_putc( 0x00 , &ser.fd ) ; 

        while( frame_len-- )
        {
            kfile_putc( *buf++ , &ser.fd ) ;
        }

        kfile_putc( 0xC0 , &ser.fd ) ;
}

void init(void)
{
        
	IRQ_ENABLE;
	kdbg_init();
	timer_init();

	/* Initialize serial port, we are going to use it to show APRS messages*/
	ser_init(&ser, SER_UART0);
	ser_setbaudrate(&ser, 19200L);

        // initialize kiss transmit object
        kiss_init( &kiss ) ;
        DDRB |= 0x03 ;

        // settle
        _delay_ms(2000) ;

	// Init afsk demodulator
	afsk_init(&afsk, ADC_CH, 0);
	
        // Init AX25 context
	ax25_init(&ax25, &afsk.fd, kiss_rx_callback);

        // Init AX25 KISS TX object
        kiss_init(&kiss) ;
        
        // Announce
        kfile_printf( &ser.fd, "\r\n== BeRTOS AVR/Arduino KISS TNC\r\n" ) ;
        kfile_printf( &ser.fd, "== RX/TX Version 0.02, Build %d\r\n", VERS_BUILD ) ;
        kfile_printf( &ser.fd, "== Starting.\r\n" ) ;
}


int main(void)
{
	init();

	while (1)
	{
		// Use AX25 module call to check AFSK object for incoming data
                ax25_poll( &ax25 , AX25_OUTPUT_KISS ); 
                
                // Use local code to check incoming serial object for new data
		kiss_serial_poll( &ax25 , &kiss ) ;

	}
	return 0;
}

// end of file
