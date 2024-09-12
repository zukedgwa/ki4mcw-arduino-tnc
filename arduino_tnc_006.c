// ==== includes
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <inttypes.h>

// ==== defs
#define MIN_PACKET_LEN   10
#define PACKET_SIZE      200
#define AX25_MARK        0
#define AX25_SPACE       1
#define MAX_SYNC_ERRS    5
#define MIN_DCD			 20

// ==== protos
unsigned char getadcval (void) ;
unsigned char mock_interrupt(void) ;

// ==== constants
// lookup table for 2200 Hz sine wave
// sampled at 13200 Hz, 2200 Hz divides evenly into 6 samples
const signed char sinhi_table[6] = {  0,  6,  6,  0, -6, -6 } ;
const signed char coshi_table[6] = {  7,  4, -4, -7, -4,  4 } ;

// lookup tables for 1200 Hz sime wave
// sampled at 13200 Hz, 1200 Hz divides evenly into 11 samples
const signed char sinlo_table[11] = {  0,  4,  6,  7,  5,  2, -2, -5, -7, -6, -4 } ;
const signed char coslo_table[11] = {  7,  6,  3, -1, -5, -7, -7, -5, -1,  3,  6 } ;


// ==== vars
FILE *adcdatafile ;				// text file with ADC data (13200 Hz sampling)
char filetxt[8] ;				// buffer from file

signed char     adcval,
                last_phase_err;
				
int16_t         sinhi_x_adc_cb[11], 
                coshi_x_adc_cb[11],
                sinlo_x_adc_cb[11], 
                coslo_x_adc_cb[11];
                
int16_t         sinhi_x_adc_sum, 
                coshi_x_adc_sum,
                sinlo_x_adc_sum, 
                coslo_x_adc_sum; 
                
unsigned char   adcfetch,
				since_last_chg, 
                phase_err, 
                current_symbol, 
                last_symbol, 
				last_symbol_inaframe,
                inaframe, 
                bittimes, 
                bitqlen, 
                popbits, 
                byteval,
                cb_pos, 
                sinhi_pos, 
                msg[PACKET_SIZE + 1],
                msg_pos,
                msg_ready,
                x,
                test,
                //decode_state,
                hb12 ;

unsigned char   sync_err_cnt,
				dcd,
				bit_timer,
				thesebits ;
				
signed char     fourier_sum,
                fourier_cb[11] ;
				
uint32_t        hi_squares,
                lo_squares,
				bitq,
				sampnum ;
				
unsigned char   debug = 0 ;


//############				
//   Main
//############


int main ()
{
	adcdatafile = fopen("c:\\robapps\\devel\\perldev\\avr_tnc\\ADC_13200_opensq.wav.txt" , "r" ) ;  
	if ( adcdatafile == NULL ) 
	{
		printf("File open error!\n") ;
		return 1 ;
	}
	else
	{
	    while ( mock_interrupt() == 0 ) { ; }
	}	
	fclose( adcdatafile ) ;
    return 0 ;
}


//################
//   Functions
//################

unsigned char getadcval (void)
{
    char filetxt[10] ;
	
 	fgets( filetxt, 8, adcdatafile ) ;
    if ( ferror(adcdatafile) )
    {
      printf("Read error! Numbah %d\n" , errno ) ;
	  return 0 ;
	}
	else if ( feof(adcdatafile) )
	{
      printf("End of file.\n" ) ;
      return 0 ;
    }
	
    return atoi(filetxt) ;
}


unsigned char mock_interrupt (void)
{
    sampnum++ ;

    //=====================
    //  symbol recovery 
    //=====================

    // analyze incoming audio, and determine which AX.25 frequency (symbol)
    // it represents.

    //adcval = ADCH - 128 ;
    adcfetch = getadcval() ;
    if ( adcfetch == 0 ) { return 1 ; }
    else                 { adcval = adcfetch - 128 ; }
  
    // our main circle buffers and sinlo loop all use 11 samples / bit, 
    // so use the same counter to drive both
    if (++cb_pos == 11) { cb_pos = 0 ; }
  
    // the sinhi lookup table is much larger, so needs its own counter
    if (++sinhi_pos == 6) { sinhi_pos = 0 ; } 

	//===============
    // Fourier math
	//===============

    // this looks ugly, but saves a lot of cycles
    // first, back out the previous (adc*lookup) values for this position
    // from the running totals
    sinhi_x_adc_sum -= sinhi_x_adc_cb[ cb_pos ] ;
    coshi_x_adc_sum -= coshi_x_adc_cb[ cb_pos ] ;
    sinlo_x_adc_sum -= sinlo_x_adc_cb[ cb_pos ] ;
    coslo_x_adc_sum -= coslo_x_adc_cb[ cb_pos ] ;
  
    // next, figure out the new values for this position in the circle
    sinhi_x_adc_cb[ cb_pos ] = adcval * sinhi_table[ sinhi_pos ] ;
    coshi_x_adc_cb[ cb_pos ] = adcval * coshi_table[ sinhi_pos ] ;
    sinlo_x_adc_cb[ cb_pos ] = adcval * sinlo_table[ cb_pos ] ;
    coslo_x_adc_cb[ cb_pos ] = adcval * coslo_table[ cb_pos ] ;

    // now add the new values back into the sums
    sinhi_x_adc_sum += sinhi_x_adc_cb[ cb_pos ] ;
    coshi_x_adc_sum += coshi_x_adc_cb[ cb_pos ] ;
    sinlo_x_adc_sum += sinlo_x_adc_cb[ cb_pos ] ;
    coslo_x_adc_sum += coslo_x_adc_cb[ cb_pos ] ;

    // next, square each sum to magnify the differences
    hi_squares = (sinhi_x_adc_sum * sinhi_x_adc_sum) +
                 (coshi_x_adc_sum * coshi_x_adc_sum) ;

    lo_squares = (sinlo_x_adc_sum * sinlo_x_adc_sum) +
                 (coslo_x_adc_sum * coslo_x_adc_sum) ;

    // put the final verdict for this sample into another
    // running total via circle-buffer: +1 for hi, -1 for lo, 0 for unsure
    fourier_sum -= fourier_cb[ cb_pos ] ;
    
    if      ( hi_squares > ( lo_squares << 1 ) ) { fourier_cb[ cb_pos ] =  1 ; }
    else if ( lo_squares > ( hi_squares << 1 ) ) { fourier_cb[ cb_pos ] = -1 ; }
    else                                         { fourier_cb[ cb_pos ] =  0 ; }
  
    fourier_sum += fourier_cb[ cb_pos ] ;
  
    // and finally, force a super-majority before changing the symbol
    if      ( fourier_sum >=  4 ) { current_symbol = AX25_SPACE ; }
    else if ( fourier_sum <= -4 ) { current_symbol = AX25_MARK ; }
    else                          { ; } // inconclusive - dont change it
  
  
    //=============================
    //   clock and bit recovery
    //=============================

    // increment # of samples since last symbol change
    if ( ++since_last_chg > 200 ) { since_last_chg = 200 ; }
    thesebits = 0 ;
	
    if ( debug ) 
    {
		printf( "%d (%d): HI %d LO %d = %d FS %d CS %d SLC %d\n" ,
			sampnum, inaframe, hi_squares, lo_squares, 
			fourier_cb[cb_pos], fourier_sum, current_symbol, since_last_chg ) ;
    }
  
    	
    // the in-a-frame and seeking-frame modes require 
	// different strategies - let's split them up here

	
	if ( inaframe ) 
	{
		//================================
		// clock recovery within a frame
		//================================
		
		bit_timer-- ;
		if ( current_symbol != last_symbol ) 
		{
			if ( debug ) 
			{ 
				printf( "%d (%d): SLC %d BT %d\n", sampnum, inaframe, 
					since_last_chg, bit_timer ) ; 
			}
		
			// save the new symbol
			last_symbol = current_symbol ;
			// reset counter
			since_last_chg = 0 ;
			
			// check sync
			// for inaframe, bit_timer should be 4 when
			// symbol changes. 5,6 are fine, 3,2 are fine,
			// 7,8 and 0,1 need adjustment, 9,10 are out of bounds
			if ( ( bit_timer == 7 ) || ( phase_err == 8 ) )
			{
			    // drifting a little (other station is slow
				// or we're fast). nudge timer.
				bit_timer -= 1 ;
				if ( debug ) 
				{
					printf( "%d (%d): Nudging timer down one.\n" , 
						sampnum, inaframe ) ;
				}		
			}
			else if ( ( bit_timer == 0 ) || ( bit_timer == 1 ) )
			{
			    // they're fast or we're slow
				bit_timer += 1 ;
				if ( debug )
				{
					printf( "%d (%d): Nudging timer up one.\n" , 
						sampnum, inaframe ) ;
				}		
			}	
			else if ( ( bit_timer == 9 ) || ( phase_err == 10 ) )
			{
				// too much error
				if ( ++sync_err_cnt > MAX_SYNC_ERRS ) 
				{
					if ( debug ) 
					{
						printf("%d: Phase errs > %d! Cancel frame & clear buffers!\n" , 
							sampnum, MAX_SYNC_ERRS) ;
					}		
					dcd = 0 ;	
					sync_err_cnt = 0 ;
					msg_pos = 0 ;
					inaframe = 0 ;
					bitq = 0 ;
					bitqlen = 0 ;
					bit_timer = 0 ;
					bittimes = 0 ;
					// turn off DCD light
					//PORTB &= ~0x80 ;
					return 0 ;
				}
				else
				{
					if ( debug ) 
					{
						printf( "%d: Added a sync error - up to %d.\n" ,
							sampnum, sync_err_cnt ) ;
					}		
				}
			} // end bit_timer cases
		} // end if symbol change
		
		//=============================
		// bit recovery within a frame
		//=============================
		
		if ( bit_timer == 0 )
		{
			if ( debug ) 
			{ printf( "%d (%d): Bit timer = 0\n", sampnum, inaframe ) ; }
			
			// reset timer
			bit_timer = 11 ;
			// another bit time has expired
		    bittimes++ ;
			
			// wait for a symbol change to clock in some bits
			if ( current_symbol != last_symbol_inaframe ) 
			{ 
				// add one as ready-to-decode flag
				thesebits = bittimes + 1 ; 
				bittimes = 0 ;
				last_symbol_inaframe = current_symbol ;
			}
		} // end if bit_timer==0 

	} // end if inaframe

	else
	{
		//=================
		// not in a frame
		//=================
		
		// housekeeping
		// phase_err = since_last_change MOD 11
		phase_err = since_last_chg ;
		while ( phase_err >= 11 ) { phase_err -= 11 ; }
	
		// elapsed bittimes = round (since_last_chg / 11)
		bittimes = 0 ;
		test = since_last_chg + 5 ;
		while ( test > 11 ) { test -= 11 ; bittimes++ ; }
		thesebits = 0 ;
	
		//====================================
		// clock recovery NOT within a frame
		//====================================
		
		// need a symbol transition to sync or to clock in bits
		if ( current_symbol == last_symbol ) 
		{ return 0 ; }
	
		else	
		{
			// symbol change 
			if ( debug ) 
			{ 
				printf( "%d (%d): SLC %d PE %d\n", sampnum, inaframe, 
					since_last_chg, phase_err ) ; 
			}
	
			// save the new symbol, reset counter
			last_symbol = current_symbol ;
			since_last_chg = 0 ;
	
			// check bit sync
			if ( ( phase_err >= 4 ) && ( phase_err <= 7 ) )
			{
				// too much error
			    bitq = 0 ;
				bitqlen = 0 ;
				dcd = 0 ;
				// PORTB &= ~0x80 ;
			}	
			else
			{
				// good timing
				if ( ++dcd > 200 ) { dcd = 200 ; }
				if ( dcd > MIN_DCD ) { ; } //PORTB |= 0x80 ; }
			}
		
			// save these bits
			thesebits = bittimes + 1 ;
			
		} // end else ( = symbol change)	
          
	} // end else ( = not inaframe)
	
	
	//========================================
	//   bit recovery, in or out of a frame
	//========================================
	
	
	if ( thesebits == 0 ) { return 0 ; }
	else                  { thesebits-- ; }
	
	switch ( thesebits )
	{
	case 1: break ;    // no ones to add ------> binary       "0"
                          
	case 2: bitq |= ( 0x01 << bitqlen ) ;     // binary      "01"
			break ;
                           
	case 3: bitq |= ( 0x03 << bitqlen ) ;     // binary     "011"
			break ;

	case 4: bitq |= ( 0x07 << bitqlen ) ;     // binary    "0111"
			break ;
                           
	case 5: bitq |= ( 0x0F << bitqlen ) ;     // binary   "01111"
			break ;
                           
	// "six" is a special case - drop zero, only add 5 bits!
	case 6: bitq |= ( 0x1F << bitqlen ) ;     // binary   "11111"
			thesebits = 5 ;
			break ;
                  
	// "seven" is another special case - only legal for an HDLC byte			  
	case 7: if ( bitqlen > 0 )                // binary "0111111"
	        {
			    // some bits still pending
				// fill out that byte and append the full HDLC char
				// the byte recovery code below will handle the rest
				bitq = ( bitq & 0xFF ) | 0x7E00 ;
				bitqlen = 16 ;
			}
			else
			{
			    // nothing pending
				// clear the queue and insert an HDLC
				// the byte recovery code below will handle the rest
				bitq = 0x7E ;
				bitqlen = 8 ;
			}
			// do not add to bitqlen (below)
			thesebits = 0 ;
			break ;
  
	default: 
			// less than a full bit or more than seven have elapsed
			// clear buffers
			if ( debug ) { printf( "%d: Bitstuff error!\n", sampnum ) ; }
			msg_pos = 0 ;
			inaframe = 0 ;
			bitq = 0 ;
			bitqlen = 0 ;
			// do not add to bitqlen (below)
			thesebits = 0 ;
			// turn off DCD light
			//PORTB &= ~0x80 ;
			break ;
                   
	} // end switch

	// how many bits did we add?
	bitqlen += thesebits ;
              
	
	//===================
    //   byte recovery
    //===================

    // got our bits in a row.
    // now let's talk about bytes.

    // check the bit queue, repeat if necessary
    while ( bitqlen >= 8 )
    {
        // take the bottom 8 bits to make a byte
        byteval = bitq & 0xFF ;

        // special case - HDLC frame marker
        if ( byteval == 0x7E )
        {
            if ( inaframe == 0 ) 
            {
			    if ( debug ) { printf( "%d: HDLC starter found!\n" , sampnum ) ; }
                inaframe = 1 ;
				sync_err_cnt = 0 ;
				bit_timer = 15 ;
				bittimes = 0 ;
				// turn on the DCD light
                //PORTB |= 0x80 ;
                // pop entire byte (later)
                popbits = 8 ;
            }	

            else if ( msg_pos < MIN_PACKET_LEN )
            {
                // We are already in a frame, but have not rec'd any/enough data yet.
                // AX.25 preamble is sometimes a series of HDLCs in a row, so 
                // let's assume that's what this is, and do nothing.
				if ( debug ) 
				{ printf( "%d: Another HDLC - ignoring.\n" , sampnum ) ; }
                popbits = 8 ;
            }    
           
            else     
            {
                // in a frame, and have some data,
                // so this HDLC is probably an ender
				// (and maybe also a starter)
				if ( debug ) 
                { printf( "%d: msg_pos = %d\n" , sampnum, msg_pos ) ; }
				
                if ( msg_pos > 0 )
                {
                    printf( "Message was:" ) ;
                    for ( x=0 ; x < msg_pos ; x++ )
                    {
                        printf( " %02X", msg[x] ) ;
                    }    
                    printf( "\n" ) ; 
                }

                msg_pos = 0 ;
			    // ...set the flag instead
                msg_ready = 1 ;
                
				// stay in inaframe-mode, in case a new frame is starting
				sync_err_cnt = 0 ;
				bittimes = 0 ;
				//inaframe = 0 ;
				// turn off the DCD light
                //PORTB &= ~0x80 ;
                // pop entire byte (later)
                popbits = 8 ;
              
            }  // end else for "if not inaframe"

        }  // end if byteval = 0x7E

        else if ( inaframe == 1 ) 
        {
            // not an HDLC frame marker, but =is= a data character
            // add it to the incoming message
            msg[ msg_pos ] = byteval ;
            msg_pos++ ;
            // pop entire byte (later)
            popbits = 8 ;
            if ( debug ) { printf( "%d (%d): Add byte %02X\n" , 
		                      sampnum, inaframe, byteval ) ; }
        }    

        else
        { 
            // not already in a frame, and this byte is not a frame marker.
            // It is possible (likely) that when an HDLC byte arrives, its
            // 8 bits will not align perfectly with the 8 we just checked.
            // So instead of dropping all 8 of these bits, let's just drop
            // one. This increases our chances of seeing the HDLC byte in
            // the incoming bitstream.
            popbits = 1 ; 
        }

        // pop the used bits off the end 
        // (i know, not really a "pop")
        bitq = ( bitq >> popbits ) ;
        bitqlen -= popbits ;

    } // end while bitqueue longer than 8

    // debug: check timing
    //end_time = TCNT3 ;

    //sei() ;
    return 0 ;
}  // end timer3 interrupt


/*
void decodeax25 (void)
{
    x = 0 ;
    decode_state = 0 ;     // 0=header, 1=got 0x03, 2=got 0xF0

    //debug( "Decode routine - rec'd " . length($pkt) . " bytes." ) ;

    // lop off last 2 bytes (FCS checksum, which we're not sending to the PC)
    for ( x = 0 ; x < msg_pos-2 ; x++ )
    {
        if ( decode_state == 0 ) 
        {
            // in the header
	        if ( msg[x] == 0x03 ) { decode_state = 1 ; }
            else 
            {
                if ( msg[x] == 40 ) { Serial.print( "_" ) ; }
                else                { Serial.print( ( msg[x] >> 1 ) , BYTE ) ; }
            }
        }	

        else if ( decode_state == 1 ) 
        {
            // got the 0x03, waiting for the 0xF0
            if ( msg[x] == 0xF0 ) { decode_state = 2 ; }
            else 
            {
                // wow - corrupt packet? abort
                Serial.println( "" ) ;
 	            return ;
            }
        }

        else if ( decode_state == 2 )
        {
            Serial.print( msg[x] , BYTE ) ;
        }	

    } // end for	

    Serial.println( "" ) ;
}
*/
	
// end of file

