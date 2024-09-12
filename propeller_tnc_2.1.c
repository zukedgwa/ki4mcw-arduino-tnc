/**
 * @file Test2.1.c
 * This is the main Test2.1 program start point.
 */

#define MIN_PACKET_LEN   10
#define PACKET_SIZE      200
#define AX25_MARK        0
#define AX25_SPACE       1
#define TICKS13200       6061

#define LW_MAIN_PIN      18
#define LW_MAIN_ON       (OUTA |= (1<<LW_MAIN_PIN))
#define LW_MAIN_OFF      (OUTA &= ~(1<<LW_MAIN_PIN))

#define LW_SAMPLER_PIN   19
#define LW_SAMPLER_ON    (OUTA |= (1<<LW_SAMPLER_PIN))
#define LW_SAMPLER_OFF   (OUTA &= ~(1<<LW_SAMPLER_PIN))

#define HB_PIN           20
#define HB_ON            (OUTA |= (1<<HB_PIN))
#define HB_OFF           (OUTA &= ~(1<<HB_PIN))

#define DCD_PIN          21
#define DCD_ON           (OUTA |= (1<<DCD_PIN))
#define DCD_OFF          (OUTA &= ~(1<<DCD_PIN))

#define ACT_PIN          23
#define ACT_ON           (OUTA |= (1<<ACT_PIN))
#define ACT_OFF          (OUTA &= ~(1<<ACT_PIN))

#define ADC_PIN          16

// one of these please - not both, not neither
//#define DECODE_KISS      1
#define DECODE_ASCII     1

#include <stdio.h>
#include <propeller.h>
#include <inttypes.h>
#include "cog.h"
#include <sys/thread.h>


static void sample_timer(void* dummy) ;
static void get_sample(void) ;
static void reset_all(void) ;
static void decode_bits(uint8_t samples) ;
static void decode_ax25(void) ;
static void robprintf(uint8_t val) ;


int             samples_cnt_lock ;         // changing the value of sampler_cnt

uint8_t         samples_cnt,               // number of samples since the last symbol change 
                inaframe,                  // rec'd start-of-frame marker
                bitqlen,                   // number of rec'd bits in bitq
                popbits,                   // number of bits to pop (drop) off the end
                byteval,                   // rec'd byte, read from bitq
                msg[PACKET_SIZE + 1],      // rec'd data
                msg_pos,                   // bytes rec'd, next array element to fill
                x ;                        // misc counter
                    
uint32_t        bitq,                      // rec'd bits awaiting grouping into bytes
                nextcnt,                   // time for our next sample (used by waitcnt)
                lastcnt ;                  // monitor elapsed time

static int      sampler_stack[200];        // no idea how big this should be


int main(void)
{
    uint8_t     sampler_cog,               // save the number of the spawned COG (1-7)
                elapsed_samples ;          // samples elapsed since the last symbol change (0-200)

    // =========
    // setup
    // =========

    samples_cnt_lock = locknew() ;
    DIRA |= (1<<ACT_PIN) ;
    DIRA |= (1<<LW_MAIN_PIN) ;

    // ============
    // settle time
    // ============

    ACT_ON ; LW_MAIN_ON ;
    waitcnt( CNT + (2 * CLKFREQ) ) ;

    ACT_OFF ; LW_MAIN_OFF ;
    printf( "Starting Prop TNC test v2 at %d Hz...\n" , CLKFREQ ) ;
    waitcnt( CNT + (2 * CLKFREQ) ) ;

    // ======================
    // launch sampler thread 
    // in a(nother) cog
    // ======================    

    sampler_cog = cogstart( &sample_timer, NULL, sampler_stack, sizeof(sampler_stack) );
    printf( "Started sampler in COG %d\n" , sampler_cog ) ;

    ACT_ON ; LW_MAIN_ON ;
    waitcnt( CNT + (2 * CLKFREQ) ) ;
    ACT_OFF ; LW_MAIN_OFF ;

    // =======
    //   go!
    // =======

    while(1)
    {
        if ( samples_cnt )
        {
            ACT_ON ;
            elapsed_samples = samples_cnt ;
            while ( lockset( samples_cnt_lock ) == -1 ) { LW_MAIN_ON ; }
            samples_cnt = 0 ;
            lockclr( samples_cnt_lock ) ;
            LW_MAIN_OFF ;
            decode_bits( elapsed_samples ) ;
        }

        // pause, but not too long
        //waitcnt( CNT + 3000 ) ;
        ACT_OFF ;
    }

    // should never exit
}


static void sample_timer(void* dummy)
{
    // main function for sampler COG
    // calls sampler code at regular intervals

    long sampletime ;

    // =======
    // setup
    // =======

    // set data direction register for this COG
    DIRA |= (1<<HB_PIN) | (1<<DCD_PIN) | (1<<LW_SAMPLER_PIN) ;
    DIRA &= ~(1<<ADC_PIN) ;
    
    // blink lights to show COG is active, let things settle
    waitcnt( CNT + ( CLKFREQ * 3 )) ;
    HB_ON ; DCD_ON ; LW_SAMPLER_ON ;
    waitcnt( CNT + ( CLKFREQ * 3 )) ;
    HB_OFF ; DCD_OFF ; LW_SAMPLER_OFF ;

    while(1)
    {
        // set alarm for the end of this cycle time
        sampletime = CNT + TICKS13200 ;

        // go
        get_sample() ;
        
        // wait for the alarm
        waitcnt( sampletime ) ;
    }
    // should never exit
}



static void get_sample(void)
{
    // ADC sampler and low-level decode
    //
    // This code is called 13,200 times per second - this is 11x the bit rate for
    // 1200 baud AX.25. This rate is an even multiple of the two audio frequencies
    // used for this protocol (11 x 1200 Hz and 6 x 2200 Hz), so performs better
    // than a 9600 Hz sample rate (in my mind, at least). 
    //
    // Note that 13K2 sps is only 6061 ticks of an 80MHz clock per sample. This 
    // code has to run very quickly, with plenty of time left over for calls, 
    // returns, and housekeeping within every sample cycle. 4000 ticks of CNT is
    // a good high-water mark. The code from COG0/main that uses the output of the
    // sampler is not nearly as time-sensitive, so can take maybe 8-10 times as
    // many CNT ticks per cycle without affecting anything.

    static int16_t  mult_cb[7],                // circular buffer for adcval*delay values (+/- 48^2)
                    mult_sum ;                 // sum of mult_cb values (+/- 7 * (48^2))

    static int8_t   adcval,                    // zero-centered, de-biased ADC input (+/- 48)
                    adc_bias,                  // calculated off-center-ness of 1-bit ADC (+/- 16)
                    bias_sum,                  // sum of last 128 raw ADC readings (0-128)
                    adc_cb[6] ;                // delay line for adc readings (+/- 48)

    static uint8_t  rawadc,                    // value read directly from INA register (0 or 1)
                    bias_cnt,                  // count of ADC samples collected (0-128)
                    since_last_chg,            // samples since the last MARK/SPACE symbol change (0-201)
                    since_last_chg_save,       // samples since the last MARK/SPACE symbol change, saved (0-201)
                    current_symbol,            // MARK or SPACE (0 or 1, via macro)
                    last_symbol,               // MARK or SPACE from previous reading (0 or 1)
                    adc_cb_pos,                // position within the circular buffer of adc readings (0-6)
                    mult_cb_pos,               // position within the circular buffer of adc multiples (0-7)
                    hb12 ;                     // heartbeat @ 1200 Hz (0 or 1)

    
    // beat heart
    if ( hb12 ) { hb12 = 0 ; HB_OFF ; } 
    else        { hb12 = 1 ; HB_ON ; }

    // acquire
    rawadc = ( INA & (1<<ADC_PIN) ) ? 1 : 0 ;

    // =============    
    // bias mgmt
    // =============

    bias_sum += rawadc ;
    if ( ++bias_cnt == 128 )
    {
        adc_bias = ( bias_sum >> 1 ) - 32 ;
        if ( adc_bias >  16 ) { adc_bias =  16 ; }
        if ( adc_bias < -16 ) { adc_bias = -16 ; }
        bias_cnt = 0 ;
        bias_sum = 0 ;
    }
    adcval = ( rawadc ? 32 : -32 ) - adc_bias ;
    
    // =================
    // symbol detection
    // =================

    // multiply adc input times value from 6 samples ago
    // sum the last 7 of those multiples
    if ( ++mult_cb_pos == 7 ) { mult_cb_pos = 0 ; }
    if ( ++adc_cb_pos  == 6 ) { adc_cb_pos  = 0 ; }
 
    mult_sum -= mult_cb[ mult_cb_pos ] ;
    mult_cb[ mult_cb_pos ] = adcval * adc_cb[ adc_cb_pos ] ;
    mult_sum += mult_cb[ mult_cb_pos ] ;
    adc_cb[ adc_cb_pos ] = adcval ;
  
    if      ( mult_sum >=  100 ) { current_symbol = AX25_SPACE ; }
    else if ( mult_sum <= -100 ) { current_symbol = AX25_MARK ; }
  
    // ======================
    // symbol chg detection
    // ======================
  
    if ( current_symbol == last_symbol ) 
    { 
        if ( ++since_last_chg > 200 ) { since_last_chg = 200 ; }
    }
    else 
    { 
        since_last_chg_save = since_last_chg ;
        since_last_chg = 0 ;
        last_symbol = current_symbol ;
    }

    // if last change was stable, pass it back to COG0
    if ( since_last_chg == 6 )
    {
        while ( lockset( samples_cnt_lock ) == -1 ) { LW_SAMPLER_ON ; }
        samples_cnt = since_last_chg_save ;
        lockclr( samples_cnt_lock ) ;
        LW_SAMPLER_OFF ;
    }
    return ;
}


static void decode_bits(uint8_t samples)
{  
    // process data from sampler cog. called by main.
    // [samples] is the number of elapsed sampler cycles since a symbol change (0-200)
    //   this forms the basis for clock recovery and data detection
                                    
    static uint8_t      bit_times,                 // elapsed bit durations (11 samples each)
                        timing_err_cb_pos  ;       // position within the circular buffer of timing errors (0-6)

    static int16_t      temp,                      // temp variable, based on samples (-10 to 205)
                        timing_err ;               // clock drift for this symbol change (briefly -10 to 200)

    static int8_t       timing_err_sum,            // clock drift, accumulated (+/- 60)
                        timing_err_cb[6] ;         // clock drift for each of the past few symbol changes (+/- 10)


    // tweak the elapsed-samples count if it is consistently off
    if      ( timing_err_sum >=  3 ) { samples-- ; }
    else if ( timing_err_sum <= -3 ) { samples++ ; }

    // check timing error, get elapsed bit times
    // avoid this FP math:  bit_times = round(samps_btw_chgs / 11)
    bit_times = 0 ;
    temp = samples + 5 ; // for rounding the elapsed bit_times w/o changing the value of samples
    timing_err = samples ;
    while ( temp >= 11 ) { temp -= 11 ; bit_times++ ; timing_err -= 11 ; }
  
    // check recent clock error, update history
    if ( ++timing_err_cb_pos == 6 ) { timing_err_cb_pos = 0 ; }
    timing_err_sum -= timing_err_cb[ timing_err_cb_pos ] ;
    timing_err_cb[ timing_err_cb_pos ] = timing_err ;
    timing_err_sum += timing_err_cb[ timing_err_cb_pos ] ;

    //printf( "CS %1d SBC %3d PE %2d PE5 %3d TEH %2d %2d %2d %2d %2d %2d\n" , 
    //current_symbol , samps_btw_chgs , timing_err , timing_err_sum, 
    //timing_err_hist[5], timing_err_hist[4], timing_err_hist[3], 
    //timing_err_hist[2], timing_err_hist[1], timing_err_hist[0]    ) ; 

    // =======================
    // convert times -> bits
    // =======================

    // the "elapsed bit times" indicate the time since the last symbol change.
    // an AX25 symbol change denotes a binary "0", no-change = binary "1".

    switch ( bit_times )
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
                           
        case 6: // "six" is a special case:       // binary   "11111"
                // the zero bit was just a filler ("bitstuff") to force a 
                // symbol change, so that we would not lose clock sync. 
                // to decode, just drop the zero, and only add the 5 
                // "one" bits that preceeded it

                bitq |= ( 0x1F << bitqlen ) ;
                bit_times = 5 ;
                break ;
                  

        case 7: // "seven" is a special case      // binary "0111111"
                // this many zeros in a row is only legal for an HDLC byte
                // (used as a start/end-of-packet flag, and also as the
                // long squelch-break tone at the beginning of a packet)

                if ( bitqlen >= 4 ) // partial byte before HDLC
                { bitq = 0x7E00 | ( bitq & 0xFF ) ; bitqlen = 16 ; }
                              
                else    // all other cases - drop the rest, keep the HDLC
                { bitq = 0x7E ; bitqlen = 8 ; }
                    
                // we've already made the necessary adjustments to bitqlen,
                // so do not add anything else (below)          
                bit_times = 0 ;
                break ;

        default:
                // less than a full bit (?), or more than seven have elapsed
                // neither case is usable - clear buffers
                //printf( "Bitstuff overrun!\n" ) ;
                reset_all() ;
                return ;
                break ;

    } // end switch

    bitqlen += bit_times ;
    //printf( "--- BITQ %08X LEN %d MSGPOS %d\n" , bitq  , bitqlen, msg_pos ) ;

    // =========================
    // check bit queue 
    // (maybe more than once) 
    // for completed bytes
    // =========================

    while ( bitqlen >= 8 )
    {
        // take the bottom 8 bits to make a byte
        byteval = bitq & 0xFF ;

        // special case - HDLC frame marker
        if ( byteval == 0x7E )
        {
            if ( inaframe == 0 )
            {
                // ======================
                // start of frame (maybe)
                // ======================
                inaframe = 1 ;
                DCD_ON ;
                popbits = 8 ;
            }
      
            else if ( msg_pos < 10 )
            { popbits = 8 ; } // runt packet, or additional HDLC
      
            else
            {
                // ==============
                // end of frame
                // ==============

                decode_ax25() ;

                // stay inaframe, in case this also marks a frame start
                // reset msg buffer to the beginning, and pop this HDLC
                msg_pos = 0 ;
                popbits = 8 ;
            }
        }  // end if byte is an HDLC

        else if ( inaframe == 1 ) 
        {
            // data char
            msg[ msg_pos ] = byteval ;
            msg_pos++ ;
            popbits = 8 ;
            //printf( "-- Adding 0x%02X MSGPOS %3d.\n" , byteval , msg_pos ) ;
        }

        else
        { popbits = 1 ; }   // not in a frame, and this is not an HDLC
                            // cycle through the bit queue one bit at a time, 
                            // until we uncover an HDLC starting a new packet

        bitqlen -= popbits ;
        bitq = ( bitq >> popbits ) ;  // not really a pop, more like a shift to oblivion

    } // end while bitqlen >= 8

    //printf( "Took %d ticks\n" , CNT-lastcnt ) ;
    return ;  
}  


static void reset_all(void)
{
    msg_pos = 0 ;
    inaframe = 0 ;
    bitq = 0 ;
    bitqlen = 0 ;
    DCD_OFF ;
    return ;
}

// =================================
//
//  CONDITIONAL CODE SECTIONS
//
// =================================


#ifdef DECODE_KISS

static void decode_ax25(void)
{
    // Take the data in the msg array, and send it out the serial port.
  
    uint8_t     found_it,               // found it
                decode_state = 0 ;      // which part of the decode process we're in
                    // state table:
                    // 0 = just starting, need to add header to output
                    // 1 = in the header 
                    // 2 = found the 0x03 end-of-header byte
                    // 3 = found the 0xF0 start-of-data byte, are adding payload data
    
    // ==============
    // sanity check
    // ==============

    // a valid packet must have an 0x03 followed by an 0xF0,
    // and the 0x03 must occur in a non-zero (n*7)th byte.
    // so let's check

    found_it = 0 ;

    // ignore last 2 bytes of msg (FCS checksum, not used here)
    for ( x = 7 ; x < (msg_pos - 2) ; x+=7 )
    {
        if (( msg[x] == 0x03 )&&( msg[x+1] == 0xF0 )) { found_it++ ; }
    }        

    if ( found_it != 1 ) 
    {
        printf( "== Bad packet ==%c%c" , 10 , 13 ) ;
        return ;
    }

    // ==================================    
    // packet is good - format and send
    // ==================================    
    
    // ignore last 2 bytes of msg (the FCS checksum again)
    for ( x = 0 ; x < (msg_pos - 2) ; x++ )
    {
        switch ( decode_state )  
        {
        case 0:  
            // just starting
            printf( "%c" , 0xC0 ) ;  // frame start/end marker
            printf( "%c" , 0x00 ) ;  // data on port 0
            printf( "%c" , msg[x] ) ;
            decode_state = 1 ;
            break ;
        
        // note the goofy order!!
        case 2: 
            // got the 0x03 end-of-header flag, 
            // this should be the 0xF0 start-of-data flag
            if ( msg[x] == 0xF0 ) 
            { 
                printf( "%c" , msg[x] ) ;
                decode_state = 3 ; 
            }
            else 
            {
                // wow - corrupt packet? abort
                printf( "%c%c" , 13, 10 ) ;
                return ;
            } 
            break ;
            
        // note the goofy order again!!    
        case 1: 
            // in the header
            // watch for end-of-header char
            if ( msg[x] == 0x03 ) 
            { 
                printf( "%c" , msg[x] ) ;
                decode_state = 2 ; 
                break ; 
            }
            // else fall through
        
        default:
            // payload, or header fall-through
            // escape special characters 
            if ( msg[x] == 0xC0 ) { printf( "%c" , 0xDB ) ; }
            printf( "%c" , msg[x] ) ;
            if ( msg[x] == 0xDB ) { printf( "%c" , 0xDD ) ; }
            break ;
        }    

    } // end for    

    printf( "%c" , 0xC0 ) ;      // end of frame
    printf( "%c%c", 13, 10 ) ;   // CR LF
}

#endif
#ifdef DECODE_ASCII

static void decode_ax25(void)
{
    // Take the data in the msg array, and send it out the serial port.
  
    uint8_t     found_it,               // found it
                payload_start,          // msg char where the fun begins
                decode_state = 0 ;      // which part of the decode process we're in
                    // state table:
                    // 0 = just starting, need to add header to output
                    // 1 = in the header 
                    // 2 = found the 0x03 end-of-header byte
                    // 3 = found the 0xF0 start-of-data byte, are adding payload data
    
    // ==============
    // sanity check
    // ==============

    // a valid packet must have an 0x03 followed by an 0xF0,
    // and the 0x03 must occur in a non-zero (n*7)th byte.
    // so let's check

    found_it = 0 ;

    // ignore last 2 bytes of msg (FCS checksum, not used here)
    for ( x = 7 ; x < (msg_pos - 2) ; x+=7 )
    {
        if (( msg[x] == 0x03 )&&( msg[x+1] == 0xF0 )) { found_it++ ; payload_start = x + 2 ;}
    }        

    if ( found_it != 1 ) 
    {
        printf( "== Bad packet ==%c%c%c%c" , 10 , 13 , 10, 13 ) ;
        return ;
    }

    // ==================================    
    // packet is good - format and send
    // ==================================    
    
    // ignore last 2 bytes of msg (the FCS checksum again)
    for ( x = 0 ; x < (msg_pos - 2) ; x++ )
    {
        switch ( decode_state )  
        {
        case 0:  
            // just starting
            printf( "KISS packet is:\n" ) ; 
            printf( "<0xC0>" ) ;  // frame start/end marker
            printf( "<0x00>" ) ;  // data on port 0
            printf( "%c" , msg[x] >> 1 ) ;
            decode_state = 1 ;
            break ;

        case 1: 
            // in the header
            // watch for end-of-header char
            if ( msg[x] == 0x03 ) 
            { 
                printf( "<0x03>" ) ;
                decode_state = 2 ; 
            }
            else if (( msg[x] >= 96 )&&( msg[x] <= 115 )) { printf( "%c" , msg[x] >> 1 ) ; }
            else if (( msg[x] >= 130 )&&( msg[x] <= 181 )){ printf( "%c" , msg[x] >> 1 ) ; }
            else if ( msg[x] == 64 )                      { printf( " " ) ; }
            else                                          { printf( "<%d>" , msg[x] ) ; }
             
            break ; 
        
        case 2: 
            // got the 0x03 end-of-header flag, 
            // this should be the 0xF0 start-of-data flag
            if ( msg[x] == 0xF0 ) 
            { 
                printf( "<0xF0>\n" ) ;
                decode_state = 3 ; 
            }
            else 
            {
                // wow - corrupt packet? abort
                printf( "%c%c" , 10, 13 ) ;
                printf( "%c%c" , 10, 13 ) ;
                return ;
            } 
            break ;
            
        default:
            // payload
            // escape special characters 
            if      ( msg[x] == 0xC0 ) { printf( "<0xDB><0xC0>" ) ; }
            else if ( msg[x] == 0xDB ) { printf( "<0xDB><0xDD>" ) ; }
            else if ( msg[x] < 32 )    { printf( "<%d>" , msg[x] ) ; }
            else if ( msg[x] > 126 )   { printf( "<%d>" , msg[x] ) ; }
            else                       { printf( "%c" , msg[x] ) ; }

            if      ( x - payload_start == 40  ) { printf( "\n" ) ; }
            else if ( x - payload_start == 80  ) { printf( "\n" ) ; }
            else if ( x - payload_start == 120 ) { printf( "\n" ) ; }
            else if ( x - payload_start == 160 ) { printf( "\n" ) ; }

            break ;

        } // end switch

    } // end for    

    printf( "<0xC0>" ) ;         // end of frame
    printf( "%c%c", 10, 13 ) ;   // LF CR
    printf( "%c%c", 10, 13 ) ;   // LF CR
}

#endif

/*
static void robprintf(uint8_t val)
{
    // only print printable chars (ASCII 32-126)
    // otherwise, print char as "<XX>" where XX 
    // is the value in hex
 
    if ( KISS_FORMAT )                      { printf( "%c" , val ) ; }
    //else if (( val >= 32 )&&( val <= 126 )) { printf( "%c" , val ) ; }
    else                                    { printf( " %02X" , val ) ; }

    return ;
}
*/


// END OF FILE