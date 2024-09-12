/* 
    Semi-working port of Scott Miller N1VG's OpenTracker 1+ code to the
    ArduinoMEGA (based on the ATmega1280 AVR controller). Only the receive
    functions are implemented here, mostly from modules "discriminator.c" 
    and "kiss.c". The code here is a mix of vanilla AVR-GCC and the Arduino
    extensions, the latter mostly just for the convenince of the Serial 
    library. Likewise, the Arduino hardware was used because (a) it was 
    on my desk looking for something to do, and (b) my one serial port
    was already occupied.
    
    This code is entirely experimental, and should not be confused in any 
    way with Scott's excellent products, of which I own a few. If you 
    have any trouble with this code, for God's sake quit goofing around 
    and go buy something from Scott. The OT1+ kit only takes an hour or 
    two to build, and you get street cred for your mad soldering skillz.
    Alternatively, the OT2 has a scripting language built in, so you can 
    do your programming there and re-write all of your friends' data when
    you digi them. Or something. Seriously, http://www.argentdata.com
    
    Rob KI4MCW 2010-03-26
*/


// =================== DEBUG FLAG =======================
// Enable the define for DEBUG_ON to have all packet data
// output on the USB/serial port in HEX. Comment this out
// (or better, change it to "DEBUG_OFF") for normal KISS
// output.
#define DEBUG_ON


// RM: defs - Scott's, from "discriminator.c"
#define DELAY_MASK       0x10 
#define SAMPLE_BIT       4
#define WINDOW_HIGH      5
#define WINDOW_LOW       3
#define PLL_ERROR_LIMIT  12 
#define PACKET_DATA_SIZE 200
#define MIN_PACKET_SIZE  18

// RM: defs - Scott's, from "kiss.c"
#define FEND   0xC0
#define FESC   0xDB
#define TFEND  0xDC
#define TFESC  0xDD
#define STATE_FRAME_WAIT  0
#define STATE_DATA        6

// RM: defs - mine
// Timer3 TOP value = 16MHz / 9600Hz = 1667, or about 6H 131L
#define T3TOPH 6
#define T3TOPL 128
// Blinkies
#define HIGH   1
#define LOW    0
// Truth
#define TRUE   1
#define FALSE  0
// error flag for incoming packet, used with err_state
#define ERR_OVERFLOW  6
#define ERR_PLL_FAIL  5
#define ERR_PLL_WARN  4
#define ERR_BS_FAIL   3
#define ERR_BS_WARN   2 
#define ERR_JUSTCLEAR 1
#define ERR_NONE      0
// action flag for packet, used with pkt_out_state
#define PKT_OUT_READY   1
#define PKT_OUT_IDLE    0

// RM: vars - Scott's (but my comments)
unsigned char adc,                       // input from adc
              dline,                     // delay line for input voltage slope
              xline,                     // delay line for XORing slope against earlier sample
              sline,                     // delay line for symbol changes
              lpf,                       // for scoring current symbol
              sample_num,                // position within 8 samples/bit
              sample_cnt,                // for scoring current symbol
              lastbit,                   // just like it sounds
              rxbyte,                    // current incoming byte
              bitcount,                  // bits collected so far for this byte
              bitstuff,                  // number of 1's since the last 0
              sw_dcd,                    // not used here, but left it in
              frame_lock,                // flag - timing and markers good so far
              pll_error,                 // number of consecutive timing errors 
              packet_data_len,           // bytes so far in incoming data buffer
              kiss_tx_state = STATE_FRAME_WAIT ;  // progress in sending KISS data to PC
signed char tap1 ;            // zero-biased value of previous adc reading
char packet_data[PACKET_DATA_SIZE + 1] ; // incoming packet buffer


// RM: vars - mine
unsigned char x,                         // misc counter
              hb12,                      // heartbeat flag for pin 12
              pkt_out_state,             // action for outgoing packet
              err_state,                 // error flag for incoming packet
              err_val,                   // optional numeric err value to report (debug mode)
              packet_out_len;            // byte length of outbound packet data
char packet_out[PACKET_DATA_SIZE + 1] ;  // outbound (to PC) data buffer


// RM: protos - Scott's
void kiss_tx_byte(unsigned char c) ;
void kiss_tx_done(void) ;


// RM: Arduino/AVR-specific code - mine
void setup()
{
    Serial.begin(57600) ;
    // RM: set up ADC
    ADMUX   = 0x00 ;                    // channel0
    ADMUX  |= (1<<REFS0) ;              // ref to AVCC (+5v)
    ADMUX  |= (1<<ADLAR) ;              // left-justified (only need 8 bits)
    ADCSRA |= (1<<ADPS2) | (1<<ADPS1) ; // prescale 64
    ADCSRA |= (1<<ADATE) ;              // auto-trigger (free-run)
    ADCSRB  = 0x00 ;                    // free-running
    ADCSRA |= (1<<ADEN) ;               // enable ADC
    DIDR0  |= (1<<ADC0D) ;              // disable digital driver on ADC0 
    PRR0   &= ~(1<<PRADC) ;             // turn off power reduction to ADC
    ADCSRA |= (1<<ADSC) ;               // trigger first conversion  
  
    // RM: use 16-bit Timer3 (not Timer1 - conflicts with Arduino delay fns)
    // to check on things at 8x the 1200bps signalling rate (9600 Hz)
    TCCR3A = 0x00 ;
    TCCR3B = (1<<WGM32) | (1<<CS30) ;
    TCCR3C = 0x00 ; 
    OCR3AH = T3TOPH ; 
    OCR3AL = T3TOPL ;
    TIMSK3 = (1<<OCIE3A) ;                // enable compare match interrupt
  
    // RM: blinkies 
    DDRB = 0xC0 ;             // use blinky on ArduinoMEGA as DCD light (pin 13),
                              // and pin 12 for Timer3 interrupt heartbeat
    
    Serial.println( "Receive-only AX.25 TNC based on the" ) ;
    Serial.println( "OpenTracker1+ by Scott Miller N1VG" ) ;
    Serial.println( "ported to Arduino/AVRmega1280 (badly)" ) ;
    Serial.println( "by Robert Marshall KI4MCW" ) ;
    
    pkt_out_state = PKT_OUT_IDLE ;
    sei() ;
}


// RM: AVR/Arduino-specific code - mine
void loop()
{
    //sleep_mode() ;
    if ( pkt_out_state == PKT_OUT_READY )
    {
#ifdef DEBUG_ON
        // RM: Sanity check - a valid AX25 frame must contain the 
        //     "0x03 0xF0" end-of-header marker. As a simple check,
        //     does this packet contain that combination?
        if ( (char*)memchr( packet_out, 0xF0, packet_out_len ) - 
             (char*)memchr( packet_out, 0x03, packet_out_len ) == 1 )
        {     
            Serial.print( "GOOD packet rec'd (" ) ;
        }
        else
        {     
            // RM: On ArdMEGA, I see ==LOTS== of junk come through, even 
            //     with the radio's squelch closed. Much of it is just 
            //     garbage characters, though the Blinky Light swears it 
            //     heard an HDLC...
            Serial.print( "BAD packet rec'd (" ) ;
        }
        Serial.print( packet_out_len - 1, DEC ) ;
        Serial.println( " bytes):" ) ;
        
        // RM: output in HEX
        for ( x=0 ; x < packet_out_len ; x++ )
        {
            Serial.print( " " ) ;
            Serial.print( ( packet_out[x] & 0xFF ), HEX ) ;
        }
        Serial.println( "" ) ;
#else
        // RM: output data in KISS format
        for ( x=0 ; x < packet_out_len ; x++ )
        {
            kiss_tx_byte( packet_out[x] ) ;
        }
        kiss_tx_done() ;
#endif
        // RM: Okay, all done. Clean up before we leave.
        packet_out_len = 0 ;
        pkt_out_state = PKT_OUT_IDLE ;
        PORTB &= ~0x80 ;         // RM: DCD light off
    }

    else
    {
        // RM: No packet to send. Pause and check again.
        delay(100) ;
    }    
    
} // RM: end of "loop"


// RM: ATmega1280-specific Timer3 interrupt routine, called 8x 
//     per signalling bit (thus 9600 Hz).
//
//     This is the meat of the program, stolen shamelessly from
//     Scott's OpenTracker1+ Freepository project. As you can 
//     see from the comments, I did make a couple of changes
//     both for hardware differences and for debugging.


ISR(TIMER3_COMPA_vect)
{
    // RM: timer heartbeat on pin 12
    if ( hb12 ) { hb12 = FALSE ; PORTB &= ~0x40 ; }
    else        { hb12 = TRUE  ; PORTB |= 0x40 ;  } 
    
    // RM: just need the top 8 bits from the ADC (left-justified)
    adc = ADCH ;
    
    // RM: Originally a macro for clearing the timer interrupt
    //     on the Freescale HC08. Not required for AVR.
    //ack_demod_timer;
    
    dline = ( dline << 1 ) & 0xFF ; // shift delay line left
    xline = ( xline << 1 ) & 0xFF ; // shift lpf line left
    
    if (adc > 128 + (((tap1 << 1) + tap1) >> 2))
    {
        dline |= 1; // put bit on delay line
        if (!(dline & DELAY_MASK))
        {
            xline |= 1; // XOR of delayed sample
            lpf++;			
        }
    }

    else
    {
        if (dline & DELAY_MASK)
        {
            xline |= 1; // XOR
            lpf++;
        }
    }

    if ((xline & 0x20) && lpf) { lpf-- ; }

    // RM: tap2 not used here
    //tap2 = tap1;
    tap1 = adc - 128;

    // RM: We set the AVR ADC to free-run mode, 
    //     so dont need to start it manually
    //start_demod_sample;

    sline = (sline << 1) & 0xFF ;

    if (lpf + (xline & 1) > 3) // Low-pass filter threshold
    {
        sline |= 1;
    }

    if (++sample_num == 8)
    {
        sample_num = 0;
    }


    // Received bit handler


    if (sample_num == SAMPLE_BIT)
    {
        // This test determines state of bit for this bit time
        sample_cnt = ((sline & 7) == 6 || (sline & 7) == 3 || (sline & 7) == 7 || (sline & 7) == 5);

        if (sample_cnt != lastbit)		// NRZI: Change = 0, No change = 1
        {
            // Change (0) - check for bit stuffing
            if (bitstuff != 5)
            {
                rxbyte = rxbyte >> 1;		// Record bit if not a stuffed 0
                bitcount++;
            }

	    if (bitstuff == 6)	        	// HDLC flag character
            {
                //frame_end();
                // RM: functions from that routine copied here:
                bitcount = 0 ;
                frame_lock = 1 ;
                PORTB |= 0x80 ;      // RM: turn on DCD light
                
                // RM: Make a copy of the packet and send it to the PC
                //     from elsewhere in the code (not from the ISR).
                //     That way we can keep up with any incoming data
                //     in case another packet is coming in.
                
		if ( packet_data_len > MIN_PACKET_SIZE ) 
                {
                    packet_out_len = packet_data_len - 2 ;
                    memcpy( packet_out, packet_data, packet_out_len ) ;
                    pkt_out_state = PKT_OUT_READY ;
                }
                
                // RM: Either way, this was an HDLC, so start a new packet
                packet_data_len = 0 ;
                
            } // RM: end if bitstuff == 6

            // RM: true bit stuff (bitstuff=5), take no action (drop the "0")

            bitstuff = 0;			// Clear bitstuff counter			

	} // RM: end if sample_cnt <> lastbit

        else
        {
            // RM: sample_cnt must = lastbit, which means...
            // No change (1)
            rxbyte = 0x80 | (rxbyte >> 1);	// Record bit
            bitcount++;
            bitstuff++;				// Increment bitstuff counter
            
            if (bitstuff > 7)			// No activity for 8 bit times, no data present
            {
                //sw_dcd = 0 ;
                //frame_lock = 0 ;
                // RM: Group & report these functions below
                if ( frame_lock ) { err_state = ERR_BS_FAIL ; }
                else              { err_state = ERR_BS_WARN ; }
                err_val = bitstuff ;
            }

	} // RM: end else for "sample_cnt <> lastbit"

        lastbit = sample_cnt;			// Save last bit

	if (frame_lock && bitcount == 8)	// See if we have lock and finished a byte
        {
            //got_byte();			// Output received byte
            // RM: functions from that routine copied here
            packet_data[packet_data_len++] = ( rxbyte & 0xFF ) ;
            if (packet_data_len >= PACKET_DATA_SIZE)
            { 
                //packet_data_len = 0 ;
                //frame_lock = 0 ;
                // RM: Group & report
                err_state = ERR_OVERFLOW ;
            }    
            bitcount = 0 ;
        }

    } // RM: end if sample_num == sample_bit


    // Clock recovery

	
    if ((sline & 0xf) == 0x3 || (sline & 0xf) == 0xc)	// Detect transition
    {
        // See if transition occured during expected bit boundary
        if (sample_num > WINDOW_HIGH || sample_num < WINDOW_LOW)
        {
            // RM: good timing
            if (sw_dcd < 255) { sw_dcd++ ; }
            pll_error = 0;
        }

	else                                  // Outside window, count as a PLL error								
        {
            if (sw_dcd) { sw_dcd-- ; }
            // PLL_ERROR_LIMIT lets us tolerate a certain amount of noise in a byte
            pll_error++;
            
            //if (pll_error > PLL_ERROR_LIMIT || !frame_lock) // Too many errors, resync PLL and reset
            // RM: split now, group & report later
            if (pll_error > PLL_ERROR_LIMIT)
            {
                //sw_dcd = 0;
                // RM: Group & report
                err_state = ERR_PLL_FAIL ;
                err_val = sample_num ;
                sample_num = 1;
                // RM: Was HC08-specific macro
                //reset_demod_timer;
            }
            else if (!frame_lock) 
            { 
                sample_num = 1 ; 
                err_state = ERR_JUSTCLEAR ; 
            }
            else
            {
                err_state = ERR_PLL_WARN ;
                err_val = sample_num ;
            }    
            
	} // RM: end else for "if sample_num > WINDOW HIGH, etc"

    } // RM: end if sline, etc, for clock recovery
    
    // RM: Cleanup functions, grouped
    if ( err_state != ERR_NONE )
    {
        sw_dcd = 0 ;
        frame_lock = 0 ;
        packet_data_len = 0 ;           // RM: this seems like a good idea here
        PORTB &= ~0x80 ;                // RM: turn off DCD light
        
 #ifdef DEBUG_ON
        switch (err_state)
        {
        case ERR_BS_WARN:      //Serial.print( "b" ) ;            // hundreds of these a minute
                               break ;
                               
        case ERR_BS_FAIL:      Serial.print( "B" ) ;                       
                               //Serial.print( err_val , DEC ) ;
                               Serial.print( "!" ) ;
                               break ;
                               
        case ERR_PLL_WARN:     Serial.print( "p" ) ;
                               Serial.print( err_val , DEC ) ;
                               Serial.print( "!" ) ;
                               break ;
                               
        case ERR_PLL_FAIL:     Serial.print( "P" ) ; 
                               Serial.print( err_val , DEC ) ;
                               Serial.print( "!" ) ;
                               break ;
                       
        case ERR_OVERFLOW:     Serial.print( "=O=" ) ; 
                               break ;
        }                       
#endif

        err_state = ERR_NONE ;
        err_val = 0 ;
    }    
    sei() ;
    
} // RM: end Timer3 Interrupt


// RM: Scott's send-to-host routine, from "kiss.c", Arduin-ized
void kiss_tx_byte(unsigned char c)
{
    // RM: The Serial.print functions keep tring to print 8 hex characters
    //     for 8-bit values. Filter that now before it goofs up our output.
    c &= 0xFF ;
    
    switch (kiss_tx_state)
    {
      
    case STATE_FRAME_WAIT:                   // Start of new frame
        Serial.print( FEND , BYTE ) ;        // Frame end marker
        Serial.print( 0    , BYTE ) ;        // Data frame on port 0
        kiss_tx_state = STATE_DATA ;         // Now in data mode - fall through
        
    case STATE_DATA:
        if ( c == FEND ) 
        {
            Serial.print( FESC , BYTE ) ;
            c = TFEND ;
        }
        if ( c == FESC ) 
        {
            Serial.print( FESC , BYTE ) ;
            c = TFESC ;
        }
        Serial.print( c , BYTE ) ;
        break ;
        
    default:
        kiss_tx_state = STATE_FRAME_WAIT ;
        break ;
    }
}    


// RM: another routine from "kiss.c", Arduin-ized
void kiss_tx_done(void)
{
    Serial.print( FEND , BYTE ) ;
    Serial.println( "" ) ;
    kiss_tx_state = STATE_FRAME_WAIT ;
}    


// end of file 


