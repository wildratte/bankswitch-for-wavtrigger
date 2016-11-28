/*
 * main.c
 *
 * Bank-switch-Extension for Wavtrigger
 *
 * Created: 24.10.2016 22:47:15
 * Author : JH
 *
 * 27.11.2016
 * Lots of feature improvements like volume setting, UI clarification etc.
 *
 */ 

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>

//
// port mapping for 7-Segment display
//
#define _7SEGMENT_DDR     DDRB
#define _7SEGMENT_OPORT   PORTB
#define _7SEGMENT_SEG_A   (1<<PB2)
#define _7SEGMENT_SEG_B   (1<<PB3)
#define _7SEGMENT_SEG_C   (1<<PB5)
#define _7SEGMENT_SEG_D   (1<<PB6)
#define _7SEGMENT_SEG_E   (1<<PB7)
#define _7SEGMENT_SEG_F   (1<<PB0)
#define _7SEGMENT_SEG_G   (1<<PB1)
#define _7SEGMENT_SEG_DP  (1<<PB4)
#define _7SEGMENT_BITS    (_7SEGMENT_SEG_A + _7SEGMENT_SEG_B + _7SEGMENT_SEG_C + _7SEGMENT_SEG_D + _7SEGMENT_SEG_E + _7SEGMENT_SEG_F + _7SEGMENT_SEG_G + _7SEGMENT_SEG_DP)
//
#define DUMMY_LED_DDR     DDRD
#define DUMMY_LED_OPORT   PORTD
#define DUMMY_LED_PIN     (1<<PD6)

//
// port mapping for switches 1..6, BS
// ports PD0, PA1, PA0, PD2, PD3, PD4, PD5 (pins 2, 4, 5, 6, 7, 8, 9)
//
#define TAST_TRIG_1_2_DDR     DDRA
#define TAST_TRIG_1_2_OPORT   PORTA
#define TAST_TRIG_1_2_IPORT   PINA
#define TAST_TRIG_1           (1<<PA1)
#define TAST_TRIG_2           (1<<PA0)
#define TAST_TRIG_1_2_BITS    (TAST_TRIG_1 + TAST_TRIG_2)
#define TAST_TRIG_1_2_PCINT_vect   PCINT1_vect
#define TAST_TRIG_1_2_PCINT_maskreg  PCMSK1
#define TAST_TRIG_1_2_PCINT_mask  ((1<<PCINT9) + (1<<PCINT8))

#define TAST_BS_TRIG_3_6_DDR     DDRD
#define TAST_BS_TRIG_3_6_OPORT   PORTD
#define TAST_BS_TRIG_3_6_IPORT   PIND
#define TAST_BS           (1<<PD0)
#define TAST_TRIG_3       (1<<PD2)
#define TAST_TRIG_4       (1<<PD3)
#define TAST_TRIG_5       (1<<PD4)
#define TAST_TRIG_6       (1<<PD5)
#define TAST_BS_TRIG_3_6_BITS    (TAST_BS + TAST_TRIG_3 + TAST_TRIG_4 + TAST_TRIG_5 + TAST_TRIG_6)
#define TAST_BS_TRIG_3_6_PCINT_vect   PCINT2_vect
#define TAST_BS_TRIG_3_6_PCINT_maskreg  PCMSK2
#define TAST_BS_TRIG_3_6_PCINT_mask  ((1<<PCINT11) + (1<<PCINT13) + (1<<PCINT14) + (1<<PCINT15) + (1<<PCINT16))

// control variables
volatile unsigned char bank = 1; // selected bank 1..6 - display '1'..'6'
volatile unsigned char bs_mode = 0; // 0 - normal trigger operation, 10 - bank select ( display blinking 'b'), 1..6: trigger on selected bank for volume adjust (display track volume), 11 - wait for trigger for volume adjust (display blinking 'u')
volatile unsigned char trigger = 0; // 1..6: selected trigger

#define TRIGGER_VAL_NEW_BS_MODE   250

// display variables
volatile unsigned char display_blink = 0; // start blinking, 1 - blink for short time, 2 - blink forever, 3 - stop blinking
volatile unsigned char display_char_idx_normal = 0; // what to display in normal mode (not blinking), 0x80 - flag for decimal point
volatile unsigned char display_char_idx_blink = 0; // what to display in blinking mode, 0x80 flag for decimal point

// 'internal' variables needed in interrupts
volatile unsigned char level_hi_time[7] = {0,0,0,0,0,0,0};
volatile unsigned short level_lo_time_bs = 0;

// 1:4 time multiplexed 7 segment display with 2 segments (or 1 segment + 1 dummy load) set at a time
// if (xx[x][0] == 0) if (dp) activate dummy_led and decimal point in cycle 0
// else if (dp) activate decimal point in cycle 0
// else activate dummy_led in cycle 0
const unsigned char _7segment_field[12][4] PROGMEM = // [4] 4 double segment time slices!
{
	// [0] segments for '0'
	{0, _7SEGMENT_SEG_A + _7SEGMENT_SEG_B, _7SEGMENT_SEG_C + _7SEGMENT_SEG_D, _7SEGMENT_SEG_E + _7SEGMENT_SEG_F},
	// [1] segments for '1'
	{0, _7SEGMENT_SEG_B + _7SEGMENT_SEG_C, 0, 0},
	// [2] segments for '2'
	{_7SEGMENT_SEG_A, _7SEGMENT_SEG_B + _7SEGMENT_SEG_D, _7SEGMENT_SEG_E + _7SEGMENT_SEG_G, 0}, // dummy-led in [0]
	// [3] segments for '3'
	{_7SEGMENT_SEG_A, _7SEGMENT_SEG_B + _7SEGMENT_SEG_C, _7SEGMENT_SEG_D + _7SEGMENT_SEG_G, 0}, // dummy-led in [0]
	// [4] segments for '4'
	{0, _7SEGMENT_SEG_B + _7SEGMENT_SEG_C, _7SEGMENT_SEG_F + _7SEGMENT_SEG_G, 0},
	// [5] segments for '5'
	{_7SEGMENT_SEG_A, _7SEGMENT_SEG_C + _7SEGMENT_SEG_D, _7SEGMENT_SEG_F + _7SEGMENT_SEG_G, 0}, // dummy-led in [0]
	// [6] segments for '6'
	{0, _7SEGMENT_SEG_A + _7SEGMENT_SEG_C, _7SEGMENT_SEG_D + _7SEGMENT_SEG_E, _7SEGMENT_SEG_F + _7SEGMENT_SEG_G},
	// [7] segments for '7'
	{_7SEGMENT_SEG_A, _7SEGMENT_SEG_B + _7SEGMENT_SEG_C, 0, 0}, // dummy-led in [0]
	// [8] segments for '8'
	{_7SEGMENT_SEG_A, _7SEGMENT_SEG_B + _7SEGMENT_SEG_C, _7SEGMENT_SEG_D + _7SEGMENT_SEG_E, _7SEGMENT_SEG_F + _7SEGMENT_SEG_G}, // dummy-ledin [0]
	// [9] segments for '9'
	{0, _7SEGMENT_SEG_A + _7SEGMENT_SEG_B, _7SEGMENT_SEG_C + _7SEGMENT_SEG_D, _7SEGMENT_SEG_F + _7SEGMENT_SEG_G},
	// [10] segments for 'b'
	{_7SEGMENT_SEG_C, _7SEGMENT_SEG_D + _7SEGMENT_SEG_E, _7SEGMENT_SEG_F + _7SEGMENT_SEG_G, 0}, // dummy-led in [0]
	// [11] segments for 'u'
	{_7SEGMENT_SEG_C, _7SEGMENT_SEG_D + _7SEGMENT_SEG_E, _7SEGMENT_SEG_F + _7SEGMENT_SEG_G, 0}, // dummy-led in [0]
};

#define _7SEGMENT_IDX_B  10
#define _7SEGMENT_IDX_U  11

#define TIMER_RES_MS 3
#define TASTER_STATLVL_MS        30
#define TASTER_LOLVL_LONGPRES_MS 4000

#define DISPLAY_BLINK_INTERVAL_MS    250 // length of blink interval = on-cycle = off-cycle
#define DISPLAY_BLINK_INTERVAL_COUNT  5 // number of blink intervals when blinking for a short time, max 254 -> ca. 62 seconds max. blink time


// timer interrupt
// every TIMER_RES_MS ms
ISR(TIMER0_COMPA_vect)
{
	static unsigned char display_blink_interval_count = 0;
	static unsigned char display_blink_interval_toggle = 0;
	static unsigned char display_blink_interval_toggle_count = 0;
	//
	unsigned char tmp = 0;
	//
	unsigned char blank = 0;
	unsigned char dp = 0;

	// blinking function
	// start blinking
	if (display_blink == 1 || display_blink == 2) // init
	{
		display_blink_interval_count = DISPLAY_BLINK_INTERVAL_MS/TIMER_RES_MS;
		display_blink_interval_toggle = 1; // start with on-cycle
		display_blink_interval_toggle_count = 1;
		if (display_blink == 1) display_blink = 0;
		else  display_blink = 250;
	}
	else if (display_blink == 3) // stop blinking immediately
	{
		display_blink = 0;
		display_blink_interval_count = 0;
		display_blink_interval_toggle_count = DISPLAY_BLINK_INTERVAL_COUNT;
	}
	// blinking continues
	if (display_blink_interval_toggle_count)
	{
		if (display_blink_interval_count == 0)
		{
			display_blink_interval_count = DISPLAY_BLINK_INTERVAL_MS/TIMER_RES_MS;
			//timer_show_triggers_toggle = 1 - timer_show_triggers_toggle;
			display_blink_interval_toggle ^= 1; // xor also toggles, but has no advantage over the above
			display_blink_interval_toggle_count++;
			// blinking interval finished
			if (display_blink_interval_toggle_count > DISPLAY_BLINK_INTERVAL_COUNT)
			{
				if (display_blink == 250) display_blink_interval_toggle_count = 1; // blink infinite 
				else display_blink_interval_toggle_count = 0; // stop it
			}
		}

		display_blink_interval_count--;

		if (display_blink_interval_toggle_count && display_blink_interval_toggle == 0)
		{
			blank = 1;
		}
	}
		
	if (display_blink_interval_toggle_count) 
	{
		tmp = display_char_idx_blink & 0x7f;
		dp = display_char_idx_blink & 0x80;
	}
	else
	{
		tmp = display_char_idx_normal & 0x7f;
		dp = display_char_idx_normal & 0x80;
	}

	if (blank)
	{
		_7SEGMENT_OPORT = 0;
		DUMMY_LED_OPORT &= ~DUMMY_LED_PIN;
		_7SEGMENT_OPORT &= ~_7SEGMENT_SEG_DP;
	}
	// 1:4 time multiplexed 7 segment display with 2 segments (or 1 segment + 1 dummy load) set at a time
	// if (xx[x][0] == 0) if (dp) activate dummy_led and decimal point in cycle 0
	// else if (dp) activate decimal point in cycle 0
	// else activate dummy_led in cycle 0
	else
	{
		static unsigned char count = 0;

		tmp = pgm_read_byte(&_7segment_field[0][(tmp << 2) + count]); // 4 byte field length / character
		// tmp now segment port bits of time slice!
		// needed below again!
		_7SEGMENT_OPORT = tmp;

		if (count == 0)
		{
			if (tmp == 0)
			{
				if (dp) {DUMMY_LED_OPORT |= DUMMY_LED_PIN; _7SEGMENT_OPORT |= _7SEGMENT_SEG_DP;}
			}
			else if (dp) _7SEGMENT_OPORT |= _7SEGMENT_SEG_DP;
			else DUMMY_LED_OPORT |= DUMMY_LED_PIN;

			count = 3;
		}
		else
		{
			DUMMY_LED_OPORT &= ~DUMMY_LED_PIN;
			_7SEGMENT_OPORT &= ~_7SEGMENT_SEG_DP;
		
			count--;
		}
	}

	// switch levels
	// count high level timer intervals
	// falling edge detection in pin change interrupt!
	// switches 1 + 2 on port A, Taster BS + 3..6 on port B!
	tmp = TAST_TRIG_1_2_IPORT; // 1x read
	//  switch 1
	if (tmp & TAST_TRIG_1 && level_hi_time[1] < 250) level_hi_time[1]++; // high
	//  switch 2
	if (tmp & TAST_TRIG_2 && level_hi_time[2] < 250) level_hi_time[2]++; // high

	tmp = TAST_BS_TRIG_3_6_IPORT; // 1x read
	
	//  switch BS
	if (tmp & TAST_BS) // high
	{
		level_lo_time_bs = 0;
		if (level_hi_time[0] < 250) level_hi_time[0]++;
	}
	else // low
	{
		if (level_lo_time_bs < TASTER_LOLVL_LONGPRES_MS/TIMER_RES_MS) level_lo_time_bs++;
		else {bs_mode = 11; trigger = TRIGGER_VAL_NEW_BS_MODE;} // new bs_mode -> special trigger
	}
	
	//  switch 3
	if (tmp & TAST_TRIG_3 && level_hi_time[3] < 250) level_hi_time[3]++; // high
	//  switch 4
	if (tmp & TAST_TRIG_4 && level_hi_time[4] < 250) level_hi_time[4]++; // high
	//  switch 5
	if (tmp & TAST_TRIG_5 && level_hi_time[5] < 250) level_hi_time[5]++; // high
	//  switch 6
	if (tmp & TAST_TRIG_6 && level_hi_time[6] < 250) level_hi_time[6]++; // high
}


// trigger switches 1..2 interrupt handler
ISR(TAST_TRIG_1_2_PCINT_vect)
{
	unsigned char tmp = TAST_TRIG_1_2_IPORT;
	
	// if the last level is hi (>=TASTER_STATLVL_MS) -> falling edge
	// level_hi_time[x] = 0 prevents new triggering for TASTER_STATLVL_MS ms
	if (!(tmp & TAST_TRIG_1) && level_hi_time[1] >= TASTER_STATLVL_MS/TIMER_RES_MS) {trigger = 1; level_hi_time[1] = 0;}
	else if (!(tmp & TAST_TRIG_2) && level_hi_time[2] >= TASTER_STATLVL_MS/TIMER_RES_MS) {trigger = 2; level_hi_time[2] = 0;}
}

// BS switch + trigger switches 3..6 interrupt handler
ISR(TAST_BS_TRIG_3_6_PCINT_vect)
{
	unsigned char tmp = TAST_BS_TRIG_3_6_IPORT;
	
	// if the last level is hi (>=TASTER_STATLVL_MS) -> falling edge
	// level_hi_time[x] = 0 prevents new triggering for TASTER_STATLVL_MS ms
	if (!(tmp & TAST_BS) && level_hi_time[0] >= TASTER_STATLVL_MS/TIMER_RES_MS) {if (bs_mode) bs_mode = 0; else bs_mode = 10; level_hi_time[0] = 0; trigger = TRIGGER_VAL_NEW_BS_MODE;} // new bs_mode -> special trigger
	else if (!(tmp & TAST_TRIG_3) && level_hi_time[3] >= TASTER_STATLVL_MS/TIMER_RES_MS) {trigger = 3; level_hi_time[3] = 0;}
	else if (!(tmp & TAST_TRIG_4) && level_hi_time[4] >= TASTER_STATLVL_MS/TIMER_RES_MS) {trigger = 4; level_hi_time[4] = 0;}
	else if (!(tmp & TAST_TRIG_5) && level_hi_time[5] >= TASTER_STATLVL_MS/TIMER_RES_MS) {trigger = 5; level_hi_time[5] = 0;}
	else if (!(tmp & TAST_TRIG_6) && level_hi_time[6] >= TASTER_STATLVL_MS/TIMER_RES_MS) {trigger = 6; level_hi_time[6] = 0;}
}


/*
Wavtrigger protocol:

16-bit data values such as track number and volume are sent "little-endian", that is with the LSB first and the MSB second.

Message format: ( SOM1, SOM2, length, message code, data * n , EOM ),  where SOM1=0xf0,  SOM2=0xaa,  EOM=0x55

CONTROL_TRACK
Message Code = 0x03, Length = 8
Data = Track Control Code (1 byte), Track Number (2 bytes)
Response = none
Comments: Sends a Track Control Code to a specific track number

Example: 0xf0, 0xaa, 0x08, 0x03, 0x01, 0x0a, 0x00, 0x55

Track Control Codes:

PLAY_SOLO = 0x00: Play track without polyphony, stops all other tracks
PLAY_POLY = 0x01: Play track polyphonically
PAUSE = 0x02: Pause track
RESUME = 0x03: Resume track
STOP = 0x04: Stop track
LOOP_ON = 0x05: Set the track loop flag
LOOP_OFF = 0x06: Clear the track loop flag
LOAD = 0x07: Load and pause track


TRACK_VOLUME
(Note 1)

Message Code = 0x08, Length = 9
Data = Track Number (2 bytes), Volume (2 bytes, signed int, -70dB to +10dB)
Response = none
Comments: Updates the volume of a track with the specified gain in dB
Example: 0xf0, 0xaa, 0x09, 0x08, 0x01, 0x00, 0x00, 0x00, 0x55
*/

unsigned char wt_prot_play_solo[8] = {0xf0, 0xaa, 0x08, 0x03, 0x00, 0x00, 0x00, 0x55}; // track number in [5] + [6]
unsigned char wt_prot_track_volume[9] = {0xf0, 0xaa, 0x09, 0x08, 0x00, 0x00, 0x00, 0x00, 0x55}; // track number in [4] + [5], track volume in [6] + [7] (16bit signed)

// calculate track number send to Wavtrigger from bank and trigger
uint16_t calc_prot_tracknumber(unsigned char bank, unsigned char trigger)
{
	uint16_t track = 0;
	uint16_t bs;

	//track = (uint16_t)bank * 100 + (uint16_t)trigger;  !! bank * 10 + trigger would also work, then track number only 11..66 (1 Byte)
	bs = (uint16_t)(bank << 2); // bank * 4
	track += bs;
	bs <<= 3; // bank * 4 * 8 (32)
	track += bs;
	bs <<= 1; // bank * 4 * 8 * 2 (64)
	track += bs;
	track += (uint16_t)trigger;

	return track;
}

void send_prot(unsigned char * prot, unsigned char prot_len)
{
	unsigned char i;

	for (i = 0; i < sizeof(wt_prot_play_solo); i++)
	{
		// Wait for empty transmit buffer
		while ( !( UCSRA & (1<<UDRE)) );
		// Put data into buffer, sends the data
		UDR = wt_prot_play_solo[i];
	}
}

// send play track monophonic
void play_track(unsigned char bank, unsigned char trigger)
{
	uint16_t track = 0;
	unsigned char * track_ucp = (unsigned char *)&track;

	track = calc_prot_tracknumber(bank, trigger);

	wt_prot_play_solo[5] = track_ucp[0];
	wt_prot_play_solo[6] = track_ucp[1];

	// send protocol
	send_prot(wt_prot_play_solo,sizeof(wt_prot_play_solo));
}

//
// track volume
// stored in EEPROM with offset of 80 to detect uninitialized EEPROM (would result in track volume of -1)
//
signed char track_volume[6][6] =
{
	{0,0,0,0,0,0},
	{0,0,0,0,0,0},
	{0,0,0,0,0,0},
	{0,0,0,0,0,0},
	{0,0,0,0,0,0},
	{0,0,0,0,0,0},
};

// sends the volume of track [bank][trigger] to Wavtrigger board
void send_volume(unsigned char bank, unsigned char trigger)
{
	uint16_t track;
	unsigned char * track_ucp = (unsigned char *)&track;
	//
	signed char volume = track_volume[bank-1][trigger-1];

	track = calc_prot_tracknumber(bank, trigger);

	wt_prot_track_volume[4] = track_ucp[0];
	wt_prot_track_volume[5] = track_ucp[1];

	wt_prot_track_volume[6] = *((unsigned char *)&volume);
	wt_prot_track_volume[7] = 0;
	if (volume < 0 ) wt_prot_track_volume[7] = 0xff;

	// send protocol
	send_prot(wt_prot_track_volume,sizeof(wt_prot_track_volume));
}

// saves volume of track [bank][trigger] to EEPROM
void save_volume(unsigned char bank, unsigned char trigger)
{
	signed char volume = track_volume[bank-1][trigger-1];

	cli();
	// Wait for completion of previous write
	while(EECR & (1<<EEPE));
	// Set Programming mode
	EECR = (0<<EEPM1)|(0<<EEPM0);
	// Set up address and data registers
	EEAR =  bank * 10 + trigger;
	EEDR = (unsigned char)(volume + 80);
	// Write logical one to EEMPE
	EECR |= (1<<EEMPE);
	// Start EEPROM write by setting EEPE
	EECR |= (1<<EEPE);
	//
	sei();
	return;
}

// loads volume of track [bank][trigger] from EEPROM
void load_volume(unsigned char bank, unsigned char trigger)
{
	unsigned char result;

	// Wait for completion of previous write
	while (EECR & (1<<EEPE)) ;
	// Set up address register
	EEAR = bank * 10 + trigger;
	// Start EEPROM read by writing EERE
	EECR |= (1<<EERE);
	// Return data from data register
	result = EEDR;
	if (result >= 80-9 && result <= 80+9) // valid !!! (-9 .. +9)
	{
		track_volume[bank-1][trigger-1] = (signed char)result - 80;
	}
}

// display volume of track [bank][trigger]
void show_volume(unsigned char bank, unsigned char trigger)
{
	signed char volume = track_volume[bank-1][trigger-1];

	if (volume >= 0) display_char_idx_normal = (unsigned char)volume;
	else display_char_idx_normal = 0x80 + (unsigned char)(-volume); // with decimal point
}

// what to say
int main(void)
{
	//
	// read calibration for RC oscillator from EEPROM and write it to OSCCAL?
	// not needed
	/*
	// Wait for completion of previous write
	while (EECR & (1<<EEPE)) ;
	// Set up address register
	EEAR = 0;
	// Start EEPROM read by writing EERE
	EECR |= (1<<EERE);
	// Return data from data register
	OSCCAL = EEDR;
	*/

	// 7-Segment port pins are outputs (DDRB-Bit = 1)
	_7SEGMENT_DDR |= _7SEGMENT_BITS;
	// Dummy-LED port pin is output
	DUMMY_LED_DDR |= DUMMY_LED_PIN;

	// switches 1..2 port pins are input, internal pull-ups
	TAST_TRIG_1_2_OPORT |= TAST_TRIG_1_2_BITS;
	TAST_TRIG_1_2_DDR &= ~ TAST_TRIG_1_2_BITS;
	// switches BS + 3..6 port pins are input, internal pull-ups
	TAST_BS_TRIG_3_6_OPORT |= TAST_BS_TRIG_3_6_BITS;
	TAST_BS_TRIG_3_6_DDR &= ~ TAST_BS_TRIG_3_6_BITS;

	// pin change interrupt mask for BS and switches
	TAST_TRIG_1_2_PCINT_maskreg |= TAST_TRIG_1_2_PCINT_mask;
	TAST_BS_TRIG_3_6_PCINT_maskreg |= TAST_BS_TRIG_3_6_PCINT_mask;
	// global interrupt mask for external interrupts
	GIMSK = (1 << PCIE1) + (1 << PCIE2); // PCIE1, PCIE2

	// timer 0:
	// triggers every x ms, 8MHz (0.125us) * 256 = 32us: all n divided clocks
	// CTC-Mode: -> (WGM02:0 = 2), (CS02:0 = 4), OCR0A = n - 1
	OCR0A = TIMER_RES_MS / 0.032 - 0.5; // precomputed by compiler includes rounding +0.5
	TCCR0A = (1 << WGM01);
	TCCR0B = (1 << CS02);
	TIMSK = (1 << OCIE0A);

	// UART
	// 57600 Baud @ 8MHz, only TxD
	// U2X = 1: Baud = fosc / 8 / (UBRR+1); UBRR = fosc / 8 / baud - 1 -> UBRR = 16 (smaller error than with U2x = 0!)
	UBRRH = 0;
	UBRRL = 16; // fosc / 8 / baud - 1
	UCSRA = (1 << U2X);
	// Enable transmitter
	UCSRB = (1 << TXEN);
	// Set frame format: 8data, 1stop bit
	UCSRC = (1 << USBS) | (3 << UCSZ0);

	// debug
	/*
	track_volume[0][0] = -8;
	track_volume[5][5] = 7;
	save_volume(1,1);
	save_volume(6,6);
	load_volume(1,1);
	load_volume(6,6);
	send_volume(1,1);
	send_volume(6,6);
	show_volume(1,1);
	*/

	//
	// read 36 track volumes from EEPROM
	//
	{
		unsigned char b, t;

		for (b = 1; b <= 6; b++)
		{
			for (t = 1; t <= 6; t++)
			{
				load_volume(b, t);
			}
		}
	}

	// enable interrupts
	sei();

	// initial kick
	trigger = TRIGGER_VAL_NEW_BS_MODE;

	// loop forever
	while (1) 
	{
		if (trigger) // trigger / bank switch pressed
		{
			if (trigger == TRIGGER_VAL_NEW_BS_MODE) // bank switch pressed
			{
				if (bs_mode == 10)
				{
					display_char_idx_blink = _7SEGMENT_IDX_B; display_blink = 2; // display blinking 'b'
				}
				else if (bs_mode == 11)
				{
					display_char_idx_blink = _7SEGMENT_IDX_U; display_blink = 2; // display blinking 'u'
				}
				else // == 0 deactivate mode
				{
					display_char_idx_normal = bank;
					display_blink = 3; // stop blinking
				}
				trigger = 0;
			}
			else if (bs_mode == 10) // switch bank to number of pressed trigger switch
			{
				bank = trigger;
				//
				display_blink = 3; // stop blinking
				display_char_idx_normal = bank;
				bs_mode = 0;
				//
				trigger = 0;
			}
			else if (bs_mode == 11) // select track on current bank for volume change
			{
				bs_mode = trigger;
				//
				display_blink = 3; // stop blinking
				//display track volume (decimal point when < 0)
				show_volume(bank, bs_mode);
				// 
				trigger = 0;
			}
			else if (bs_mode > 0) // change track volume: bs_mode is track number on bank
			{
				signed char change = 0;
				signed char tv = track_volume[bank-1][bs_mode-1];

				if (tv > -9 && tv < 9)
				{
					switch (bs_mode)
					{
						// volume adjust with trigger switches 5 + 6
						case 1: case 2: case 3:
							if (trigger == 5) change = -1;// Volume-
							else if (trigger == 6) change = 1;// Volume+
							break;
						// volume adjust with trigger switches 1 + 2
						case 4: case 5: case 6:
							if (trigger == 1) change = -1;// Volume-
							else if (trigger == 2) change = 1;// Volume+
							break;
					}
				}
				if (change != 0)
				{
					// save and show new track volume, don't play
					track_volume[bank-1][bs_mode-1] = tv + change;

					save_volume(bank, bs_mode);
					show_volume(bank, bs_mode);
					// play_track(bank, bs_mode);
					send_volume(bank, bs_mode); // (re)enables track volume update of playing track

					trigger = 0;
				}
				else if (trigger != bs_mode) // switch without meaning in this mode
				{
					trigger = 0;
				}
				// let through trigger for playing this track
			}
		}

		if (trigger) // trigger switch depressed, trigger WavTrigger track, show track number for short time blinking with decimal point
		{
			display_char_idx_blink = 0x80 + trigger;
			display_blink = 1;

			play_track(bank, trigger);
			
			// always send track volume after play_track to keep track volume integrity ensured
			send_volume(bank, trigger);

			trigger = 0;
		}
	}
}

