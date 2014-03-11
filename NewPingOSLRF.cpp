// ---------------------------------------------------------------------------
// Created by Tim Eckel - teckel@leethost.com
// Copyright 2012 License: GNU GPL v3 http://www.gnu.org/licenses/gpl-3.0.html
//
// See "NewPing.h" for purpose, syntax, version history, links, and more.
//
// Modified March 2014, for the Open Source Laser Range Finder OSLRF01 
// http://lightware.co.za/shop/en/index.php?controller=attachment&id_attachment=7
// By Michael Ball  unix_guru@hotmail.com 
// ---------------------------------------------------------------------------

#include "NewPingOSLRF.h"


// ---------------------------------------------------------------------------
// NewPing constructor
// ---------------------------------------------------------------------------

NewPing::NewPing(uint8_t sync_pin, uint8_t zero_pin, uint8_t echo_pin, int max_cm_distance) {
	_syncBit = digitalPinToBitMask(sync_pin);       // Get the port register bitmask for the sync pin.
	_zeroBit = digitalPinToBitMask(zero_pin);       // Get the port register bitmask for the echo pin.
	_syncPin = sync_pin;
	_zeroPin = zero_pin;
	_echoBit = digitalPinToBitMask(echo_pin);       // Get the port register bitmask for the echo pin.
	_echoPin = echo_pin;
	_syncOutput = portOutputRegister(digitalPinToPort(sync_pin)); // Get the output port register for the sync pin.
	_zeroInput = portInputRegister(digitalPinToPort(zero_pin));         // Get the input port register for the zero pin.
	_echoInput = portInputRegister(digitalPinToPort(echo_pin));         // Get the input port register for the echo pin.

	_syncMode = (uint8_t *) portModeRegister(digitalPinToPort(sync_pin)); // Get the port mode register for the sync pin.

	_maxEchoTime = min(max_cm_distance, MAX_SENSOR_DISTANCE) * US_ROUNDTRIP_CM + (US_ROUNDTRIP_CM / 2); // Calculate the maximum distance in uS.

#if DISABLE_ONE_PIN == true
	*_syncMode |= _syncBit; // Set sync pin to output.
#endif

#if FASTADC
  // set ADC prescale to 16
  bit_set(ADCSRA,ADPS2) ;
  bit_clear(ADCSRA,ADPS1) ;
  bit_clear(ADCSRA,ADPS0) ;
#endif

}


// ---------------------------------------------------------------------------
// Standard ping methods
// ---------------------------------------------------------------------------

unsigned int NewPing::ping() {
	if (!ping_sync()) return NO_ECHO;                // Sync a ping, if it returns false, return NO_ECHO to the calling function.
	//while (*_echoInput & _echoBit)                      // Wait for the ping echo.
	while (analogRead(_echoPin) < RETURN_THRESHOLD)
		if (micros() > _max_time) return NO_ECHO;       // Stop the loop and return NO_ECHO (false) if we're beyond the set maximum distance.
	return (micros() - (_max_time - _maxEchoTime) - 5); // Calculate ping time, 5uS of overhead.
}


unsigned int NewPing::ping_in() {
	unsigned int echoTime = NewPing::ping();          // Calls the ping method and returns with the ping echo distance in uS.
	return (float(echoTime-zeroRise)/syncWidth)*722; // Convert uS to centimeters.
}


unsigned int NewPing::ping_cm() {
	unsigned int echoTime = NewPing::ping();          // Calls the ping method and returns with the ping echo distance in uS.
	return (float(echoTime-zeroRise)/syncWidth)*1833; // Convert uS to centimeters.
}


unsigned int NewPing::ping_median(uint8_t it) {
	unsigned int uS[it], last;
	uint8_t j, i = 0;
	uS[0] = NO_ECHO;
	while (i < it) {
		last = ping();           // Send ping.
		if (last == NO_ECHO) {   // Ping out of range.
			it--;                // Skip, don't include as part of median.
			last = _maxEchoTime; // Adjust "last" variable so delay is correct length.
		} else {                       // Ping in range, include as part of median.
			if (i > 0) {               // Don't start sort till second ping.
				for (j = i; j > 0 && uS[j - 1] < last; j--) // Insertion sort loop.
					uS[j] = uS[j - 1]; // Shift ping array to correct position for sort insertion.
			} else j = 0;              // First ping is starting point for sort.
			uS[j] = last;              // Add last ping to array in sorted position.
			i++;                       // Move to next ping.
		}
		if (i < it) delay(PING_MEDIAN_DELAY - (last >> 10)); // Millisecond delay between pings.
	}
	return (uS[it >> 1]); // Return the ping distance median.
}



// ---------------------------------------------------------------------------
// OSLRF support functions for Constructor.... 
// zero_rise() and sync_width are required to determine start of pulse and 
// time base.   zero_rise initiates outbound laser pulse.  then the difference 
// between that and the start of the return signal is divided by the sync_width
// multiplied by the constant 18.33m        D = ((Rt-Zt) / Sp) * 18.33M
// ---------------------------------------------------------------------------

unsigned int NewPing::sync_width() {		// Calculate the width of the OSLRF01 Sync Pulse
  unsigned int sync_drop;
  unsigned int sync_rise;
	while (*_syncOutput & _syncBit && micros() <= _max_time) {} // Wait for Sync pin to clear.
//	sync_drop = micros();
//	while (!(*_syncOutput & _syncBit))                        // Wait for Sync pin to rise.
//		if (micros() > (_max_time*2)) return 999;         // Something went wrong, abort.
//	sync_rise = micros();

//	return (sync_rise - sync_drop)*2;			  // Count period of low pulse times 2
	return pulseIn(_syncPin,HIGH)*2;
}

unsigned int NewPing::zero_rise() {
	if (!ping_sync()) return NO_ECHO;                // Sync a ping, if it returns false, return NO_ECHO to the calling function.
	while (analogRead(_zeroPin) < ZERO_THRESHOLD)
		if (micros() > _max_time) return NO_ECHO;       // Stop the loop and return NO_ECHO (false) if we're beyond the set maximum distance.
	return (micros() - (_max_time - _maxEchoTime) - 5); // Calculate ping time, 5uS of overhead.
}

void NewPing::begin(){
	zeroRise = NewPing::zero_rise();
	syncWidth = NewPing::sync_width();
}

// ---------------------------------------------------------------------------
// Standard ping method support functions (not called directly)
// ---------------------------------------------------------------------------

boolean NewPing::ping_sync() {

	_max_time =  micros() + MAX_SENSOR_DELAY;                  // Set a timeout for the ping to trigger.
	while (*_syncOutput & _syncBit && micros() <= _max_time) {} // Wait for Sync pin to clear.

// I need to replace analogRead() here, with a more efficient continuous running ADC
	while(analogRead(_zeroPin) < ZERO_THRESHOLD)
		if (micros() > _max_time) return false;                // Something went wrong, abort.

	_max_time = micros() + _maxEchoTime; // Ping started, set the timeout.
	return true;                         // Ping started successfully.
}


// ---------------------------------------------------------------------------
// Timer interrupt ping methods (won't work with ATmega8 and ATmega128)
// ---------------------------------------------------------------------------

void NewPing::ping_timer(void (*userFunc)(void)) {
	if (!ping_sync()) return;         // Trigger a ping, if it returns false, return without starting the echo timer.
	timer_us(ECHO_TIMER_FREQ, userFunc); // Set ping echo timer check every ECHO_TIMER_FREQ uS.
}

 
boolean NewPing::check_timer() {
	if (micros() > _max_time) { // Outside the timeout limit.
		timer_stop();           // Disable timer interrupt
		return false;           // Cancel ping timer.
	}

// I need to replace analogRead() here, with a more efficient continuous running ADC
	if (analogRead(_echoPin) > RETURN_THRESHOLD) { // Ping echo received.
		timer_stop();                // Disable timer interrupt
		ping_result = (micros() - (_max_time - _maxEchoTime) - 13); // Calculate ping time, 13uS of overhead.
		return true;                 // Return ping echo true.
	}

	return false; // Return false because there's no ping echo yet.
}


// ---------------------------------------------------------------------------
// Timer2/Timer4 interrupt methods (can be used for non-ultrasonic needs)
// ---------------------------------------------------------------------------

// Variables used for timer functions
void (*intFunc)();
void (*intFunc2)();
unsigned long _ms_cnt_reset;
volatile unsigned long _ms_cnt;


void NewPing::timer_us(unsigned int frequency, void (*userFunc)(void)) {
	timer_setup();      // Configure the timer interrupt.
	intFunc = userFunc; // User's function to call when there's a timer event.

#if defined (__AVR_ATmega32U4__) // Use Timer4 for ATmega32U4 (Teensy/Leonardo).
	OCR4C = min((frequency>>2) - 1, 255); // Every count is 4uS, so divide by 4 (bitwise shift right 2) subtract one, then make sure we don't go over 255 limit.
	TIMSK4 = (1<<TOIE4);                  // Enable Timer4 interrupt.
#else
	OCR2A = min((frequency>>2) - 1, 255); // Every count is 4uS, so divide by 4 (bitwise shift right 2) subtract one, then make sure we don't go over 255 limit.
	TIMSK2 |= (1<<OCIE2A);                // Enable Timer2 interrupt.
#endif
}


void NewPing::timer_ms(unsigned long frequency, void (*userFunc)(void)) {
	timer_setup();                       // Configure the timer interrupt.
	intFunc = NewPing::timer_ms_cntdwn;  // Timer events are sent here once every ms till user's frequency is reached.
	intFunc2 = userFunc;                 // User's function to call when user's frequency is reached.
	_ms_cnt = _ms_cnt_reset = frequency; // Current ms counter and reset value.

#if defined (__AVR_ATmega32U4__) // Use Timer4 for ATmega32U4 (Teensy/Leonardo).
	OCR4C = 249;         // Every count is 4uS, so 1ms = 250 counts - 1.
	TIMSK4 = (1<<TOIE4); // Enable Timer4 interrupt.
#else
	OCR2A = 249;           // Every count is 4uS, so 1ms = 250 counts - 1.
	TIMSK2 |= (1<<OCIE2A); // Enable Timer2 interrupt.
#endif
}


void NewPing::timer_stop() { // Disable timer interrupt.
#if defined (__AVR_ATmega32U4__) // Use Timer4 for ATmega32U4 (Teensy/Leonardo).
	TIMSK4 = 0;
#else
	TIMSK2 &= ~(1<<OCIE2A);
#endif
}


// ---------------------------------------------------------------------------
// Timer2/Timer4 interrupt method support functions (not called directly)
// ---------------------------------------------------------------------------

void NewPing::timer_setup() {
#if defined (__AVR_ATmega32U4__) // Use Timer4 for ATmega32U4 (Teensy/Leonardo).
	timer_stop(); // Disable Timer4 interrupt.
	TCCR4A = TCCR4C = TCCR4D = TCCR4E = 0;
	TCCR4B = (1<<CS42) | (1<<CS41) | (1<<CS40) | (1<<PSR4); // Set Timer4 prescaler to 64 (4uS/count, 4uS-1020uS range).
	TIFR4 = (1<<TOV4);
	TCNT4 = 0;    // Reset Timer4 counter.
#else
	timer_stop();           // Disable Timer2 interrupt.
	ASSR &= ~(1<<AS2);      // Set clock, not pin.
	TCCR2A = (1<<WGM21);    // Set Timer2 to CTC mode.
	TCCR2B = (1<<CS22);     // Set Timer2 prescaler to 64 (4uS/count, 4uS-1020uS range).
	TCNT2 = 0;              // Reset Timer2 counter.
#endif
}


void NewPing::timer_ms_cntdwn() {
	if (!_ms_cnt--) {            // Count down till we reach zero.
		intFunc2();              // Scheduled time reached, run the main timer event function.
		_ms_cnt = _ms_cnt_reset; // Reset the ms timer.
	}
}


#if defined (__AVR_ATmega32U4__) // Use Timer4 for ATmega32U4 (Teensy/Leonardo).
ISR(TIMER4_OVF_vect) {
#else
ISR(TIMER2_COMPA_vect) {
#endif
	if(intFunc) intFunc(); // If wrapped function is set, call it.
}


// ---------------------------------------------------------------------------
// Conversion methods (rounds result to nearest inch or cm).
// ---------------------------------------------------------------------------

unsigned int NewPing::convert_in(unsigned int echoTime) {
	return NewPingConvert(echoTime, US_ROUNDTRIP_IN); // Convert uS to inches.
}


unsigned int NewPing::convert_cm(unsigned int echoTime) {
	return NewPingConvert(echoTime, US_ROUNDTRIP_CM); // Convert uS to centimeters.
}
