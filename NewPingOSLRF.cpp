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
	_syncInput = portInputRegister(digitalPinToPort(sync_pin)); 	    // Get the input port register for the sync pin.
	_zeroInput = portInputRegister(digitalPinToPort(zero_pin));         // Get the input port register for the zero pin.
	_echoInput = portInputRegister(digitalPinToPort(echo_pin));         // Get the input port register for the echo pin.

//	_maxEchoTime = min(max_cm_distance, MAX_SENSOR_DISTANCE) * US_ROUNDTRIP_CM + (US_ROUNDTRIP_CM / 2); // Calculate the maximum distance in uS.
	_maxEchoTime = min(max_cm_distance, MAX_SENSOR_DISTANCE) * US_ROUNDTRIP_CM ; // Calculate the 	zeroThresh =  ZERO_THRESHOLD;
	echoThresh =  RETURN_THRESHOLD;

#if FASTADC
  // set ADC prescale to 16
//  bit_set(ADCSRA,ADPS2) ;
//  bit_clear(ADCSRA,ADPS1) ;
//  bit_clear(ADCSRA,ADPS0) ;

  // set ADC prescale to 8
  bit_clear(ADCSRA,ADPS2) ;
  bit_set(ADCSRA,ADPS1) ;
  bit_set(ADCSRA,ADPS0) ;
#endif

}


// ---------------------------------------------------------------------------
// Standard ping methods
// ---------------------------------------------------------------------------

unsigned long NewPing::ping() {
  unsigned long echo_rise;
  unsigned long echo_fall;
  int ev;
	echoValue = 0;					 // Reset Max value of Echo for new reading.
	if (!ping_sync()) return NO_ECHO;                     // Sync a ping, if it returns false, return NO_ECHO.
	while ((echoValue = analogRead(_echoPin)) < echoThresh)
	    if (micros() > _max_time) return NO_ECHO;       // Stop the loop and return NO_ECHO (false) 
							    // if we're beyond the set maximum distance.
	echo_rise = micros();

	while (ev <= echoValue ) {			   // Get maximum height of pulse...
		ev = echoValue;
		echoValue = analogRead(_echoPin);	    
	}
	//echo_fall = micros();
	echoWidth = micros() - echo_rise;   

 	ping_result = (echo_rise - _syncFall ) ; // Calculate ping time, 5uS of overhead.
	return (ping_result);		// ping_result is round trip... just send one way value
}


unsigned int NewPing::ping_in() {
	unsigned long echoTime = NewPing::ping();      // Calls the ping method and returns with the ping echo distance in uS.
	return ((echoTime-zeroRise)*722)/syncWidth; // Convert uS to centimeters.
}


unsigned int NewPing::ping_cm() {
	unsigned long echoTime = NewPing::ping();      // Calls the ping method and returns with the ping echo distance in uS.
	return ((echoTime-zeroRise)*1833)/syncWidth; // Convert uS to centimeters.
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

unsigned long NewPing::sync_width() {		// Calculate the width of the OSLRF01 Sync Pulse
  unsigned long sync_drop;
  unsigned long sync_rise;
	while (!(*_syncInput & _syncBit)) {}                     // Wait for Sync pin to rise so we can find a start state.
	while (*_syncInput & _syncBit) {} 			 // Wait for Sync pin to clear.
	sync_drop = micros();					 // Falling edge time.
	while (!(*_syncInput & _syncBit)) {}                       // Wait for Sync pin to rise.
	sync_rise = micros();					 // Rising edge time.

	syncWidth = (sync_rise - sync_drop) *2 ;
	_maxEchoTime = syncWidth *2; 
	return syncWidth;
}



unsigned int NewPing::zero_rise() {		    // Find the start delay of the outgoing pulse for calculations
	if (!ping_sync()) return NO_ECHO;           // Sync a ping with falling edge of Sync, if it returns false, 
	while (analogRead(_zeroPin) < zeroThresh) {}
	return (micros() - _syncFall);
}

void NewPing::begin(){
	syncWidth = NewPing::sync_width();
	zeroRise = NewPing::zero_rise();
}

// ---------------------------------------------------------------------------
// Standard ping method support functions (not called directly)
// ---------------------------------------------------------------------------

boolean NewPing::ping_sync() {		// Find falling edge of ping sync

	_max_time =  micros() + MAX_SENSOR_DELAY;                  // Set a timeout for the ping to trigger.
	while (!(*_syncInput & _syncBit)) {}                      // Wait for Sync pin to rise so we can find a start state.
	while (*_syncInput & _syncBit && micros() <= _max_time) {} // Wait for Sync pin to clear.
	if (micros() > _max_time) return false;                   // Something went wrong, abort.
	_max_time = micros() + _maxEchoTime;                      // Ping started, set the timeout.
	_syncFall = micros();
	return true;                                              // Ping started successfully.
} 




