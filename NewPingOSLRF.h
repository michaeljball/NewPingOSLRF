// ---------------------------------------------------------------------------
// PingOSLRF Library - v1.0 - 04/21/2014
//
// Original AUTHOR/LICENSE:
// Created by Tim Eckel - teckel@leethost.com
// Copyright 2012 License: GNU GPL v3 http://www.gnu.org/licenses/gpl-3.0.html
//
// Modified March 2014, for the Open Source Laser Range Finder OSLRF01 
// http://lightware.co.za/shop/en/index.php?controller=attachment&id_attachment=7
// By Michael Ball  unix_guru@hotmail.com 


// LINKS:
// Project home: http://code.google.com/p/arduino-new-ping/
// Blog: http://arduino.cc/forum/index.php/topic,106043.0.html
//
// DISCLAIMER:
// This software is furnished "as is", without technical support, and with no 
// warranty, express or implied, as to its usefulness for any purpose.
//
//
// CONSTRUCTOR:
//   PingOSLRF sonar(trigger_pin, echo_pin [, max_cm_distance])
//     trigger_pin & echo_pin - Arduino pins connected to sensor trigger and echo.
//       NOTE: To use the same Arduino pin for trigger and echo, specify the same pin for both values.
//     max_cm_distance - [Optional] Maximum distance you wish to sense. Default=500cm.
//
// SYNTAX:
//   sonar.ping() - Send a ping and get the echo time (in microseconds) as a result. 
//   sonar.ping_in() - Send a ping and get the distance in whole inches.
//   sonar.ping_cm() - Send a ping and get the distance in whole centimeters.
//   sonar.ping_median(iterations) - Do multiple pings (default=5), discard out of range pings and return median in microseconds. 
//   sonar.convert_in(echoTime) - Convert echoTime from microseconds to inches (rounds to nearest inch).
//   sonar.convert_cm(echoTime) - Convert echoTime from microseconds to centimeters (rounds to nearest cm).
//   sonar.ping_timer(function) - Send a ping and call function to test if ping is complete.
//   sonar.check_timer() - Check if ping has returned within the set distance limit.
//   NewPing::timer_us(frequency, function) - Call function every frequency microseconds.
//   NewPing::timer_ms(frequency, function) - Call function every frequency milliseconds.
//   NewPing::timer_stop() - Stop the timer.
//

#ifndef NewPing_h
#define NewPing_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include <Arduino.h>
#else
	#include <WProgram.h>
	#include <pins_arduino.h>
#endif

#include <avr/io.h>
#include <avr/interrupt.h>

// Shoudln't need to changed these values unless you have a specific need to do so.
#define MAX_SENSOR_DISTANCE 1000 // Maximum sensor distance can be as high as 500cm, no reason to wait for ping longer than sound takes to travel this distance and back.

//********************************** These two need to be worked on.  with a sliding timebase these do not work  *********//
#define US_ROUNDTRIP_IN 722     // Microseconds (uS) it takes sound to travel round-trip 1 inch (2 inches total), uses integer to save compiled code space.
#define US_ROUNDTRIP_CM 1833      // Microseconds (uS) it takes sound to travel round-trip 1cm (2cm total), uses integer to save compiled code space.


// Probably shoudln't change these values unless you really know what you're doing.
#define NO_ECHO 0               // Value returned if there's no ping echo within the specified MAX_SENSOR_DISTANCE or max_cm_distance.
#define MAX_SENSOR_DELAY 25000  // Maximum uS it takes for sensor to start the ping (SRF06 is the highest measured, just under 18ms).
#define ECHO_TIMER_FREQ 24      // Frequency to check for a ping echo (every 24uS is about 0.4cm accuracy).
#define PING_MEDIAN_DELAY 29    // Millisecond delay between pings in the ping_median method.

#define ZERO_THRESHOLD  50	// Analog value to accept as Zero rise.
#define RETURN_THRESHOLD  80	// Analog value to accept as Return rise.




#define FASTADC 1				// Allows ADC Prescalar of 16 or 500khz on 16Mhz Arduino


// defines for setting and clearing register bits
#ifndef bit_get
#define bit_get(p,m) ((p) & (m))
#endif
#ifndef bit_set
#define bit_set(p,m) ((p) |= (m))
#endif
#ifndef bit_clear
#define bit_clear(p,m) ((p) &= ~(m))
#endif
#ifndef bit_flip
#define bit_flip(p,m) ((p) ^= (m))
#endif
#ifndef bit_write
#define bit_write(c,p,m) (c ? bit_set(p,m) : bit_clear(p,m))
#endif
#ifndef BIT
#define BIT(x) (0x01 << (x))
#endif
#ifndef LONGBIT
#define LONGBIT(x) ((unsigned long)0x00000001 << (x)) 
#endif


class NewPing {
	public:
		NewPing(uint8_t sync_pin, uint8_t zero_pin,  uint8_t echo_pin, int max_cm_distance = MAX_SENSOR_DISTANCE);
		unsigned long ping();
		void begin();
		unsigned int ping_in();
		unsigned int ping_cm();
		unsigned long sync_width();
		unsigned long syncWidth;
		unsigned int zero_rise();
		unsigned int zeroRise;
		unsigned int zeroThresh;
		unsigned int echoThresh;
		unsigned int echoValue;
		unsigned long echoWidth;
		unsigned int ping_median(uint8_t it = 5);

		void ping_timer(void (*userFunc)(void));
		boolean check_timer();
		unsigned long ping_result;
		static void timer_us(unsigned int frequency, void (*userFunc)(void));
		static void timer_ms(unsigned long frequency, void (*userFunc)(void));
		static void timer_stop();
	private:
		boolean ping_sync();
		boolean ping_wait_timer();
		uint8_t _syncBit;
		uint8_t _zeroBit;
		uint8_t _echoBit;
		uint8_t _zeroPin;
		uint8_t _echoPin;
		uint8_t _syncPin;
		volatile uint8_t *_syncInput;
		volatile uint8_t *_echoInput;
		volatile uint8_t *_zeroInput;
		unsigned int _maxEchoTime;
		unsigned long _max_time;
		volatile long _syncRise;
		volatile long _syncFall;
		static void timer_setup();
		static void timer_ms_cntdwn();
};


#endif
