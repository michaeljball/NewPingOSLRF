// ---------------------------------------------------------------------------
// Example NewPing library sketch that does a ping about 20 times per second.
//
// Modified March 2014, for the Open Source Laser Range Finder OSLRF01 
// http://lightware.co.za/shop/en/index.php?controller=attachment&id_attachment=7
// By Michael Ball  unix_guru@hotmail.com 
// ---------------------------------------------------------------------------

#include <NewPingOSLRF.h>

#define SYNC_PIN  2  // Arduino pin tied to trigger pin on the OSLRF.
#define ZERO_PIN     A1  // Arduino pin tied to echo pin on the OSLRF.
#define ECHO_PIN     A3  // Arduino pin tied to echo pin on the OSLRF.
#define MAX_DISTANCE 900 // Maximum distance we want to ping for (in centimeters). Maximum sensor distance is rated at 900cm.

NewPing oslrf(SYNC_PIN, ZERO_PIN,  ECHO_PIN, MAX_DISTANCE); // NewPing setup of pins and maximum distance.
unsigned int syncWidth;
unsigned  int zeroRise;

float Distance;
void setup() {
  Serial.begin(115200); // Open serial monitor at 115200 baud to see ping results.

 oslrf.begin();
}

void loop() {
  syncWidth = oslrf.syncWidth;
  zeroRise = oslrf.zeroRise;
  delay(50);                      // Wait 50ms between pings (about 20 pings/sec). 29ms should be the shortest delay between pings.
  unsigned int uS = oslrf.ping(); // Send ping, get ping time in microseconds (uS).
  Serial.print("Ping:  ");
  Serial.print(zeroRise);
  Serial.print(" uS       ");
  Serial.print(uS); // Convert ping time to distance in cm and print result (0 = outside set distance range)
  Serial.print(" uS       ");
  Serial.print(syncWidth);
  Serial.print(" uS      Distance =  ");
  Distance = (float(uS-zeroRise)/syncWidth)*1833;
 Serial.print(Distance);			// Using the raw values
  Serial.print(" cm  ");
 Serial.print(oslrf.ping_cm());			// Using the ping_cm() function
  Serial.print(" cm  ");
 Serial.print(oslrf.ping_in());		        // Using the ping_in() function
  Serial.println(" inches  ");
  
  
}
