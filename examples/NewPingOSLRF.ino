// ---------------------------------------------------------------------------
// Example NewPing library sketch that does a ping about 20 times per second.
// ---------------------------------------------------------------------------

#include <NewPingOSLRF.h>

#define SYNC_PIN  2  // Arduino pin tied to Sync pin on the OSLRF.
#define ZERO_PIN     A1  // Arduino pin tied to Zero pin on the OSLRF.
#define ECHO_PIN     A3  // Arduino pin tied to Signal pin on the OSLRF.
#define MAX_DISTANCE 800 // Maximum distance we want to ping for (in centimeters). Maximum sensor distance is rated at 400-500cm.

NewPing oslrf(SYNC_PIN, ZERO_PIN,  ECHO_PIN, MAX_DISTANCE); // NewPing setup of pins and maximum distance.
unsigned int syncWidth;
unsigned int zeroRise;
unsigned long uS;
unsigned long Distance;

void setup() {
  Serial.begin(115200); // Open serial monitor at 115200 baud to see ping results.

 oslrf.begin();
  syncWidth = oslrf.syncWidth;
  zeroRise = oslrf.zeroRise;
  oslrf.zeroThresh = 40;   // Manipulate the rising edge of the laser pulse
  oslrf.echoThresh = 60;   // Manipulate the rising edge of the echo pulse
  delay(29);               // 29ms should be the shortest delay between pings.
 
}

void loop() {
  
      delay(1000);               // 29ms should be the shortest delay between pings.
      uS = oslrf.ping();
      Serial.print("ZeroRise:  ");
      Serial.print(zeroRise);
      Serial.print(" uS    Echo  ");
      Serial.print(uS); // Convert ping time to distance in cm and print result (0 = outside set distance range)
      Serial.print(" uS    Syncwidth  ");
      Serial.print(syncWidth);
      Serial.print(" uS   Distance =  ");
      Distance = ((uS - zeroRise) *1833)/syncWidth;
      Serial.print(Distance);
      Serial.print(" cm  echoValue = ");
      Serial.print(oslrf.echoValue);
      Serial.print("  echoWidth = ");
      Serial.println(oslrf.echoWidth);
      
  delay(30);               // 29ms should be the shortest delay between pings.
      
      Serial.print("From ping_cm() ");
      Serial.print(oslrf.ping_cm());
      Serial.print(" cm  ");
  delay(30);               // 29ms should be the shortest delay between pings.
      Serial.print(oslrf.ping_in());
      Serial.print(" inches      ");

  delay(30);               // 29ms should be the shortest delay between pings.
      uS = oslrf.ping_median();
      Distance = ((uS - zeroRise) *1833)/syncWidth;
      Serial.print(Distance);
      Serial.println(" median cm  ");
 
}
