/**
 * https://forum.arduino.cc/index.php?topic=8774.0
 * CalculateUptimeSeconds()
 * 
 * Handle millis() rollover and calculate the total uptime in seconds.
 * This function must be called at least once for every 50 days to be
 * able to see the rollover.
 */ 
unsigned long CalculateUptimeSeconds(void) {
  static unsigned int  _rolloverCount   = 0;    // Number of 0xFFFFFFFF rollover we had in millis()
  static unsigned long _lastMillis      = 0;    // Value of the last millis() 

  // Get the current milliseconds uptime from the system.
  // Note: This only works as long as no one else did hook up with timer0
  //       because the arduino system uses timer0 to manage delay() and millis().
  unsigned long currentMilliSeconds = millis();

  // If we had a rollover we count that.
  if (currentMilliSeconds < _lastMillis) {
    _rolloverCount++;
  }

  // Now store the current number of milliseconds for the next round.
  _lastMillis = currentMilliSeconds;    

  // Based on the current milliseconds and the number of rollovers
  // we had in total we calculate here the uptime in seconds since 
  // poweron or reset.
  // Caution: Because we shorten millis to seconds we may miss one 
  // second for every rollover (1 second every 50 days).
  return (0xFFFFFFFF / 1000 ) * _rolloverCount + (_lastMillis / 1000);  
}

// https://forum.arduino.cc/index.php?topic=71212.0
char * uptime(){
 long days=0;
 long hours=0;
 long mins=0;
 long secs=0;
static char buf[32]; // make a global array, http://forum.arduino.cc/index.php?topic=63659.0
// secs = currentmillis/1000; //convect milliseconds to seconds
 secs = CalculateUptimeSeconds();
 mins=secs/60; //convert seconds to minutes
 hours=mins/60; //convert minutes to hours
 days=hours/24; //convert hours to days
 secs=secs-(mins*60); //subtract the coverted seconds to minutes in order to display 59 secs max
 mins=mins-(hours*60); //subtract the coverted minutes to hours in order to display 59 minutes max
 hours=hours-(days*24); //subtract the coverted hours to days in order to display 23 hours max
 //Display results
   sprintf(buf,"Uptime %d days %2.2d:%2.2d:%2.2d", (byte)days, (byte)hours, (byte)mins, (byte)secs );
  return buf;
}
