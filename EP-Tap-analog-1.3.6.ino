/*
  EP-Tap-analog_1_0.ino  version 1.3.6
  Analog tap responses for E-Prime
  Latest revision by EKF 2015-10-23

  Collects a train of response times from thresholded analog input, sends
  results to a serial receiver (e.g., E-Prime).

  Circuit:
  Analog signal to A0; specifically, piezoelectric element, with ...
  . Also accepts digital (e.g., button) inputs at pins 6 & 7.
 
   Updated circuit (2017-0417): 0.1 micro-Farad ceramic cap in parallel with 
   sensor, 100 kohm R, and diode in series to A0 (rectifier + low-pass filter)
   
  Input parameters from serial port:  How many responses to collect, time limit
  (ms).  (If requested number of responses exceeds the maximum allowed, just get
  the maximum allowed instead.)  Data collection begins right after getting the
  termination character of the last parameter.

  Serial output:  Echo of input parameters (number of responses to collect, time
  limit); number of responses collected, timestamp of start of data collection;
  response timestamps (relative to start of data collection); timestamp of end
  of output.  All timestamp values in us.

  On-board LED:  Lights when Arduino is ready to take trial parameters, and
  during debounce period before each response; LED off means ready to take
  responses, or reporting results.

  All serial I/O in the form of integers terminated by a nondigit character.
  When testing with Arduino Serial Monitor, set to "No line ending" to better
  emulate communication with E-Prime.

  Using micros() just to avoid leap-ms behavior of millis() about every 42 ms.
  Although micros() rolls over about every 71.5 min, the modulo nature of
  unsigned arithmetic means that relative or difference times come out correctly
  as long as the two times do not cross more than one rollover.

  This code is available under the GNU GPLv3,
  http://www.gnu.org/licenses/gpl.html

  Developers:  Ellie Fromboluti, David McFarlane, Michigan State University.
  Development platform:  Arduino IDE 1.6.0.
  Hardware platform:  Aruduino Uno (R3?)  //?

  Predecessors:
  Baath, R. (2011), "Construction of a low latency tapping board". LUCS Minor
    17.  (Arduino + piezoelectric element)
    http://www.sumsar.net/blog/2012/09/construction-of-a-low-latency-tapping-board/
  Margolis, M. (2011). Arduino Cookbook, O'Reilly Media.  (Arduino
    SerialReceiveMultipleFields sketch)
  Schubert, T. W., D'Ausilio, A, & Canto, R. (2013), "Using Arduino
    microcontroller boards to measure response latencies". Behav Res Meth
    45:1332-1346.  (Arduino + E-Prime)
    http://link.springer.com/article/10.3758/s13428-013-0336-z
    https://reactiontimes.wordpress.com/2013/02/25/arduinort_brm/
    https://osf.io/cuzj5/
  Schultz & van Vugt (2015)"taparduino" library of functions https://github.com/florisvanvugt/taparduino.git
  
  Revisions:
  2017-04-17 (EKF): v1.3.6 -- modify voltage thresholds for rectifier+filter circuit
  2015-10-23 (EKF): v1.3.3 -- bug fix for occasional resps < tDebounce. Change tDebounce variable type from int to long 
  2015-10-16 (GRK): v1.3.2 -- changes for new two-diode circuit with reversed wiring. 5V max.
    adThreshHigh = 204 (1V); adThreshLow = 204; tDebounce = 60ms.
  2015-10-01 (EKF): v1.3.1 -- minor changes; working with new wiring (voltage divider, 10k R; sim to S&vV)
    lowering high voltage threshold (based on trial and error)
  2015-09-08 (EKF): v1.3 -- tDebounce to 140ms (time to look for tap), voltage thres to 10 / 300 (good for cyril/dredd type)
     tried adding a delayMicroseconds()to wait for 60ms before looking for another tap (total debounce time 100ms). 
     MAY WANT TO CONSIDER SOME KIND OF HARDWARE DEBOUNCE (damping materials, or circuit) 
  2015-08-20 (EKF): v1.2 -- removed 'wait' loop from last revision. lenghtened tDebounce to 80ms
    (consistent with total dBounce time of S&vV) and lowered adThreshLow from 20 to 10 (to catch more 
    accidental dbl taps / bounce in the piezo)
  2015-08-14 (EKF): Version 1.1 -- moved debounce loop after 'get one response' loop (within
   the GET RESPONSES loop. added a 'wait' loop with debounce offset time threshold to deal 
   double taps. basing times on Schultz & van Vugt: https://github.com/florisvanvugt/taparduino
   and on viewing analog response (in Audacity) of specific sensors. 41mm and 27mm piezos
   may require different 'wait' times (based on resonance properties).
  2015-06-08 (EKF): Version 1.0 -- Upgrading, ready to run with NIH experiment
  2015-06-08 (EKF): Version 0.3 -- Playing around with debouncing parameters
  2015-03-27 (DKM):  Version 0.2 -- Added conditional compilation of debug
    messages to serial output.  Added on-board LED.  Added array overrun
    protection.
  2015-03-02 (DKM):  Version 0.1 -- First fully working prototype.
  */


// enable/disable debugging messages to serial:
// #define  DEBUG  //?

const int  led1 = 13;  // on-board LED
const int  button1 = 6, button2 = 7;  //?
const int  adInput = A0;  // for piezoelectric input
// analog thresholds:
const unsigned int  adThreshHigh = 25;  // detect tap on piezo
const unsigned int  adThreshLow = 25;  // end of tap on piezo
const unsigned long  tDebounce = 60000L;  // in us. wait for analog to go below thresh (40ms from S&vV)
//const unsigned long  tOffDebounce = 100000; // in us. wait before looking for another tap (40ms from on S&vV)

// Some global variables, not out of necessity but for reasons of convenience,
// efficiency, & laziness ...
// Response data array (beware of Arduino RAM limits, large arrays will compile
// & upload but not run):
const int  MaxRespCount = 256;
unsigned long  respTAbs[MaxRespCount];  // us


void setup() 
{
  Serial.begin(9600);  // Must match E-Prime or Serial Monitor baud rate!
  // Make Serial.parseInt() wait for terminating nondigit:
  Serial.setTimeout(24L * 60L * 60L * 1000L);  // close enough to forever
  pinMode(led1, OUTPUT);
  pinMode(adInput, INPUT); 
  pinMode(button1, INPUT);  //?
  pinMode(button2, INPUT);  //?
}

void loop()  // one trial
{
  unsigned long  respCountTarget, respTLimit_ms;  // parameters
  unsigned long  respTLimit;  // us
  unsigned int  respCount;
  unsigned long  t0, t;

  // GET TRIAL PARAMETERS ...
  #ifdef DEBUG
  Serial.println("Getting trial parameters ...");
  #endif
  digitalWrite(led1, HIGH);
  while (Serial.available())  Serial.read();  // Ignore stale serial input data
  // Two trial parameters input here -- respCountTarget & respTLimit_ms:
  respCountTarget = Serial.parseInt();
  respTLimit_ms = Serial.parseInt();
  t0 = micros();  // starting timestamp
  // protect against array overrun:
  if (respCountTarget > MaxRespCount)  respCountTarget = MaxRespCount;
  respTLimit = respTLimit_ms * 1000;
  #ifdef DEBUG
  Serial.print(respCountTarget);  Serial.print(" ");
  Serial.print(respTLimit_ms);  Serial.print(" ");
  Serial.println(t0);
  Serial.println("Getting responses ...");
  #endif
  // GET RESPONSES ...
  for (respCount = 0;  (respCount < respCountTarget);  ++respCount)
  {
    digitalWrite(led1, HIGH);
    // debounce based on going below specified voltage in a time frame ...
    for (t = micros(); (micros() - t) < tDebounce;  /**/)
    {
      if (analogRead(adInput) >= adThreshLow)  t = micros();  // extend timeout
//      if (digitalRead(button1) | digitalRead(button2))  t = micros();  //?
//      if (digitalRead(button1) == 1 ) t = micros();  //?
//      delayMicroseconds(tOffDebounce);
       
    }
    digitalWrite(led1, LOW);
           
    // get one response ...
    do
    {
      t = micros();
      if ((t - t0) >= respTLimit)  goto respLoopExit;
      if (digitalRead(button1) | digitalRead(button2))  break;  //?
    } while (analogRead(adInput) < adThreshHigh);
    respTAbs[respCount] = t;
  }
respLoopExit:
  // REPORT RESULTS ...
  #ifdef DEBUG
  Serial.println("Results ...");
  #endif
  // trial parameters:
  Serial.print(respCountTarget);  Serial.print(" ");
  Serial.print(respTLimit_ms);  Serial.print(" ");
  // starting timestamp, number responses collected:
  Serial.print(t0);  Serial.print(" ");  // to help link to E-Prime clock times
  Serial.print(respCount);  Serial.print(" ");
  // response timestamps (us) from start:
  for (int i = 0;  i < respCount;  ++i)
  {
    Serial.print( respTAbs[i] - t0 );  Serial.print(" ");
  }
  // current timestamp to help link to E-Prime clock times:
  Serial.print( micros() );
  Serial.println(".");  // end of results
  #ifdef DEBUG
  Serial.print("Trial done.\n");
  #endif
}  // loop()
