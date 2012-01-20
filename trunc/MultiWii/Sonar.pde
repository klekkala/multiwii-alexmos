/**************************************************************
Sonar sensor HC-SR04 support (by alexmos)

From datasheet:
	"Adopt IO trigger through supplying at least 10us sequence of high level signal.
	The module automatically send eight 40khz square wave and automatically detect 
		whether receive the returning pulse signal.
	If there is signals returning, through outputting high level and the time 
		of high level continuing is the time of that from the ultrasonic transmitting to receiving. 
	Test distance = (high level time * sound velocity (340M/S) / 2."

Connections HC-SR04 <-> PROMINI:
 VCC <-> VCC
 trig(T) <-> A2
 echo(R) <-> D12 (or D8, see definitions below)
 GND <-> GND

Note:
-CAMTRIG and RCAUXPIN12 must be disabled in general config!
-Implemented only for PROMINI board: I have only this one, sorry.
***************************************************************/



#ifdef SONAR

/* Maximum measured distance, cm */
#define SONAR_MAX_DISTANCE 500

/* Maximum measure time, micros */
#define SONAR_MAX_TIME 30000  

/* Maximum number of errors when sonarDistance should be treated as 'undefined' */
#define SONAR_MAX_ERRORS 10

/* LPF factor (integer >=0). Bigger values means less noise (and less reaction speed).
	Comment it to disable LPF at all */
#define SONAR_LPF_FACTOR 1

/* LPF velocity factor (integer >=1). Bigger values means less system inertion (velocity does not taked in to account)
  Works only with SONAR_LPF_FACTOR > 0 */
#define SONAR_LPF_VEL_FACTOR 1




/* Define PIN mask to setup interrupt and to read data */
#if (SONAR_READ==12)
	#define SONAR_READ_MASK 1<<4
#endif
#if (SONAR_READ==8)
	#define SONAR_READ_MASK 1<<0
#endif



/* Global variables holding current sensor state */
volatile int16_t sonarDistance = 0; // distance, cm (0..SONAR_MAX_DISTANCE)
volatile uint8_t sonarErrors = 0; // errors count (0..SONAR_MAX_ERRORS). Increased by 1 each time answer was missed.
volatile uint16_t startTime = 0; // 0 - finished, >0 - in progress


// Configure pins and interrupt
void initSonar() {
  pinMode(SONAR_PING, OUTPUT); 
  pinMode(SONAR_READ, INPUT); 
  digitalWrite(SONAR_READ, HIGH); // enable pullups
  
  #if defined(PROMINI)
	  PCICR |= (1<<0) ; // PCINT activated for PINS [D8-D13] on port B
  	PCMSK0 = SONAR_READ_MASK; // trigger interrupt on this pin only
  #endif
  #if defined(MEGA)
    // TODO: setup interrupt on MEGA
  #endif
}

// Handle interrupt to receive data
#if defined(PROMINI)
  ISR(PCINT0_vect) {
    uint8_t pin;
    uint16_t cTime,dTime;
    static uint16_t edgeTime;
    uint16_t sonarData;

    pin = PINB;
    cTime = micros();
    sei(); // re-enable interrupts
    
    // Read sonar pin state
    if(pin & SONAR_READ_MASK) { 
    	edgeTime = cTime;
    } else {
      processSonarData((cTime-edgeTime) / 59);  //  distance_cm = time_us * 340 * 100 / 1000000 / 2
    }
  }
#endif


// Analize measured data (in cm)
inline void processSonarData(uint16_t data)
{
  if(data < SONAR_MAX_DISTANCE) { // valid data received
		// Apply LPF filter with velocity taked in account
	  #ifdef SONAR_LPF_FACTOR
		  uint16_t T, dT;
		  static uint16_t Tprev = 0, dTprev = 0;
		  int16_t tmp;
		  static int16_t dSprev = 0;
		
		  T = micros(); // keep only 16 last bits
		  dT = T - Tprev;
		  if(dT > SONAR_MAX_TIME) dT = SONAR_MAX_TIME;  // too big interval means measure error or time counter overflow
			tmp = sonarDistance;
		  sonarDistance = ((sonarDistance + ((int32_t)dSprev)*dT/dTprev/SONAR_LPF_VEL_FACTOR) * SONAR_LPF_FACTOR + data) / (SONAR_LPF_FACTOR + 1);
		  Tprev = T; 
		  dTprev = dT;
		  dSprev = sonarDistance - tmp;
	  
	  #else
	    sonarDistance = data;
	  #endif
	  
	  sonarErrors = 0;
  } else {
  	incError();
  }

  startTime = 0; // we are ready for next measure
}

inline void incError() {
	if(sonarErrors < SONAR_MAX_ERRORS)
		sonarErrors++;
}	

// Trigger sonar measure. 
// TODO: 10us pause here, is it better to replace waiting by usefull code?
void sonarTrigger() {
	uint16_t curTime = micros();

	// If we are waiting too long,  finish waiting and increse error counter
	if(curTime - startTime > SONAR_MAX_TIME) {
		incError();
		startTime = 0;
	}

	// Start new measure only if previous measure was finished
	if(startTime == 0) {
		startTime = curTime;
		digitalWrite(SONAR_PING, HIGH);
	  delayMicroseconds(10); 
	  digitalWrite(SONAR_PING, LOW);
	}
}
	  

#endif