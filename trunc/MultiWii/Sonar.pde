/**************************************************************
Sonar sensor HC-SR04 support (by alexmos)
http://www.google.ru/url?sa=t&rct=j&q=HC-SR04&source=web&cd=1&ved=0CCUQFjAA&url=http%3A%2F%2Fjaktek.com%2Fwp-content%2Fuploads%2F2011%2F12%2FHC-SR04.pdf&ei=19gdT7GPHKLk4QShyciWDQ&usg=AFQjCNGvwkkPlRVU3B2v7KfGMKRTPYZ4hw

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

/* Maximum measured distance, mm */
#define SONAR_MAX_DISTANCE 4500

/* Maximum measure time, ms 
* If no signal received after this time, start next measure */
#define SONAR_MAX_TIME 300

/* Pause between measures, ms. 
* (recomended 50ms to skip echo from previous measure) */
#define SONAR_WAIT_TIME 50

/* If measuring takes more than this time, result treated as error. (ms) */
/* (I have added this for my buggy HC-SR04 sensor) */
#define SONAR_ERROR_TIME 150

/* Maximum number of errors (i.e. 'sonarError' variable upper limit) */
#define SONAR_ERROR_MAX 10


/* LPF factor (integer 1..10). Bigger values means less noise (and less reaction speed).
	Comment it to disable LPF at all */
#define SONAR_LPF_FACTOR 5




/* Define PIN mask to setup interrupt and to read data */
#if (SONAR_READ==12)
	#define SONAR_READ_MASK 1<<4
#endif
#if (SONAR_READ==8)
	#define SONAR_READ_MASK 1<<0
#endif


static uint16_t startTime = 0; // 0 - finished, >0 - in progress
volatile uint8_t state = 0; // 0 - idle, 1 - measuring, 2 - pause before next measure
volatile uint16_t sonarData = 0; // measured time, us

// Configure pins and interrupt
void initSonar() {
  pinMode(SONAR_PING, OUTPUT); 
  pinMode(SONAR_READ, INPUT); 
  digitalWrite(SONAR_READ, LOW);
  //digitalWrite(SONAR_READ, HIGH); // enable pullups
  
  #if defined(PROMINI)
	  PCICR |= (1<<0) ; // PCINT activated for PINS [D8-D13] on port B
  	PCMSK0 = SONAR_READ_MASK; // trigger interrupt on this pin only
  #endif
  #if defined(MEGA)
    // TODO: setup interrupt on MEGA
  #endif
}

// Install interrupt  handler
ISR(PCINT0_vect) {
  uint8_t pin;
  uint16_t cTime;
  static uint16_t edgeTime;

  pin = PINB;
  cTime = micros();
  sei(); // re-enable interrupts
  
  // Read sonar pin state
  if(pin & SONAR_READ_MASK) { 
  	edgeTime = cTime;
  } else {
		#ifdef SONAR_DEBUG
			// measureTime = millis() - startTime; TODO: uncomment!
		#endif
		
		sonarData = cTime - edgeTime;  // sonarData will be processed later (exit this interrupt quickly)

	  state = 2; // finished measure
  }
}



inline void incError() {
	if(sonarErrors < SONAR_ERROR_MAX)
		sonarErrors++;
}	

// Trigger sonar measure and calculate distance
inline void sonarTrigger() {
	uint16_t curTime = millis();
	uint16_t dTime = curTime - startTime;

	// If we are waiting too long,  finish waiting and increase error counter
	if(dTime > SONAR_MAX_TIME) {
		incError();
		state = 0;
	}
	
	else if(state == 2) { // Measure finished
		if(dTime > SONAR_ERROR_TIME) { // wrong time, it should be error!
			incError();
			state = 0;
		} else if(dTime > SONAR_WAIT_TIME) {
			uint16_t dist = sonarData / 6;
			measureTime = dist; //TODO: remove!

		  if(dist < SONAR_MAX_DISTANCE) { // valid data received
				// Apply LPF filter
			  #ifdef SONAR_LPF_FACTOR
				  sonarDistance = (sonarDistance * SONAR_LPF_FACTOR  + dist) / (SONAR_LPF_FACTOR + 1);
			  #else
			    sonarDistance = dist;
			  #endif
			  
			  sonarErrors = 0; 
		  } else {
		  	incError();
		  }
		  
		  state = 0; // ready for next measure
		}
	}

	// Start new measure
	if(state == 0) {
		digitalWrite(SONAR_PING, HIGH);
		startTime = curTime;
		state = 1;
	  delayMicroseconds(10); 
	  digitalWrite(SONAR_PING, LOW);
	}
}
	  

#endif