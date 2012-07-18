/* Maximum number of errors to switch to baro (integer 1..20) */
#define SONAR_ERROR_MAX 10

#ifdef SONAR
	static int16_t SonarAlt = 0; // distance, cm (0..SONAR_MAX_DISTANCE)
	static uint8_t SonarErrors = 0; // errors count (0..SONAR_ERROR_MAX). 
	
	#define SONAR_USED (SonarErrors < SONAR_ERROR_MAX)
#else
	#define SONAR_USED 0
#endif



