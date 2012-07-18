void computeIMU () {
  uint8_t axis;
  static int16_t gyroADCprevious[3] = {0,0,0};
  int16_t gyroADCp[3];
  int16_t gyroADCinter[3];
  static uint32_t timeInterleave = 0;

  //we separate the 2 situations because reading gyro values with a gyro only setup can be acchieved at a higher rate
  //gyro+nunchuk: we must wait for a quite high delay betwwen 2 reads to get both WM+ and Nunchuk data. It works with 3ms
  //gyro only: the delay to read 2 consecutive values can be reduced to only 0.65ms
  if (!ACC && nunchuk) {
    annexCode();
    while((micros()-timeInterleave)<INTERLEAVING_DELAY) ; //interleaving delay between 2 consecutive reads
    timeInterleave=micros();
    WMP_getRawADC();
    getEstimatedAttitude(); // computation time must last less than one interleaving delay
    while((micros()-timeInterleave)<INTERLEAVING_DELAY) ; //interleaving delay between 2 consecutive reads
    timeInterleave=micros();
    while(WMP_getRawADC() != 1) ; // For this interleaving reading, we must have a gyro update at this point (less delay)

    for (axis = 0; axis < 3; axis++) {
      // empirical, we take a weighted value of the current and the previous values
      // /4 is to average 4 values, note: overflow is not possible for WMP gyro here
      gyroData[axis] = (gyroADC[axis]*3+gyroADCprevious[axis]+2)/4;
      gyroADCprevious[axis] = gyroADC[axis];
    }
  } else {
    #if ACC
      ACC_getADC();
      getEstimatedAttitude();
    #endif
    #if GYRO
      Gyro_getADC();
    #else
      WMP_getRawADC();
    #endif
    for (axis = 0; axis < 3; axis++)
      gyroADCp[axis] =  gyroADC[axis];
    timeInterleave=micros();
    annexCode();
    if ((micros()-timeInterleave)>650) {
       annex650_overrun_count++;
    } else {
       while((micros()-timeInterleave)<650) ; //empirical, interleaving delay between 2 consecutive reads
    }
    #if GYRO
      Gyro_getADC();
    #else
      WMP_getRawADC();
    #endif
    for (axis = 0; axis < 3; axis++) {
      gyroADCinter[axis] =  gyroADC[axis]+gyroADCp[axis];
      // empirical, we take a weighted value of the current and the previous values
      gyroData[axis] = (gyroADCinter[axis]+gyroADCprevious[axis]+1)/3;
      gyroADCprevious[axis] = gyroADCinter[axis]/2;
      if (!ACC) accADC[axis]=0;
    }
  }
  #if defined(GYRO_SMOOTHING)
    static uint8_t Smoothing[3]  = GYRO_SMOOTHING; // How much to smoothen with per axis
    static int16_t gyroSmooth[3] = {0,0,0};
    for (axis = 0; axis < 3; axis++) {
      gyroData[axis] = (gyroSmooth[axis]*(Smoothing[axis]-1)+gyroData[axis]+1)/Smoothing[axis];
      gyroSmooth[axis] = gyroData[axis];
    }
  #elif defined(TRI)
    static int16_t gyroYawSmooth = 0;
    gyroData[YAW] = (gyroYawSmooth*2+gyroData[YAW]+1)/3;
    gyroYawSmooth = gyroData[YAW];
  #endif
}

// **************************************************
// Simplified IMU based on "Complementary Filter"
// Inspired by http://starlino.com/imu_guide.html
//
// adapted by ziss_dm : http://www.multiwii.com/forum/viewtopic.php?f=8&t=198
//
// The following ideas was used in this project:
// 1) Rotation matrix: http://en.wikipedia.org/wiki/Rotation_matrix
// 2) Small-angle approximation: http://en.wikipedia.org/wiki/Small-angle_approximation
// 3) C. Hastings approximation for atan2()
// 4) Optimization tricks: http://www.hackersdelight.org/
//
// Currently Magnetometer uses separate CF which is used only
// for heading approximation.
//
// Modified: 19/04/2011  by ziss_dm
// Version: V1.1
//
// code size reduction and tmp vector intermediate step for vector rotation computation: October 2011 by Alex
// **************************************************

//******  advanced users settings *******************
/* Set the Low Pass Filter factor for ACC */
/* Increasing this value would reduce ACC noise (visible in GUI), but would increase ACC lag time*/
/* Comment this if  you do not want filter at all.*/
/* Default WMC value: 8*/
#define ACC_LPF_FACTOR 6

/* Set the Low Pass Filter factor for Magnetometer */
/* Increasing this value would reduce Magnetometer noise (not visible in GUI), but would increase Magnetometer lag time*/
/* Comment this if  you do not want filter at all.*/
/* Default WMC value: n/a*/
//#define MG_LPF_FACTOR 4
/* Set the Gyro Weight for Gyro/Acc complementary filter */
/* Increasing this value would reduce and delay Acc influence on the output of the filter*/
/* Default WMC value: 300*/
#define GYR_CMPF_FACTOR 500.0f

/* Set the Gyro Weight for Gyro/Magnetometer complementary filter */
/* Increasing this value would reduce and delay Magnetometer influence on the output of the filter*/
/* Default WMC value: n/a*/
#define GYR_CMPFM_FACTOR 500.0f

//****** end of advanced users settings *************

#define INV_GYR_CMPF_FACTOR   (1.0f / (GYR_CMPF_FACTOR  + 1.0f))
#define INV_GYR_CMPFM_FACTOR  (1.0f / (GYR_CMPFM_FACTOR + 1.0f))
#if GYRO
  #define GYRO_SCALE ((2380 * PI)/((32767.0f / 4.0f ) * 180.0f * 1000000.0f)) //should be 2279.44 but 2380 gives better result
  // +-2000/sec deg scale
  //#define GYRO_SCALE ((200.0f * PI)/((32768.0f / 5.0f / 4.0f ) * 180.0f * 1000000.0f) * 1.5f)     
  // +- 200/sec deg scale
  // 1.5 is emperical, not sure what it means
  // should be in rad/sec
#else
  #define GYRO_SCALE (1.0f/200e6f)
  // empirical, depends on WMP on IDG datasheet, tied of deg/ms sensibility
  // !!!!should be adjusted to the rad/sec
#endif 
// Small angle approximation
#define ssin(val) (val)
#define scos(val) 1.0f

typedef struct fp_vector {
  float X;
  float Y;
  float Z;
} t_fp_vector_def;

typedef union {
  float   A[3];
  t_fp_vector_def V;
} t_fp_vector;

int16_t _atan2(float y, float x){
  #define fp_is_neg(val) ((((uint8_t*)&val)[3] & 0x80) != 0)
  float z = y / x;
  int16_t zi = abs(int16_t(z * 100)); 
  int8_t y_neg = fp_is_neg(y);
  if ( zi < 100 ){
    if (zi > 10) 
     z = z / (1.0f + 0.28f * z * z);
   if (fp_is_neg(x)) {
     if (y_neg) z -= PI;
     else z += PI;
   }
  } else {
   z = (PI / 2.0f) - z / (z * z + 0.28f);
   if (y_neg) z -= PI;
  }
  z *= (180.0f / PI * 10); 
  return z;
}

// Rotate Estimated vector(s) with small angle approximation, according to the gyro data
void rotateV(struct fp_vector *v,float* delta) {
  fp_vector v_tmp = *v;
  v->Z -= delta[ROLL]  * v_tmp.X + delta[PITCH] * v_tmp.Y;
  v->X += delta[ROLL]  * v_tmp.Z - delta[YAW]   * v_tmp.Y;
  v->Y += delta[PITCH] * v_tmp.Z + delta[YAW]   * v_tmp.X; 
}

// alexmos: need it later
static t_fp_vector EstG = {0,0,0};
static float InvG = 0;
static float HRM[2] = {1,0};

void getEstimatedAttitude(){
  uint8_t axis;
  int32_t accMag = 0;
#if MAG
  static t_fp_vector EstM;
#endif
#if defined(MG_LPF_FACTOR)
  static int16_t mgSmooth[3]; 
#endif
#if defined(ACC_LPF_FACTOR)
  static int16_t accTemp[3];  //projection of smoothed and normalized magnetic vector on x/y/z axis, as measured by magnetometer
#endif
  static uint16_t previousT;
  uint16_t currentT = micros();
  float scale, deltaGyroAngle[3];

  scale = (currentT - previousT) * GYRO_SCALE;
  previousT = currentT;

  // Initialization
  for (axis = 0; axis < 3; axis++) {
    deltaGyroAngle[axis] = gyroADC[axis]  * scale;
    #if defined(ACC_LPF_FACTOR)
      accTemp[axis] = (accTemp[axis] - (accTemp[axis] >>ACC_LPF_FACTOR)) + accADC[axis];
      accSmooth[axis] = accTemp[axis]>>ACC_LPF_FACTOR;
      #define ACC_VALUE accSmooth[axis]
    #else  
      accSmooth[axis] = accADC[axis];
      #define ACC_VALUE accADC[axis]
    #endif
//    accMag += (ACC_VALUE * 10 / (int16_t)acc_1G) * (ACC_VALUE * 10 / (int16_t)acc_1G);
    accMag += (int32_t)ACC_VALUE*ACC_VALUE ;
    #if MAG
      #if defined(MG_LPF_FACTOR)
        mgSmooth[axis] = (mgSmooth[axis] * (MG_LPF_FACTOR - 1) + magADC[axis]) / MG_LPF_FACTOR; // LPF for Magnetometer values
        #define MAG_VALUE mgSmooth[axis]
      #else  
        #define MAG_VALUE magADC[axis]
      #endif
    #endif
  }
  accMag = accMag*100/((int32_t)acc_1G*acc_1G);
  
  rotateV(&EstG.V,deltaGyroAngle);
  #if MAG
    rotateV(&EstM.V,deltaGyroAngle);
  #endif 

  if ( abs(accSmooth[ROLL])<acc_25deg && abs(accSmooth[PITCH])<acc_25deg && accSmooth[YAW]>0)
    smallAngle25 = 1;
  else
    smallAngle25 = 0;

  // Apply complimentary filter (Gyro drift correction)
  // If accel magnitude >1.4G or <0.6G and ACC vector outside of the limit range => we neutralize the effect of accelerometers in the angle estimation.
  // To do that, we just skip filter, as EstV already rotated by Gyro
  if ( ( 36 < accMag && accMag < 196 ) || smallAngle25 )
    for (axis = 0; axis < 3; axis++) {
      int16_t acc = ACC_VALUE;
      #if !defined(TRUSTED_ACCZ)
        if (smallAngle25 && axis == YAW)
          //We consider ACCZ = acc_1G when the acc on other axis is small.
          //It's a tweak to deal with some configs where ACC_Z tends to a value < acc_1G when high throttle is applied.
          //This tweak applies only when the multi is not in inverted position
          acc = acc_1G;      
      #endif
      EstG.A[axis] = (EstG.A[axis] * GYR_CMPF_FACTOR + acc) * INV_GYR_CMPF_FACTOR;
    }
  #if MAG
    for (axis = 0; axis < 3; axis++)
      EstM.A[axis] = (EstM.A[axis] * GYR_CMPFM_FACTOR  + MAG_VALUE) * INV_GYR_CMPFM_FACTOR;
  #endif
  
  // Attitude of the estimated vector
  angle[ROLL]  =  _atan2(EstG.V.X , EstG.V.Z) ;
  angle[PITCH] =  _atan2(EstG.V.Y , EstG.V.Z) ;
  #if MAG
    /* 
    //alexmos: calculate rotation matrix to rotate heading
    HRM[0] = EstG.V.X * EstM.V.Z - EstG.V.Z * EstM.V.X;
    HRM[1] = EstG.V.Z * EstM.V.Y - EstG.V.Y * EstM.V.Z; 
    heading = _atan2( HRM[0] , HRM[1]  ) / 10;
    */
    
    // Attitude of the cross product vector GxM
    heading = _atan2( EstG.V.X * EstM.V.Z - EstG.V.Z * EstM.V.X , EstG.V.Z * EstM.V.Y - EstG.V.Y * EstM.V.Z  ) / 10;
    
    /*
    // Normalize rotation matrix
    float mod = InvSqrt(fsq(HRM[0]) + fsq(HRM[0]));
   	HRM[0]/= mod;
   	HRM[1]/= mod;
   	*/
  #endif
  
  // alexmos: calc some useful values
  InvG = InvSqrt(fsq(EstG.V.X) + fsq(EstG.V.Y) + fsq(EstG.V.Z));
 	cosZ =  EstG.V.Z * InvG * 100.0f; // cos(angleZ) * 100.
}




/* Exponential moving average filter (optimized for integers) with factor = 2^n */
typedef struct avg_var16 {
  int32_t buf; // internal bufer to store non-rounded average value
  int16_t res; // result (rounded to int)
} t_avg_var16;

typedef struct avg_var8 {
  int16_t buf; // internal bufer to store non-rounded average value
  int8_t res; // result (rounded to int)
} t_avg_var8;
/* n=(1..16) */
void average16(struct avg_var16 *avg, int16_t cur, int8_t n) {
	avg->buf+= cur - avg->res;
	avg->res = avg->buf >> n;
}
/* n=(1..8) */
void average8(struct avg_var8 *avg, int8_t cur, int8_t n) {
	avg->buf+= cur - avg->res;
	avg->res = avg->buf >> n;
}



/* alexmos: baro + ACC altitude estimator */
/* It outputs altitude, velocity and 'pure' acceleration projected on Z axis (with 1G substracted) */
/* It has a very good resistance to inclinations, horisontal movements, ACC drift and baro noise. */

/* Below are rates of correction estimated altitude, acceleration and velocity. */
/* You can increse it if more precise BARO used */
#define ALT_P (1.0f/ALT_TRUST_ACC) // estimated altitude correction (default 0.05)
#define ACC_I 0.0001f // ACC zero calibration (default 0.0001)
#define VEL_P 0.1f // velocity correction (default 0.1)
#define VEL_DAMP 0.005f // velocity damping factor (helps remove oscillations) default 0.005


void getEstimatedAltitude(){
  static int8_t initDone = 0;
  static float alt = 0; // cm
  static float vel = 0; // cm/sec
 	static float errI[3] = {0,0,0};
  static float thrWindCorrScale = 0, accVelScale = 0; // config vars
  float accZ, err, tmp;
  static t_avg_var16 baroSonarDiff = {0,0}, avgAlt = {0,0}; 
  int32_t sensorAlt;
  int8_t axis;

	// delta time
	static uint16_t prevTime = 0;
	uint16_t tmpTime = micros();
	uint16_t dTime = tmpTime - prevTime;
	prevTime = tmpTime;
    
  // soft start
  if(!initDone) {
  	if(cycleCnt > 100 && BaroAlt != 0) { // start only if sensor data avaliable
		  #ifdef SONAR
	  		alt = SonarAlt;
	  		BaroAltGround = BaroAlt - alt;
		  #else
			  BaroAltGround = BaroAlt;
		  	alt = 0;
		  #endif
	  	accVelScale = 9.80665f / acc_1G / 10000.0f;
	  	#ifdef THROTTLE_WIND_CORRECTION
	  		thrWindCorrScale = ((float)THROTTLE_WIND_CORRECTION) / acc_1G / acc_1G;
	  	#endif
		  EstG.A[2] = accADC[2]; // G-vector not initiated properly at start, do it here
	  	errI[2] = accADC[2] - (int16_t)acc_1G; 

	  	initDone = 1;
	  }
	  return;
  }

  sensorAlt = BaroAlt - BaroAltGround;

  #ifdef SONAR
  	// get sonar data and trigger next measure
  	sonarUpdate();

		// Get difference between sonar and baro and slightly average it in time
		if(SONAR_USED) {
			average16(&baroSonarDiff, constrain(SonarAlt - sensorAlt, -32000, 32000), 7);
		}

		// Check if sonar is not crazy: its value compared to baro should not go outside window +/-3m
		if(abs(baroSonarDiff.res) < 300) {
			sensorAlt+= baroSonarDiff.res;

			// Sonar gives precise values, use it
			if(SonarErrors == 0) {
				sensorAlt = SonarAlt;
			// Sonar gives some errors: use cross-section of SONAR and BARO altitudes to softly switch to baro
			}  else if(SONAR_USED) {
				sensorAlt = (SonarAlt * (SONAR_ERROR_MAX - SonarErrors) + sensorAlt * SonarErrors)/SONAR_ERROR_MAX;
			}
		} else {
			// Sonar is crasy, so use baro only + sonar value on the end of limits
			sensorAlt+= constrain(baroSonarDiff.res, -300, 300);
		}
  #endif

  
  // error between estimated alt and BARO alt
  //err = constrain((int16_t)((int32_t)alt - sensorAlt), -500, 500) * ALT_P;
  err = constrain((int16_t)(EstAlt - sensorAlt), -500, 500) * ALT_P;
  
  // I term of error for each axis
  // (If Z angle is not zero, we should take X and Y axis into account and correct them too.
  // We will spread I error proportional to Cos(angle) for each axis
  // TODO: we got "real" ACC zero in this calibration procedure and may use it to correct ACC in angle estimation, too
  tmp = err*ACC_I;
	errI[2]+= EstG.A[2] * tmp;
	if(abs(cosZ) < 95) {
  	errI[0]+= EstG.A[0] * 2 * tmp; 
  	errI[1]+= EstG.A[1] * 2 * tmp; 
  }
  
  // Project ACC vector A to 'global' Z axis (estimated by gyro vector G) with I term taked into account
  // Math: accZ = (A + errI) * G / |G| - 1G
	tmp = (accADC[0] - errI[0]) * EstG.V.X + (accADC[1] - errI[1]) * EstG.V.Y;
	accZ = (tmp + (accADC[2] - errI[2]) * EstG.V.Z) * InvG - acc_1G;
	
 	#if THROTTLE_ANGLE_CORRECTION > 0
	  // Correct throttle to keep constant altitude in fast movement
 		throttleAngleCorrection = THROTTLE_ANGLE_CORRECTION * constrain((int16_t)(tmp * thrWindCorrScale) + 100 - cosZ, -150, 150) / 100; // 16 bit ok: 200*150 = 30000
 	#endif

  
  // Integrator - velocity, cm/sec
  vel+= accZ*accVelScale*dTime - err*VEL_P - vel*VEL_DAMP;
  
	// Integrator - altitude, cm
	alt+= vel * dTime * 1.0e-6f - err;
	

	#if defined(ALT_LPF_FACTOR) && ALT_LPF_FACTOR>0
		// Apply LPF
		average16(&avgAlt, alt, ALT_LPF_FACTOR);
	  EstAlt = avgAlt.res;
	#else
  	EstAlt = alt;
  #endif
  
  EstVelocity = vel;

  
  // debug to GUI
  #ifdef ALT_DEBUG
  	debug1 = sensorAlt;
	  debug2 = vel*10;
	  debug3 = errI[2]*100;
	  //debug4 = (int16_t)micros() - curtime;
	#endif
}



float fsq(float x){return x * x;}

float InvSqrt (float x){ 
  union{  
    int32_t i;  
    float   f; 
  } conv; 
  conv.f = x; 
  conv.i = 0x5f3759df - (conv.i >> 1); 
  return 0.5f * conv.f * (3.0f - x * conv.f * conv.f);
} 

/* Rotate vector V(x,y) to angle delta (in 0.1 degree) using small angle approximation and integers. */
/* (Not precise but fast) */
inline void rotate16(int16_t *V, int16_t delta) {
  int16_t tmp = V[0];
  V[0]-= (int16_t)( ((int32_t)delta) * V[1] / 573);
  V[1]+= (int16_t)( ((int32_t)delta) * tmp / 573); 
}


/* Rotate vector V clockwise by heading rotation matrix */
/*
void rotate_heading_cw16(int16_t *V) {
	int16_t tmp = V[0];
	V[0] = V[0]*HRM[0] + V[1]*HRM[1];
	V[1] = tmp*HRM[1] - V[1]*HRM[0];
}	
*/

/* Rotate vector V counter-clockwise by heading rotation matrix  */
/*
void rotate_heading_ccw16(int16_t *V) {
	int16_t tmp = V[0];
	V[0] = V[0]*HRM[0] - V[1]*HRM[1];
	V[1] = tmp*HRM[1] + V[1]*HRM[0];
}	
*/



  
