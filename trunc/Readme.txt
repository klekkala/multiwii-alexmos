IMPORTANT!
There is 1-second pause before sensor calibration on startup. You have time to take hands off
after switching power to keep  ABSOLUTELY IMMOVABLE while calibrating gyroscope.
Precise gyro = stable flight!


New in r20:

-Position hold updates:
	- Faster SPI communication width DigitalWriteFast library (less cycle time)
	- ACC fusion disabled by default


New in r18:

Position Hold:
- Small bug corrcted if Alt-I > 0
- Added rotation of I-term to follow heading. It will keep wind compensation in case of copter rotation in flight
- I-term is always enabled


New in r17:

- Added position hold mode with optical flow sensor ADNS-5050. 
	It's enabled by default and start working in LEVEL mode only when sticks in their neutral position.
	To configure, see config.h and tune VEL PID's in GUI (my setup: P=5, I=0.010, D=5)
  Test video: http://www.youtube.com/watch?v=rdZnxTz1Y_g


New in r16:

- Alt Hold and sonar turns off when incination angle is too big (>60 degree).

- In Alt Hold mode, when Throttle stick moves outside deadband, copter will beep and blink led.
  It helps to return throttle stick back to its original position after altitude correction.

- AltPID slightly corrected: D-term is not depend on P-term now. I-term is not gained for sonar mode.

- Camera tilt servos disabled during calibration procedure to prevent shaking.

- YAW-D skipped in PID calculations because no efficiency.

- Some minor changes on ACC-BARO fusion algorythm: 
  - it is now invariant on ACC sensor range (tuning for various sensors not required)
  - calculations slightly optimized

- Sonar may be turned off on the fly by activating PASSTHRU mode (AltHold algorythm will switch to baro)

- Sonar is stricted by baro: it should not go outside +-3m window in case of sonar errors.
  (sonar is VERY sensitive to noise in 5V-power line, don't connect it in parallel with servos without noise filter!)

- Experimental: added throttle correction to compensate altitude drop due to Z-axis inclination.
  Aerodynamic lifting force (positive or negative) and lateral wind also taked into account.
  (It has low processor-cost because pre-calculated values are used)


New in r15:

-	More precise ACC calibration procedure (by six points), but still compatible with legacy procedure.
	Instruction (in russian): http://forum.rcdesign.ru/blogs/97312/blog13713.html
	Translated: http://translate.google.com/translate?sl=ru&tl=en&js=n&prev=_t&hl=ru&ie=UTF-8&layout=2&eotf=1&u=http%3A%2F%2Fforum.rcdesign.ru%2Fblogs%2F97312%2Fblog13713.html

- Throttle correction for inclination angle to keep constant vertical force (helps keep altitude). 
 (See THROTTLE_ANGLE_CORRECTION in config.h) 




OTHER DIFFERENCES BETWEEN OFICIAL MULTIWII FIRMWARE v1.9:	

-	Altitude estimation algorythm uses ACC+BARO sensors, and gives more precise altitude without low-pass filtering.
  As a drawback, very precise ACC calibrarion is required (but there is some kind of auto-calibration for ACC in flight)

-	Altitude hold algorytm uses stronger PID's to keep altitude in case of strong disturbances and in fast flight.
  For my setup, Alt P=10, Alt I=0.02, Alt D=20 was very good.
  
- Different altitude control in AltHold mode: you configure ALT_HOLD_DEADBAND in config.h, 
	set the throttle stick near hover and switch to AltHold (BARO) mode. 
	When throttle stick moves out of deadband region, altitude will slowly increase or decrease.

-	Added support for HC-SR04 ultrasonic sensor (and simular with Trig and Echo pins). 
	It uses free interrupt, so other extended functions and not standard configurations (such as hexacopter, AUX2,3,4 on ProMini) 
	may not works.

-	THROTTLE_EXPO - add expo curve near hover point for throttle stick (for more comfortable altitude control). 
	Expo curve is the same as configured in GUI for other axis. 
	(You need to ajust THROTTLE_HOVER  in config.h)

- More precise gyro calibration at startup.

- More trust to gyro compared to other sensors (accelerometer and magnetometer):
	GYR_CMPF_FACTOR 500.0f  (was 300)
  GYR_CMPFM_FACTOR 500.0f (was 300)

