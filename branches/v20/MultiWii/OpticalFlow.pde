/* Optical flow sensor reading and calculations */
/* (c) alexmos 2012 */
#ifdef OPTFLOW

/* attempt to fasten SPI read-write */
#if !defined(digitalWriteFast)
	#include "digitalWriteFast.h"
#endif


static uint16_t scale;

/* Sensor's row data accumulators */
static int16_t sum_dx = 0, sum_dy = 0;


/* Convert row data to displacment (in mm*10 on height 1m)  since last call */
inline void optflow_get() {
	optflow_pos[1] = constrain((int32_t)sum_dx * scale, -0x7FFF, 0x7FFF);
	optflow_pos[0] = constrain((int32_t)sum_dy * scale, -0x7FFF, 0x7FFF);

	// clear accumulated displacement
	sum_dx = 0; sum_dy = 0; 
}	










/* *************************************************** */
/* Configuration and routines for each sensor          */
#if(OPTFLOW==ADNS_5050)
	#define PRODUCT_ID          0x00 // should be 0x12
	#define PRODUCTID2          0x3e
	#define REVISION_ID         0x01
	#define MOTION_REG					0x02
	#define DELTA_Y_REG         0x03
	#define DELTA_X_REG         0x04
	#define SQUAL_REG           0x05
	#define MAXIMUM_PIXEL_REG   0x08
	#define MINIMUM_PIXEL_REG   0x0a
	#define PIXEL_SUM_REG       0x09
	#define PIXEL_DATA_REG      0x0b
	#define SHUTTER_UPPER_REG   0x06
	#define SHUTTER_LOWER_REG   0x07
	#define RESET		    0x3a
	#define MOUSE_CONTROL	0x0d
	#define MOUSE_CONTROL2 0x19

	// resolution
	//#define RES_CPI 	250
	//	#define RES_CFG 	0b10010
	//#define RES_CPI 	500
	//	#define RES_CFG 	0b10100
	//#define RES_CPI 	750
	//	#define RES_CFG 	0b10110
	//#define RES_CPI		1000 	
	//	#define RES_CFG		0b11000
	#define RES_CPI		1250
		#define RES_CFG	0b11010


inline void initOptflow() {
  pinModeFast(OF_SDIO, OUTPUT);
  pinModeFast(OF_SCLK, OUTPUT);
  pinModeFast(OF_NCS, OUTPUT);

  // reset device
  ADNS_write(RESET, 0x5a);
  delayMicroseconds(50);
  
  //scale = (uint32_t)254000  / OF_FOCAL_DIST / RES_CPI;
  scale = (uint32_t)500000 / OF_FOCAL_DIST / RES_CPI;
  
  if(ADNS_read(PRODUCT_ID) == 0x12) {
 		ADNS_write(MOUSE_CONTROL2, RES_CPI); // Set resolution
  }
}

/* Read sensor values. Should be called in every main loop to prevent sensor's internal counters overflow */
inline void optflow_update() {
  if(ADNS_read(MOTION_REG) != 0) {
  	sum_dx+= (int8_t)ADNS_read(DELTA_X_REG);
  	sum_dy+= (int8_t)ADNS_read(DELTA_Y_REG);
  }
}

/* get surface quality, (0..127) */
inline uint8_t optflow_squal() {
	return ADNS_read(SQUAL_REG);
}

/* Write byte of data to SPI */
void _spi_write(byte val) {
  for (byte i=128; i >0 ; i >>= 1) {
    digitalWriteFast (OF_SCLK, LOW);
    digitalWriteFast (OF_SDIO, (val & i) != 0 ? HIGH : LOW);
    delayMicroseconds(1);
    digitalWriteFast (OF_SCLK, HIGH);
  }
}	

/* Read register */
byte ADNS_read(byte address) {
  digitalWriteFast(OF_NCS, LOW);// select the chip
  delayMicroseconds(1);

  pinModeFast (OF_SDIO, OUTPUT);
  _spi_write(address);

  delayMicroseconds(4); // tSRAD = 4us min.

  pinModeFast (OF_SDIO, INPUT);
  byte res = 0;
  for (byte i=128; i >0 ; i >>= 1) {
    digitalWriteFast (OF_SCLK, LOW);
    delayMicroseconds(1);
    digitalWriteFast (OF_SCLK, HIGH);
    if( digitalReadFast (OF_SDIO) == HIGH )
      res |= i;
  }

  digitalWriteFast(OF_NCS, HIGH);// de-select the chip
  return res;
}

/* Write register */
void ADNS_write(byte address, byte data) {
  digitalWriteFast(OF_NCS, LOW);// select the chip
  delayMicroseconds(1);

  pinModeFast (OF_SDIO, OUTPUT);
  _spi_write(address);
  _spi_write(data);

  digitalWriteFast(OF_NCS, HIGH);// de-select the chip
}


#endif



#endif