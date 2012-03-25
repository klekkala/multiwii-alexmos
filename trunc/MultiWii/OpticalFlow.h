#ifdef OPTFLOW

/* displacment (in mm*10 on height 1m) */
static int16_t optflow_pos[2] = { 0, 0 };

/* surface quality, (0..255) */
static uint8_t optflow_squal = 0;

/* Optical Flow mode flag */
static int8_t optflowMode = OF_MODE;


/* Estimated horisontal velocity from ACC + OptFlow sensor, cm/sec */
/* Value constrained -100, 100 */
static int16_t EstHVel[2] = { 0, 0 };  

/* Estimated horisontal acceleration (in ACC sensor's units) */
static int16_t EstHAcc[2] = { 0, 0 };

#endif