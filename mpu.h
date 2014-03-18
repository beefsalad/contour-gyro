// Controller for gyros and accelerometers.
//

// --------------------------------------------------------------------
// Implementation.
// --------------------------------------------------------------------

#if HAVE_POLOLU
#include "./pololu.h"
#else
#include "./invensense.h"
#endif

// --------------------------------------------------------------------
// Interrupt communication area.
//
PRIVATE volatile SWORD  ACCO_x_bias;  // accelerometer zero-rate bias
PRIVATE volatile SWORD  ACCO_y_bias;  // "
PRIVATE volatile SWORD  ACCO_z_bias;  // "

PRIVATE volatile SWORD  GYRO_x_bias;  // gyro zero-rate bias
PRIVATE volatile SWORD  GYRO_y_bias;  // "
PRIVATE volatile SWORD  GYRO_z_bias;  // "

PRIVATE volatile SWORD  GYRO_x_urate; // gyro rate, bias corrected, unsmoothed
PRIVATE volatile SWORD  GYRO_y_urate; // "
PRIVATE volatile SWORD  GYRO_z_urate; // "

PRIVATE volatile SWORD  GYRO_x_srate; // gyro rate, bias corrected, smoothed
PRIVATE volatile SWORD  GYRO_y_srate; // "
PRIVATE volatile SWORD  GYRO_z_srate; // "

PRIVATE volatile BOOL   MPU_calibrating;
PRIVATE volatile SDWORD GYRO_x_sum;   // calibration data
PRIVATE volatile SDWORD GYRO_y_sum;   // "
PRIVATE volatile SDWORD GYRO_z_sum;   // "
PRIVATE volatile SDWORD ACCO_x_sum;   // "
PRIVATE volatile SDWORD ACCO_y_sum;   // "
PRIVATE volatile SDWORD ACCO_z_sum;   // "
PRIVATE volatile SWORD  MPU_cnt;      // "
// --------------------------------------------------------------------

// Update mpu data.
// Called by interrupt.
//
PRIVATE void
MPU_update()
   {
   // accumulate data for zero rate bias calibration
   //
   if (MPU_calibrating)
      {
      SWORD x, y, z;
      GYRO_read_xyz(&x, &y, &z); GYRO_x_sum += x; GYRO_y_sum += y; GYRO_z_sum += z;
      ACCO_read_xyz(&x, &y, &z); ACCO_x_sum += x; ACCO_y_sum += y; ACCO_z_sum += z;
      MPU_cnt += 1;
      return;
      }

   // raw sensor readings (MPU has fresh gyro data available at update rate of 1 KHz)
   //
   SWORD x, y, z;
   GYRO_read_xyz(&x, &y, &z);

   // remove zero rate biases
   //
   x -= GYRO_x_bias;
   y -= GYRO_y_bias;
   z -= GYRO_z_bias;

   // unsmoothed rates, for integrator
   //
   GYRO_x_urate = x;
   GYRO_y_urate = y;
   GYRO_z_urate = z;

   // smoothed rates, for general use
   // K = low pass filter strength (0=none, 1=weak, 4+=strong)
   //
   const BYTE K = 3;

   static SDWORD x_filter, y_filter, z_filter;

   x_filter = x_filter - (x_filter >> K) + x;
   y_filter = y_filter - (y_filter >> K) + y;
   z_filter = z_filter - (z_filter >> K) + z;

   GYRO_x_srate = x_filter >> K;
   GYRO_y_srate = y_filter >> K;
   GYRO_z_srate = z_filter >> K;
   }

// Fast blink led for N seconds during calibration.
//
PRIVATE void
MPU_blink(BYTE n)
   {
   LED_off();
   
   BYTE blinks = n * 8;
   for (BYTE i = 0; i < blinks; ++i)
      {
      TIME_pause(.125);
      LED_toggle();
      }
   
   LED_on();
   }

// --------------------------------------------------------------------
// Interface.
// --------------------------------------------------------------------

// Read accelerometers for a few seconds and compute biases needed to zero the output rates.
// Assumption: device is level, upright, and motionless
//
PUBLIC void
ACCO_calibrate()
   {
   // accumulate data
   //
   ACCO_x_sum = ACCO_y_sum = ACCO_z_sum = 0;
   MPU_cnt = 0;

   MPU_calibrating = 1;
   MPU_blink(5);
   MPU_calibrating = 0;

   // calculate biases
   //
   ACCO_x_bias = ACCO_x_sum / MPU_cnt;
   ACCO_y_bias = ACCO_y_sum / MPU_cnt;
   ACCO_z_bias = ACCO_z_sum / MPU_cnt;
   
   ACCO_z_bias -= MPU_ONE_GEE;
   
   printf("acco: cnt=%u bias=(%+d %+d %+d)\n", MPU_cnt, ACCO_x_bias, ACCO_y_bias, ACCO_z_bias);
   }

// Read gyros for a few seconds and compute biases needed to zero the output rates.
// Assumption: device is motionless
//
PUBLIC void
GYRO_calibrate()
   {
   // accumulate data
   //
   GYRO_x_sum = GYRO_y_sum = GYRO_z_sum = 0;
   MPU_cnt = 0;

   MPU_calibrating = 1;
   MPU_blink(5);
   MPU_calibrating = 0;
   
   // calculate biases
   //
   GYRO_x_bias = GYRO_x_sum / MPU_cnt;
   GYRO_y_bias = GYRO_y_sum / MPU_cnt;
   GYRO_z_bias = GYRO_z_sum / MPU_cnt;
   
   printf("gyro: cnt=%u bias=(%+d %+d %+d)\n", MPU_cnt, GYRO_x_bias, GYRO_y_bias, GYRO_z_bias);
   }

// Get direction of accelerometer vector with respect to ground reference frame, in radians.
// Note: these angles are only meaningful when the sensor is standing still or moving in a straight line at constant speed.
//
PUBLIC void
ACCO_getRotations(FLOAT *xp, FLOAT *yp)
   {
#if HAVE_ACCELEROMETERS
   DI();
   SWORD x, y, z;
   ACCO_read_xyz(&x, &y, &z);
   EI();
   
   // apply some smoothing
   // K = low pass filter strength (0=none, 1=weak, 4+=strong)
   //
   const BYTE K = 1;

   static SDWORD x_filter, y_filter, z_filter;

   x_filter = x_filter - (x_filter >> K) + x;
   y_filter = y_filter - (y_filter >> K) + y;
   z_filter = z_filter - (z_filter >> K) + z;

   x = x_filter >> K;
   y = y_filter >> K;
   z = z_filter >> K;

   x -= ACCO_x_bias;
   y -= ACCO_y_bias;
   z -= ACCO_z_bias;
    
   FLOAT ax = x * MPU_ACCO_SCALE_FACTOR,
         ay = y * MPU_ACCO_SCALE_FACTOR,
         az = z * MPU_ACCO_SCALE_FACTOR;

   *xp = atan2(ay, az);                       // roll
   *yp = atan2(-ax, sqrt(ay * ay + az * az)); // pitch
#else
   *xp = *yp = 0;
#endif
   }

// Calculate how far gyros have turned during current imu timestep, in radians.
// Note: we use unsmoothed rates to minimize imu lag (any jitter will get averaged out by imu integrator).
//
PUBLIC void
GYRO_getRotations(FLOAT *xp, FLOAT *yp, FLOAT *zp)
   {
   DI();
   SWORD x = GYRO_x_urate,
         y = GYRO_y_urate,
         z = GYRO_z_urate;
   EI();

   #define IMU_TIMESTEP (1.0 / IMU_HZ)
   *xp = x * MPU_GYRO_SCALE_FACTOR * IMU_TIMESTEP; // roll
   *yp = y * MPU_GYRO_SCALE_FACTOR * IMU_TIMESTEP; // pitch
   *zp = z * MPU_GYRO_SCALE_FACTOR * IMU_TIMESTEP; // yaw
   }

// Get (smoothed) gyro rates, in radians/sec.
//

#if 0 // UNUSED
PUBLIC FLOAT
GYRO_getRollRate()
   {
   DI();
   SWORD x = GYRO_x_srate;
   EI();
   return x * MPU_GYRO_SCALE_FACTOR;
   }

PUBLIC FLOAT
GYRO_getPitchRate()
   {
   DI();
   SWORD y = GYRO_y_srate;
   EI();
   return y * MPU_GYRO_SCALE_FACTOR;
   }
#endif

PUBLIC FLOAT
GYRO_getYawRate()
   {
   DI();
   SWORD z = GYRO_z_srate;
   EI();
   return z * MPU_GYRO_SCALE_FACTOR;
   }
