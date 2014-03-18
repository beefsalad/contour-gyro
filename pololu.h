// Controller for Pololu MinIMU-9 V2 [ 3 gyros + 3 accelerometers + 3 magnetometers ]
//             or Pololu L3GD20      [ 3 gyros + 0 accelerometers + 0 magnetometers ]
//             or Pololu L3G4200D    [ 3 gyros + 0 accelerometers + 0 magnetometers ]
//
// Units: TWI
// Ports: PORTC4 PORTC5
//
//                gyros                     accelerometers
//                -----                     --------------
// update rate:   200 Hz                    50 Hz
// bandwidth:     70 Hz                     ODR/9 = 50/9 = 5 Hz
// range:         +/- 250 deg/sec           +/- 2 gee
// sensitivity:   114 digits per deg/sec    1000 digits per gee
//               (.00875 deg/sec per digit)(.001 gee per digit)

// --------------------------------------------------------------------
// Implementation.
// --------------------------------------------------------------------

// Gyros.
//
#if   HAVE_POLOLU == 2
#define GYRO_ADDR 0x6B // L3GD20
#elif HAVE_POLOLU == 1
#define GYRO_ADDR 0x69 // L3G4200D
#else
#error  GYRO_ADDR
#endif

#define GYRO_CTRL_REG1    0x20
#define GYRO_CTRL_REG2    0x21
#define GYRO_CTRL_REG3    0x22
#define GYRO_CTRL_REG4    0x23
#define GYRO_CTRL_REG5    0x24
#define GYRO_REFERENCE    0x25
#define GYRO_OUT_TEMP     0x26
#define GYRO_STATUS_REG   0x27

#define GYRO_OUT_X_L      0x28
#define GYRO_OUT_X_H      0x29
#define GYRO_OUT_Y_L      0x2A
#define GYRO_OUT_Y_H      0x2B
#define GYRO_OUT_Z_L      0x2C
#define GYRO_OUT_Z_H      0x2D

// Accelerometers.
//
#define ACCO_ADDR 0x19 // LSM303DLHC

#define ACCO_CTRL_REG1    0x20
#define ACCO_CTRL_REG2    0x21
#define ACCO_CTRL_REG3    0x22
#define ACCO_CTRL_REG4    0x23
#define ACCO_CTRL_REG5    0x24
#define ACCO_CTRL_REG6    0x25
#define ACCO_REFERENCE    0x26
#define ACCO_STATUS_REG   0x27

#define ACCO_OUT_X_L      0x28
#define ACCO_OUT_X_H      0x29
#define ACCO_OUT_Y_L      0x2A
#define ACCO_OUT_Y_H      0x2B
#define ACCO_OUT_Z_L      0x2C
#define ACCO_OUT_Z_H      0x2D

// Read gyro sensors, mapping sensor axes to body axes such that:
//    x points ahead (body roll axis)
//    y points right (body pitch axis)
//    z points down  (body yaw axis)
//    signs respect right hand rule
//
PRIVATE void
GYRO_read_xyz(SWORD *x, SWORD *y, SWORD *z)
   {
   BYTE b[6];
   TWI_read_multi(GYRO_ADDR, GYRO_OUT_X_L | TWI_AUTO_INCREMENT, sizeof(b), b);

   *z = - (b[0] | (b[1] << 8)); // X sensor
   *x = - (b[2] | (b[3] << 8)); // Y sensor
   *y =   (b[4] | (b[5] << 8)); // Z sensor
   }

// Read accelerometer sensors, mapping sensor axes to body axes such that:
//    x points ahead (body roll axis)
//    y points right (body pitch axis)
//    z points down  (body yaw axis)
//
PRIVATE void
ACCO_read_xyz(SWORD *x, SWORD *y, SWORD *z)
   {
#if HAVE_ACCELEROMETERS
   BYTE b[6];
   TWI_read_multi(ACCO_ADDR, ACCO_OUT_X_L | TWI_AUTO_INCREMENT, sizeof(b), b);
   
   *z =   (((b[1] << 8) | b[0]) >> 4); // X sensor
   *x =   (((b[3] << 8) | b[2]) >> 4); // Y sensor
   *y = - (((b[5] << 8) | b[4]) >> 4); // Z sensor
#else
   *x = *y = *z = 0;
#endif
   }

// Sensor scale factors.
//
#define MPU_GYRO_SCALE_FACTOR DEG_TO_RAD(.00875) // radians-per-second per digit
#define MPU_ACCO_SCALE_FACTOR            .001    // gees per digit
#define MPU_ONE_GEE                      1000    // accelerometer reading corresponding to 1 gee acceleration

// --------------------------------------------------------------------
// Interface.
// --------------------------------------------------------------------

// Prepare gyros and accelerometers for use.
//
PUBLIC void
MPU_init()
   {
   
   { // gyros

   // 200 Hz data rate, 70 Hz bandwidth, power on, enable all axes
   TWI_write(GYRO_ADDR, GYRO_CTRL_REG1, 0x7F); // 0111.1111

   // 250 deg/sec scale, block (atomic) updates to LSB,MSB data pairs
   TWI_write(GYRO_ADDR, GYRO_CTRL_REG4, 0x80); // 1000.0000

   // read sensors once to restart adc after changing settings
   SWORD x, y, z;
   while ((TWI_read(GYRO_ADDR, GYRO_STATUS_REG) & 8) == 0) ;
   GYRO_read_xyz(&x, &y, &z);
   }
   
#if HAVE_ACCELEROMETERS
   { // accelerometers

   // 50 Hz data rate, power on, enable all axes
   TWI_write(ACCO_ADDR, ACCO_CTRL_REG1, 0x47); // 0100.0111
   
   // 2 gee scale, block (atomic) updates to LSB,MSB data pairs, hi resolution (bandwidth = ODR/9 = 50/9 = 5 Hz)
   TWI_write(ACCO_ADDR, ACCO_CTRL_REG4, 0x88); // 1000.1000

   // read sensors once to restart adc after changing settings
   SWORD x, y, z;
   while ((TWI_read(ACCO_ADDR, ACCO_STATUS_REG) & 8) == 0) ;
   ACCO_read_xyz(&x, &y, &z);
   }
#endif
   
   }
