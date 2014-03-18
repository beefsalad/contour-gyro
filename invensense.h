// Controller for Invensense MPU6050 [ 3 gyros + 3 accelerometers ].
//
// Units: TWI
// Ports: PORTC4 PORTC5
//
//                gyros                     accelerometers
//                -----                     --------------
// update rate:   200 Hz                    200 Hz
// bandwidth:     5 Hz                      5 Hz
// range:         +/- 250 deg/sec           +/- 2 gee
// sensitivity:   131 digits per deg/sec    16384 digits per gee

// --------------------------------------------------------------------
// Implementation.
// --------------------------------------------------------------------

// TWI address.
//
#define MPU_ADDRESS 0x68

// Registers.
//
#define MPU_SMPLRT_DIV       0x19
#define MPU_CONFIG           0x1A
#define MPU_GYRO_CONFIG      0x1B
#define MPU_ACCO_CONFIG      0x1C

#define MPU_ACCO_XOUT_H      0x3B
#define MPU_ACCO_XOUT_L      0x3C
#define MPU_ACCO_YOUT_H      0x3D
#define MPU_ACCO_YOUT_L      0x3E
#define MPU_ACCO_ZOUT_H      0x3F
#define MPU_ACCO_ZOUT_L      0x40
#define MPU_TEMP_OUT_H       0x41
#define MPU_TEMP_OUT_L       0x42
#define MPU_GYRO_XOUT_H      0x43
#define MPU_GYRO_XOUT_L      0x44
#define MPU_GYRO_YOUT_H      0x45
#define MPU_GYRO_YOUT_L      0x46
#define MPU_GYRO_ZOUT_H      0x47
#define MPU_GYRO_ZOUT_L      0x48

#define MPU_PWR_MGMT_1       0x6B
#define MPU_WHO_AM_I         0x75

// Read gyro sensors, mapping sensor axes to body axes such that:
//    x points ahead (body roll axis)
//    y points right (body pitch axis)
//    z points down  (body yaw axis)
//    signs follow right hand rule
//
PRIVATE void
GYRO_read_xyz(SWORD *x, SWORD *y, SWORD *z)
   {
   BYTE b[6];
   TWI_read_multi(MPU_ADDRESS, MPU_GYRO_XOUT_H | TWI_AUTO_INCREMENT, sizeof(b), b);

   *z = - ((b[0] << 8) | b[1]); // X sensor
   *x = - ((b[2] << 8) | b[3]); // Y sensor
   *y =   ((b[4] << 8) | b[5]); // Z sensor
   }

// Read accelerometer sensors, mapping sensor axes to body axes such that:
//    x points ahead (body roll axis)
//    y points right (body pitch axis)
//    z points down  (body yaw axis)
//
PRIVATE void
ACCO_read_xyz(SWORD *x, SWORD *y, SWORD *z)
   {
   BYTE b[6];
   TWI_read_multi(MPU_ADDRESS, MPU_ACCO_XOUT_H | TWI_AUTO_INCREMENT, sizeof(b), b);

   *z =   ((b[0] << 8) | b[1]); // X sensor
   *x =   ((b[2] << 8) | b[3]); // Y sensor
   *y = - ((b[4] << 8) | b[5]); // Z sensor
   }

// --------------------------------------------------------------------
// Interface.
// --------------------------------------------------------------------

// Filter settings.
//
//            |   ACCELEROMETER    |           GYROSCOPE
// MPU_CONFIG | Bandwidth | Delay  | Bandwidth | Delay  | Sample Rate
// -----------+-----------+--------+-----------+--------+-------------
// 0          | 260Hz     | 0ms    | 256Hz     | 0.98ms | 8 KHz
// 1          | 184Hz     | 2.0ms  | 188Hz     | 1.9ms  | 1 KHz
// 2          | 94Hz      | 3.0ms  | 98Hz      | 2.8ms  | 1 KHz
// 3          | 44Hz      | 4.9ms  | 42Hz      | 4.8ms  | 1 KHz
// 4          | 21Hz      | 8.5ms  | 20Hz      | 8.3ms  | 1 KHz
// 5          | 10Hz      | 13.8ms | 10Hz      | 13.4ms | 1 KHz
// 6          | 5Hz       | 19.0ms | 5Hz       | 18.6ms | 1 KHz

// Sensor scale factors.
//
// MPU_GYRO_CONFIG | Full scale gyro sensitivity
// ----------------+----------------------------
//     0x00        |     +/- 250 deg/sec
//     0x08        |     +/- 500 deg/sec
//     0x10        |     +/-1000 deg/sec
//     0x18        |     +/-2000 deg/sec
   
#define MPU_GYRO_SCALE_FACTOR (DEG_TO_RAD(2 * 250.0) / 65536.) // radians-per-second per digit
#define MPU_ACCO_SCALE_FACTOR (          (2 *   2.0) / 65536.) // gees per digit
#define MPU_ONE_GEE                                    16384   // accelerometer reading corresponding to 1 gee acceleration

// Prepare gyros and accelerometers for use.
//
PUBLIC void
MPU_init()
   {
   TWI_write(MPU_ADDRESS, MPU_PWR_MGMT_1, 0x80);  // device reset
   delay_ms(100);                                 // wait for reset to complete
   TWI_write(MPU_ADDRESS, MPU_PWR_MGMT_1, 0x01);  // sleep = off, clock source = x gyro
   delay_ms(5);                                   // wait for wakeup to complete

   TWI_write(MPU_ADDRESS, MPU_CONFIG,       0x01); // filter b/w  = 188 Hz, gyro output rate = 1000Hz (power on default is 256 Hz, 8000Hz) [*]
   TWI_write(MPU_ADDRESS, MPU_GYRO_CONFIG,  0x00); // gyro  scale = 250 deg/sec                       (power on default is 250 deg/sec)
   TWI_write(MPU_ADDRESS, MPU_ACCO_CONFIG,  0x00); // accel scale = 2 gee                             (power on default is 2 gee)
   TWI_write(MPU_ADDRESS, MPU_SMPLRT_DIV,   0x04); // sample rate = 200 Hz                            (power on default is 8000 Hz) [**]

   // notes:
   // [*]  when filter is off (0)   the gyro output rate is 8000Hz
   //      when filter is on  (1-6) the gyro output rate is 1000Hz
   // [**] sample rate = gyro output rate / (1 + sample rate divider)

   printf("%.1f digits per deg/sec\n", 1.0 / RAD_TO_DEG(MPU_GYRO_SCALE_FACTOR));
   }
