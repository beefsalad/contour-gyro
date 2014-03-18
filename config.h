// Persistent memory.
//

#define CONFIG_READY 1

struct
   {
   BYTE  state;            // CONFIG_READY indicates normal startup, otherwise debug startup
   FLOAT bat_k;            // battery voltage scale factor
   FLOAT center;           // servo centering adjustment
   SWORD ax, ay, az;       // accelerometer biases
   SWORD gx, gy, gz;       // gyro biases
   FLOAT roll, pitch, yaw; // camera orientation with respect to bike
   FLOAT lgain, rgain;     // servo travel volume
   BOOL  reverse;          // servo polarity with respect to camera lens and imu
   } CONFIG_Data;

void
CONFIG_init()
   {
   CONFIG_Data.state  = !CONFIG_READY;

   BATTERY_k     = CONFIG_Data.bat_k   = BATTERY_K_DEFAULT;
   SERVO_center  = CONFIG_Data.center  = 0;

   ACCO_x_bias   = CONFIG_Data.ax      = 0;
   ACCO_y_bias   = CONFIG_Data.ay      = 0;
   ACCO_z_bias   = CONFIG_Data.az      = 0;

   GYRO_x_bias   = CONFIG_Data.gx      = 0;
   GYRO_y_bias   = CONFIG_Data.gy      = 0;
   GYRO_z_bias   = CONFIG_Data.gz      = 0;

   CAMERA_roll   = CONFIG_Data.roll    = 0;
   CAMERA_pitch  = CONFIG_Data.pitch   = 0;
   CAMERA_yaw    = CONFIG_Data.yaw     = 0;

   SERVO_lgain   = CONFIG_Data.lgain   = 1;
   SERVO_rgain   = CONFIG_Data.lgain   = 1;

   SERVO_reverse = CONFIG_Data.reverse = 0;
   }

void
CONFIG_save()
   {
   CONFIG_Data.bat_k   = BATTERY_k;
   CONFIG_Data.center  = SERVO_center;

   CONFIG_Data.ax      = ACCO_x_bias;
   CONFIG_Data.ay      = ACCO_y_bias;
   CONFIG_Data.az      = ACCO_z_bias;

   CONFIG_Data.gx      = GYRO_x_bias;
   CONFIG_Data.gy      = GYRO_y_bias;
   CONFIG_Data.gz      = GYRO_z_bias;

   CONFIG_Data.roll    = CAMERA_roll;
   CONFIG_Data.pitch   = CAMERA_pitch;
   CONFIG_Data.yaw     = CAMERA_yaw;

   CONFIG_Data.lgain   = SERVO_lgain;
   CONFIG_Data.rgain   = SERVO_rgain;

   CONFIG_Data.reverse = SERVO_reverse;

   EEPROM_write_block(0, &CONFIG_Data, sizeof(CONFIG_Data));
   }
   
void
CONFIG_recall()
   {
   EEPROM_read_block(0, &CONFIG_Data, sizeof(CONFIG_Data));
   
   BATTERY_k     = CONFIG_Data.bat_k;
   SERVO_center  = CONFIG_Data.center;

   ACCO_x_bias   = CONFIG_Data.ax;
   ACCO_y_bias   = CONFIG_Data.ay;
   ACCO_z_bias   = CONFIG_Data.az;

   GYRO_x_bias   = CONFIG_Data.gx;
   GYRO_y_bias   = CONFIG_Data.gy;
   GYRO_z_bias   = CONFIG_Data.gz;

   CAMERA_roll   = CONFIG_Data.roll;
   CAMERA_pitch  = CONFIG_Data.pitch;
   CAMERA_yaw    = CONFIG_Data.yaw;

   SERVO_lgain   = CONFIG_Data.lgain;
   SERVO_rgain   = CONFIG_Data.rgain;

   SERVO_reverse = CONFIG_Data.reverse;
   }
