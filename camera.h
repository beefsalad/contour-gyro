// Camera control.
//

// --------------------------------------------------------------------
// Implementation.
// --------------------------------------------------------------------

// Alignment of camera with respect to bike, radians.
//
PRIVATE FLOAT CAMERA_roll;
PRIVATE FLOAT CAMERA_pitch;
PRIVATE FLOAT CAMERA_yaw;

// --------------------------------------------------------------------
// Interface.
// --------------------------------------------------------------------

// Align camera with respect to bike using previously saved orientation.
//
PUBLIC void
CAMERA_init()
   {
   IMU_align(CAMERA_roll, CAMERA_pitch, CAMERA_yaw);
   }
   
// Align camera with respect to bike using accelerometers (assumption: is bike level and at rest).
// This is intended for use when camera is mounted on the bike in some kind of special "non-level" attitude.
//
PUBLIC void
CAMERA_align()
   {
   FLOAT roll = 0, pitch = 0;

   // take enough readings to allow smoothing filter to do its job
   for (BYTE i = 0; i < 10; ++i)
      ACCO_getRotations(&roll, &pitch); 

   // !!TODO: provide a mode that uses gyros to track yaw motion while user swings the camera into position
   // and use this to specify yaw angle w.r.t. bike for imu alignment.
   // For now, we assume yaw angle of camera with respect to bike is zero (ie. camera is facing directly fore or aft).
   
   IMU_align(CAMERA_roll = roll, CAMERA_pitch = pitch, CAMERA_yaw = 0);
   }

// Align camera to 0,0,0.
//
PUBLIC void
CAMERA_zero()
   {
   IMU_align(CAMERA_roll = 0, CAMERA_pitch = 0, CAMERA_yaw = 0);
   }
