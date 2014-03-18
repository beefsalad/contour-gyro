// Track a motorcycle's orientation using "strap down" gyros (gyros that are fixed to the bike's frame of reference and rotate with it).
// Most of the algorithms here were developed by Bill Premerlani. The motorcycle-specific drift correction ideas are mine.
// 09 Nov 2012 Derek Lieber
//
// References:
// [Art 1] "Direction Cosine Matrix IMU Theory"                            http://gentlenav.googlecode.com/files/DCMDraft2.pdf
// [Art 2] "Computing Euler Angles From Direction Cosines"                 http://gentlenav.googlecode.com/files/EulerAngles.pdf
// [Art 3] "A Sensor Fusion Method for Smart phone Orientation Estimation" http://www.cms.livjm.ac.uk/pgnet2012/Proceedings/Papers/1569603133.pdf
// [Art 4] "Tilt Sensing Using Linear Accelerometers"                      http://cache.freescale.com/files/sensors/doc/app_note/AN3461.pdf
//
// We use "aerospace convention" for rotations throughout:
// - x is roll  axis, points ahead, positive rotation == roll right
// - y is pitch axis, points right, positive rotation == pitch up
// - z is yaw   axis, points down,  positive rotation == yaw right
// The orientation of the gyros with respect to the ground is defined in "zyx" order: first apply yaw, then pitch, then roll.
// Ref: [Art 2, Fig 2]
//

// Drift correction tuning constants.
//
#define IMU_RATE_THRESHOLD  DEG_TO_RAD(1.0) // drift correction snapshots are taken whenever turn rate is lower than this, in radians/sec
#define IMU_RATE_DURATION   0.040           // ...for at least this long, in seconds
#define IMU_TIME_CONSTANT   0.5             // time constant characterizing speed with which drift corrections are applied, in seconds
   
// --------------------------------------------------------------------
// Interrupt communication area.
//                                                                                                                                                
// Current orientation of gyros and ground with respect each other, as a rotation matrix.
// Columns are projections of gyro xyz axes on ground xyz axes.
// Rows    are projections of ground xyz axes on gyro xyz axes.
//                                                                                                                                                
PRIVATE volatile FLOAT
   Rxx, Rxy, Rxz,                                                                                                                                 
   Ryx, Ryy, Ryz,                                                                                                                                 
   Rzx, Rzy, Rzz;                                                                                                                                 
                                                                                                                                                  
// Reference angles identifying "home" orientation, in radians.
// The drift corrector will drive the rotation matrix towards this orientation whenever the bike is upright.
//                                                                                                                                                
PRIVATE volatile FLOAT IMU_rollReference, IMU_pitchReference;

// Estimated drift error awaiting correction, in radians.
//                                                                                                                                                
PRIVATE volatile FLOAT IMU_rollError, IMU_pitchError, IMU_yawError;
                                                                                                                                                  
// Have all the above values been set (ie. by IMU_align)?                                                                                         
//                                                                                                                                                
PRIVATE volatile BOOL IMU_aligned;
                                                                                                                                                  
// Apply drift correction?
//                                                                                                                                                
PUBLIC  volatile BOOL IMU_apply_dc  = 1; 
// --------------------------------------------------------------------

// Initialize the orientation matrix.
// Taken:   gyro's orientation with respect to ground, in radians
// Updated: Rxx..Rzz
// Ref: [Art 2, Eqn 2] and [http://en.wikipedia.org/wiki/Rotation_matrix]
//
PRIVATE void
IMU_set(FLOAT roll, FLOAT pitch, FLOAT yaw)
   {
   FLOAT
  
   R = roll,
   P = pitch,
   Y = yaw,
   
   sinR = sin(R), cosR = cos(R),
   sinP = sin(P), cosP = cos(P),
   sinY = sin(Y), cosY = cos(Y);

   // The full rotation matrix, R, is formed by applying individual axis rotations in yaw, pitch, roll order.
   // We do this by premultiplying, right to left:
   //    R = Rr * Rp * Ry
   // Where:
   //         1    0     0          cosR  0 sinR          cosY -sinY 0
   //    Rr = 0 cosP -sinP     Rp =    0  1    0    Ry =  sinY  cosY 0
   //         0 sinP  cosP         -sinR  0 cosR             0    0  1

   DI();
   
   Rxx = cosP * cosY; Rxy = sinR * sinP * cosY - cosR * sinY; Rxz = cosR * sinP * cosY + sinR * sinY;
   Ryx = cosP * sinY; Ryy = sinY * sinP * sinY + cosR * cosY; Ryz = cosR * sinP * sinY - sinR * cosY;
   Rzx = -sinP;       Rzy = sinR * cosP;                      Rzz = cosR * cosP;

   IMU_rollReference  = roll;
   IMU_pitchReference = pitch;
   
   IMU_rollError  = 0;
   IMU_pitchError = 0;
   IMU_yawError   = 0;
   
   EI();
   }

// Get the integrator-estimated orientation of the gyros with respect to ground, in radians.
// Ref: [Art 2, Eqn 3]
//
PUBLIC FLOAT
IMU_getRollAngle()
   { 
   DI();
   FLOAT a = Rzy;
   FLOAT b = Rzz;
   EI();
   return atan2(a, b);
   }

PUBLIC FLOAT
IMU_getPitchAngle()
   {
   DI();
   FLOAT a = Rzx;
   EI();
   return -asin(a);
   } 

PUBLIC FLOAT
IMU_getYawAngle()
   { 
   DI();
   FLOAT a = Ryx;
   FLOAT b = Rxx;
   EI();
   return atan2(a, b);
   }

// Align orientation matrix and gyros with respect to each other and with respect to ground reference.
// Taken: gyro orientation with respect to ground (as determined by accelerometers, for example), in radians
// Note: the roll and pitch angles will be the "home" orientation used by the drift corrector.
//
PUBLIC void
IMU_align(FLOAT roll, FLOAT pitch, FLOAT yaw)
   {
   IMU_aligned = 0;
   IMU_set(roll, pitch, yaw);
   IMU_aligned = 1;
   printf("imu=(%+.1f %+.1f %+.1f)\n", RAD_TO_DEG(roll), RAD_TO_DEG(pitch), RAD_TO_DEG(yaw));
   }

// Rotate orientation matrix to follow gyro's motion.
// Called by interrupt.
//
// Taken:   differential rotations of gyro around its axes, in radians
// Updated: Rxx..Rzz
//
// This algorithm is neat because it contains no divisions, trig functions, or square roots.
//
// I'm somewhat puzzled about the dimensions in use here.
// We seem to be intermixing dimensionless units (the matrix elements) with angular units (the applied differential rotations, in radians).
// I'm not sure why this all works out. I guess it's all a bunch of approximations that get cleaned up by the re-orthonormalization step.
//
PRIVATE void
IMU_rotate(FLOAT dx, FLOAT dy, FLOAT dz)
   {
   // The update formula is:   
   //
   //    R(t + dt) = R(t) * Q    
   //
   // where:
   //            Rxx Rxy Rxz
   //       R =  Ryx Ryy Ryz
   //            Rzx Rzy Rzz
   // and:
   //             1 -dz  dy
   //       Q  =  dz  1 -dx
   //            -dy dx   1
   //
   // R is orientation matrix.
   // Q is differential rotation measured in gyro frame of reference.
   //
   // dx = rotation about gyro roll  axis (points ahead), positive = roll right
   // dy = rotation about gyro pitch axis (points right), positive = pitch up
   // dz = rotation about gyro yaw   axis (points down),  positive = yaw right
   //
   // [Art 1, Eqn 17]
   //

   FLOAT
   Qxx =   1,  Qxy = -dz,  Qxz =  dy,
   Qyx =  dz,  Qyy =   1,  Qyz = -dx,
   Qzx = -dy,  Qzy =  dx,  Qzz =   1;
   
   FLOAT
   Txx = Rxx * Qxx + Rxy * Qyx + Rxz * Qzx,  Txy = Rxx * Qxy + Rxy * Qyy + Rxz * Qzy,  Txz = Rxx * Qxz + Rxy * Qyz + Rxz * Qzz,
   Tyx = Ryx * Qxx + Ryy * Qyx + Ryz * Qzx,  Tyy = Ryx * Qxy + Ryy * Qyy + Ryz * Qzy,  Tyz = Ryx * Qxz + Ryy * Qyz + Ryz * Qzz,
   Tzx = Rzx * Qxx + Rzy * Qyx + Rzz * Qzx,  Tzy = Rzx * Qxy + Rzy * Qyy + Rzz * Qzy,  Tzz = Rzx * Qxz + Rzy * Qyz + Rzz * Qzz;

   Rxx = Txx;  Rxy = Txy;  Rxz = Txz;
   Ryx = Tyx;  Ryy = Tyy;  Ryz = Tyz;
   Rzx = Tzx;  Rzy = Tzy;  Rzz = Tzz;

   // Re-orthonormalize the matrix with the following objectives:
   // The dot product of the X & Y rows should be zero.
   // The Z row should be equal to the cross product of the X & Y rows.
   // Each row should be of unit magnitude.
   //
   // The idea is that the three rows and columns will always be approximately perpendicular, because we are going to maintain them that
   // way, and we are going to maintain their lengths to be one, but we will need to fix up slight rotational errors.
   // For example, suppose vectors A and B are almost, but not exactly perpendicular, and we want to adjust them to make them closer to perpendicular.
   // We do not want to change their magnitude, we just want to rotate them. That means the adjustment to each of them is perpendicular.
   // Since B is perpendicular to A, when we want to rotate A a little bit, we simply add a portion of B. And vice-versa when we want to rotate B.
   //
   // So we take the dot product of the X and Y rows to find out if they are perpendicular. If they are, the dot product will be zero.
   // If they are not, the dot product will be measure of how much they need to be rotated toward or away from each other to be perpendicular.
   // Since we have no way of knowing whether X or Y are more likely to be correct, we split the difference, and adjust both X and Y by half.
   
   // Measure how much rows X and Y are rotated towards each other [Art 1, Eqn 18].
   //
   FLOAT dot = Rxx * Ryx + Rxy * Ryy + Rxz * Ryz;
   
   // Now rotate each away from the other by half that amount, thereby restoring their orthogonality [Art 1, Eqn 19].
   //
   FLOAT half = .5 * dot;

   // temporary copy of row X
   Txx =  Rxx;
   Txy =  Rxy;
   Txz =  Rxz;

   // rotate row X away from Y
   Rxx -= half * Ryx;
   Rxy -= half * Ryy;
   Rxz -= half * Ryz;

   // rotate row Y away from X
   Ryx -= half * Txx;
   Ryy -= half * Txy;
   Ryz -= half * Txz;

   // Set row Z to cross product of X and Y rows [Art 1, Eqn 20].
   //
   Rzx = Rxy * Ryz - Rxz * Ryy;
   Rzy = Rxz * Ryx - Rxx * Ryz;
   Rzz = Rxx * Ryy - Rxy * Ryx;

   // Normalize each row by dividing each element by the row's magnitude (square root of sum of squares)
   // noting that, since the magnitude should be approximately 1, we can use a short Taylor expansion
   // to compute the reciprocal square root:    1/sqrt(x) ~=  1/2 * (3 - x) for x ~= 1
   //
   FLOAT nx = .5 * (3 - (Rxx * Rxx + Rxy * Rxy + Rxz * Rxz));
   FLOAT ny = .5 * (3 - (Ryx * Ryx + Ryy * Ryy + Ryz * Ryz));
   FLOAT nz = .5 * (3 - (Rzx * Rzx + Rzy * Rzy + Rzz * Rzz));

   Rxx *= nx; Rxy *= nx; Rxz *= nx;
   Ryx *= ny; Ryy *= ny; Ryz *= ny;
   Rzx *= nz; Rzy *= nz; Rzz *= nz;
   }

// Update orientation matrix in step with gyro's motions and apply drift corrections.
// Called by interrupt at a rate of IMU_HZ.
//
PRIVATE void
IMU_update()
   {
   if (!IMU_aligned)
      return;

   // 1. Estimate how much the gyro has rotated during this timestep.
   // Note that these rotations are with respect to the gyro reference frame, not the ground.
   //
   FLOAT rollDelta, pitchDelta, yawDelta;
   GYRO_getRotations(&rollDelta, &pitchDelta, &yawDelta);

   // 2. Estimate a correction that will counteract any errors that have accumulated in the orientation matrix...
   //
                
   // Guess the bike's stance by examining yaw rate:
   //    yaw-rate==0 roll-rate!=0 --> bike is upright, at an inflection point in a transition from left-to-right or right-to-left turn
   //    yaw-rate==0 roll-rate==0 --> bike is upright, on a straightaway
   // Note that yaw and roll rates provide no way to guess the bike's pitch orientation.
   //
   BOOL upright = fabs(GYRO_getYawRate()) <= IMU_RATE_THRESHOLD;
   
   // Avoid false indications caused by noise or vibration - stance must persist before it's considered valid.
   //
   static FLOAT duration;
   if (upright)
      {
      duration += 1.0 / IMU_HZ;
      if (duration < IMU_RATE_DURATION) 
         upright = 0; // rate hasn't persisted long enough yet
      }
   else
      duration = 0;   // rate isn't low enough yet
   
   if (upright)
      {
      // Take a drift error snapshot.
      //
      // We assume the bike is momentarily level and its gyros have momentarily returned to their original pitch/roll orientation with respect
      // to ground. If the imu has been tracking correctly, its rotation matrix should also now be (nearly) identical to that original orientation.
      // Any differences are due to drift and, because the differences should be small, we can use small angle approximations to measure them.
      // Furthermore, since the gyro and drift correction reference frames are (nearly) aligned, we can use simple addition to combine gyro 
      // rotations and drift corrections into a single rotation update.
      //
      // Caveats: cross-axis effects will be introduced if current pitch angle != original pitch angle (for example bike is now on a hill)
      // or if current roll angle (ie. zero) != original roll angle (for example bike was originally on its sidestand).

      IMU_rollError  =  Rzy - IMU_rollReference;  // imu-calculated roll  angle should match initial reference, any difference is an error
      IMU_pitchError = -Rzx - IMU_pitchReference; // imu-calculated pitch angle should match initial reference, any difference is an error
      IMU_yawError   =  0; // we have no compass, thus nothing to compare with imu-calculated yaw angle, so assume no error
      }

#if 0  // 1 = road testing, 0 = normal field use
   // Update stance indicator as visual cue.
   //
   LED_set(upright);
#endif

   // Apply error correction at rate specified by desired time constant.
   //
   const FLOAT T = IMU_TIME_CONSTANT; // time constant, in seconds
   const FLOAT C = IMU_HZ;            // number of corrections to apply per second
   const FLOAT K = 1.0 / T / C;       // fraction to apply per correction

   FLOAT rollCorr  = - K * IMU_rollError;
   FLOAT pitchCorr = - K * IMU_pitchError;
   FLOAT yawCorr   = - K * IMU_yawError;

   // Save remaining error. We'll correct more of it at next timestep (unless overwritten by another estimate).
   //
   IMU_rollError  += rollCorr;
   IMU_pitchError += pitchCorr;
   IMU_yawError   += yawCorr;
   
   // Caveat: The way we're implementing this scheme to "snapshot" an error and then spread out the correction over multiple timesteps is not quite
   // correct in all cases. In particular, if the bike enters a turn while we're still applying residual corrections (that were measured in a 
   // previous reference frame) they'll be incorrect for the new, now rotated, reference frame. This will lead to various cross-axis
   // effects in the camera tracking.
   //
   // Cases involving a single-axis error and subsequent rotation about that same axis during the correction period will work fine, but the
   // right way to handle general multi-axis corrections would be to transform them back and forth between ground- and gyro- reference
   // frames at each timestep. In practice, however, this is unnecessary unless the bike were to climb or descend while rolling into a turn.
   //
   // For now, we'll simply try to minimize such errors by keeping the correction period short (small IMU_TIME_CONSTANT),
   // but still long enough that the camera smoothly blends in corrections without abrupt movements.

   // Suppress drift correction while debugging camera motions.
   //
   if (!IMU_apply_dc) rollCorr = pitchCorr = yawCorr = 0;

   // Apply gyro rotations and drift corrections to orientation matrix (small angles assumed).
   //
   IMU_rotate(rollDelta + rollCorr, pitchDelta + pitchCorr, yawDelta + yawCorr);
   }

// --------------------------------------------------------------------------------------------------------------------------------------------------
// Side note - calculating rotation angles between two sets of orthonormal coordinate axes using small angle approximations.
// --------------------------------------------------------------------------------------------------------------------------------------------------
//
// The following expressions are exact but computationally expensive, requiring use of atan, asin, and sqrt functions to extract imu angles:
//    roll angle difference (about x-axis) = imuRollAngle  - trueRollAngle
//   pitch angle difference (about y-axis) = imuPitchAngle - truePitchAngle
//     yaw angle difference (about z-axis) = imuYawAngle   - trueYawAngle
//
// However, if the two sets of coordinate axes are nearly aligned, the following small angle approximations are cheaper and nearly as accurate:
//    roll angle difference (about x-axis), in radians ~= difference in y components
//   pitch angle difference (about y-axis), in radians ~= difference in x components
//     yaw angle difference (about z-axis), in radians ~= difference in z components
//
// That is:
//    roll angle difference (about x-axis), in radians ~=  (Rzy - trueY)
//   pitch angle difference (about y-axis), in radians ~= -(Rzx - trueX)
//     yaw angle difference (about z-axis), in radians ~=  (Ryx - trueZ)
//
// If the bike is in its "home" orientation, then the true components are zero, and this redues to:
//    roll angle difference (about x-axis), in radians ~=   Rzy
//   pitch angle difference (about y-axis), in radians ~= - Rzx
//     yaw angle difference (about z-axis), in radians ~=   Ryx
//
// Even if the small angle assumption is violated, we can use these approximations to get a general sense of the differences in order to
// begin applying corrections.
//
// --------------------------------------------------------------------------------------------------------------------------------------------------
// Side note - time constant for slowly applied corrections.
// --------------------------------------------------------------------------------------------------------------------------------------------------
//
// Example:   T = 5 seconds
//            C = 50 corrections per second
//            K = 1/T/C = .004 = .4% per correction
// Check:     50 corrections per second * .4% per correction * 5 seconds = 100%
//            (Actually, the time constant tells how long to reach 1/e (=36%) of the initial value, not 100%).
//
// --------------------------------------------------------------------------------------------------------------------------------------------------
// Side note - cross-axis coupling due to imu misalignment.
// --------------------------------------------------------------------------------------------------------------------------------------------------
//
// Misalignment between the matrix axes, those of the bike, and those of the gyros (either due to setup errors or to erroneously applied drift
// corrections) will produce puzzling camera effects. When the axes are properly aligned, the camera will turn in response to roll motions only.
// However, when the axes are misaligned, the camera will begin to turn also in response to yaw. For example, if the bike is executing a constant
// radius turn, the camera will keep rotating even though the roll rate is zero. This will continue through 180 degrees of turn and then reverse
// direction during the second 180 degrees. Or if the bike is executing a sequence of left/right turns, the camera will overcorrect first one way, 
// then the other, generating a peculiar rocking motion in the image. The only way to avoid this kind of behavior is to ensure that the angles
// used to initialize the matrix and to apply subsequent drift corrections are chosen reliably. For example, if accelerometers are used to determine
// orientation angles, then centrifugal accelerations must be accounted for (or the bike must be in a state of motion where centrifugal accelerations
// are zero, ie. either at rest or at constant speed on a straightaway). Accelerometer noise due to engine vibration can also introduce unwanted 
// cross-axis coupling effects. This can be mitigated by choosing enough smoothing to reduce jitter but not so much that we introduce excessive lag 
// in the angle readouts. Ultimately, I decided that using accelerometers for drift correction  was impossible due to their vibration sensitivity.
// Using purely gyro-based heuristics worked out much better.
//
// --------------------------------------------------------------------------------------------------------------------------------------------------
