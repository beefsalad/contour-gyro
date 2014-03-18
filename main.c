// A horizon-stabilized camera for motorcycle photography.
// 14 Apr 2013 Derek Lieber
//
// Camera:
//      GoPro Hero mounted to directly to servo
// or   Contour Roam with gear driven lens barrel
//
// Microcontroller:
//      Atmel ATMEGA328P running at 16 MHz
//  or                               8 MHz
//
// Drive:
//      Hitec HS-425BB servo
//
// Sensors:
//      Invensense MPU6050
// or   Pololu MinIMU-9 V2
// or   Pololu L3GD20
// or   Pololu L3G4200D

//----------------------------------------------------------------------
// Atmega328 pin and port allocations.
//----------------------------------------------------------------------
//
//                        AREF = pin 21
//                        AVCC = pin 20 <-  5v
//                         VCC = pin  7 <-  5v
//                        AGND = pin 22 <-  0v
//                         GND = pin  8 <-  0v
//                      #RESET = pin  1 <-  10k pullup
//                                      
// [ICP1][PCINT0][CLKO] PORTB0 = pin 14 <-  [pin] pushbutton
// [OC1A][PCINT1]       PORTB1 = pin 15 ->  [pwm] servo
// [OC1B][PCINT2] [#SS ]PORTB2 = pin 16 
// [OC2A][PCINT3] [MOSI]PORTB3 = pin 17 
//       [PCINT4] [MISO]PORTB4 = pin 18 
//       [PCINT5] [SCK ]PORTB5 = pin 19 
//       [PCINT6][XTAL1]PORTB6 = pin  9 <-> [osc]
//       [PCINT7][XTAL2]PORTB7 = pin 10 <-> [osc]
//
//       [PCINT8] [ADC0]PORTC0 = pin 23 ->  [pout] status led
//       [PCINT9] [ADC1]PORTC1 = pin 24 <-  [adc]  battery voltage divider
//       [PCINT10][ADC2]PORTC2 = pin 25 ->  [pout] power switch
//       [PCINT11][ADC3]PORTC3 = pin 26 
// [SDA] [PCINT12][ADC4]PORTC4 = pin 27 <-> [twi]  mpu SDA
// [SCL] [PCINT13][ADC5]PORTC5 = pin 28 ->  [twi]  mpu SCL
//                      PORTC6 = n/a
//                      PORTC7 = n/a
//
//       [PCINT16] [RXD]PORTD0 = pin  2 <-  [usart] serial port
//       [PCINT17] [TXD]PORTD1 = pin  3 ->  [usart] serial port
//       [PCINT18][INT0]PORTD2 = pin  4
// [OC2B][PCINT19][INT1]PORTD3 = pin  5
// [XCK] [PCINT20]  [T0]PORTD4 = pin  6
// [OC0B][PCINT21]  [T1]PORTD5 = pin 11
// [OC0A][PCINT22][AIN0]PORTD6 = pin 12
//       [PCINT23][AIN1]PORTD7 = pin 13 
//
// Fuse settings:
//
//  lfuse=d7 => external crystal    at 16MHz with brownout boot control
//  lfuse=e6 => external resonator  at 16MHz with brownout boot control
//  lfuse=c2 => internal oscillator at  8MHz with brownout boot control
//  hfuse=dc => my bootloader, spi programming enabled
//  efuse=04 => 4.3V brownout detection threshold
//
//----------------------------------------------------------------------

// Feature selections.
//
#ifndef HAVE_POLOLU                   // 2 => Pololu L3GD20 or MinIMU
#error  HAVE_POLOLU                   // 1 => Pololu L3G4200D
#endif                                // 0 => Invensense MPU6050

#ifndef HAVE_ACCELEROMETERS           // 1 => MinIMU or MPU6050
#error  HAVE_ACCELEROMETERS           // 0 => L3GD20
#endif

#ifndef HAVE_CLOCK                    // 16 => external oscillator at 16 MHz
#error  HAVE_CLOCK                    //  8 => internal oscillator at 8 MHz
#endif

// Clock rates.
//
#define CLOCK_MHZ      HAVE_CLOCK     // system clock rate (8 or 16 MHz)
#define TWI_KHZ        200            // twi clock rate
#define IMU_HZ         250            // imu update rate           (should be >= mpu sample rate)
#if  CLOCK_MHZ == 8                   // timer tick interrupt rate (should be >= imu update rate, but see discussion in ticker.h)
#define TICKER_HZ      500            // "
#elif CLOCK_MHZ == 16                 // "
#define TICKER_HZ     1000            // "
#endif                                // "

// Includes.
//
#include <avr/io.h>                   // avr architecture - see /usr/lib/avr/include/avr/iomx8.h
#include <avr/boot.h>                 // avr fuse and lock bits
#include <avr/interrupt.h>            // avr interrupt helpers - ISR, sei, cli

#include "./include/types.h"      // BOOL, BYTE, WORD, DWORD, FLOAT
#include "./include/atomic.h"     // EI DI
#include "./include/system.h"     // standard startup
#include "./include/led.h"        // status led
#include "./include/usart.h"      // serial i/o via usart
#include "./include/stdout.h"     // stdout via usart
#include "./include/delay.h"      // delay_ms
#include "./include/reboot.h"     // processor reboot
#include "./include/bootloader.h" // LOADER_REQUEST_REBOOT
#include "./include/twi.h"        // two-wire interface
#include "./include/eeprom.h"     // persistent memory
#include "./include/stack.h"      // stack checker

#include <math.h>                                 // trig
#define RAD_TO_DEG(X) ((X) * 57.2957795130823229) // radians to degrees
#define DEG_TO_RAD(X) ((X) *  0.0174532925199433) // degrees to radians

typedef DWORD TICKS;                  // an interval measured by timer interrupt (spans 2^32 ticks = 50 days @ 1000Hz)

#include "./version.h"                // date of issue
#include "./counter.h"                // cycle counting functions   [uses TIMER2 for cycle counting]
#include "./time.h"                   // timing functions
#include "./power.h"                  // power control
#include "./battery.h"                // battery monitor
#include "./mpu.h"                    // gyros and accelerometers
#include "./imu.h"                    // orientation tracker
#include "./camera.h"                 // camera tracker
#include "./button.h"                 // push button
#include "./servo.h"                  // camera drive               [uses TIMER1 for pwm]
#include "./ticker.h"                 // background task dispatcher [uses TIMER0 for timer tick interrupt generator]
#include "./config.h"                 // board personality

// Calibrate battery monitor (set by comparing indicated reading to value measurd by external voltmeter).
//
void
adjust_battery()
   {
   for (;;)
      {
      while (!USART_ready())
         {
         FLOAT volts = BATTERY_read();
         FLOAT pct   = (volts - 7.2) / (8.4 - 7.2) * 100; // 2s lipo is 7.2V to 8.4V (curve is not really linear, this is just an approximation)
         printf("\r%4.2fV %3.0f%% k=%5.5f ", volts, pct, BATTERY_k);
         }
      switch (USART_get())
         {
         case '=':
         case '+': BATTERY_k += .00001;             break; // raise indicated voltage
         case '-': BATTERY_k -= .00001;             break; // lower indicated voltage
         
         case 'q': goto done;                       break;
         default:  printf("?\n");                   break;
         }
      }
   done:
   printf("\n");
   }

// Calibrate accelerometers (assumption: device upright, level, and motionless).
//
void
adjust_accelerometers()
   {
   BYTE how = 0;
   for (;;)
      {
      while (!USART_ready())
         {
         DI();
         SWORD x, y, z;
         ACCO_read_xyz(&x, &y, &z);
         EI();
         x -= ACCO_x_bias;
         y -= ACCO_y_bias;
         z -= ACCO_z_bias;
         if      (how == 0) printf("\rx=%+6d y=%+6d z=%+6d ", x, y, z);
         else if (how == 1) printf("\rx=%+5.2f y=%+5.2f z=%+5.2f ", x * MPU_ACCO_SCALE_FACTOR, y * MPU_ACCO_SCALE_FACTOR, z * MPU_ACCO_SCALE_FACTOR);
         else               {
                            FLOAT roll, pitch;
                            ACCO_getRotations(&roll, &pitch);
                            printf("\rroll=%+5.2f pitch=%+5.2f ", RAD_TO_DEG(roll), RAD_TO_DEG(pitch));
                            }
         }
      printf("\n");
      switch (USART_get())
         {
         case 'v': how = (how + 1) % 3; break;
         case '.': ACCO_calibrate();    break;
         case 'q': goto done;           break;
         default:  printf("?\n");       break;
         }
      }
   done:
   printf("\n");
   }

// Calibrate gyros (assumption: device motionless).
//
void
adjust_gyros()
   {
   BYTE how = 0;
   for (;;)
      {
      while (!USART_ready())
         {
         DI();
         SWORD x, y, z;
         GYRO_read_xyz(&x, &y, &z);
         EI();
         x -= GYRO_x_bias;
         y -= GYRO_y_bias;
         z -= GYRO_z_bias;
         if (how == 0) printf("\rx=%+6d y=%+6d z=%+6d ", x, y, z);
         else          printf("\rx=%+6.2f y=%+6.2f z=%+6.2f ", RAD_TO_DEG(x * MPU_GYRO_SCALE_FACTOR), RAD_TO_DEG(y * MPU_GYRO_SCALE_FACTOR), RAD_TO_DEG(z * MPU_GYRO_SCALE_FACTOR));
         }
      printf("\n");
      switch (USART_get())
         {
         case 'v': how = (how + 1) % 2; break;
         case '.': GYRO_calibrate();    break;
         case 'q': goto done;           break;
         default:  printf("?\n");       break;
         }
      }
   done:
   printf("\n");
   }

// See if motion integrator is generating proper angles.
//
void
watch_imu()
   {
   for (;;)
      {
      while (!USART_ready())
         printf("\rdc=%u roll=%+5.1f pitch=%+5.1f yaw=%+5.1f ", IMU_apply_dc, RAD_TO_DEG(IMU_getRollAngle()), RAD_TO_DEG(IMU_getPitchAngle()), RAD_TO_DEG(IMU_getYawAngle()));
      printf("\n");
      switch (USART_get())
         {
         case 'd': IMU_apply_dc = !IMU_apply_dc; break;
         case 'q': goto done;                    break;
         default:  printf("?\n");                break;
         }
      }
   done:
   printf("\n");
   }

#if !HAVE_POLOLU
void
set_filter(BYTE filter)
   {
   printf("filter=%u\n", filter);
   DI();
   TWI_write(MPU_ADDRESS, MPU_CONFIG, filter);
   EI();
   }
#endif
 
// Main loop: run motion compensation, monitor battery, adjust camera trims.
//
void
run()
   {
   LED_on();
   
   // battery monitor (blinking led means "battery needs charging")
   TICKS start_critical = 0;
   TICKS start_blink    = 0;

   // debug
   BYTE  how  = 0;
   FLOAT roll = 0;

   for (;;)
      {
      while (!USART_ready())
         {
         COUNTS start_cam = COUNTER_get();
         
         // track camera to horizon
         roll = IMU_getRollAngle();
         SERVO_setShaftAngle(roll);
         
         // if battery voltage is below critical level for more than 5 seconds, turn off the power
         if (BATTERY_critical())
            { // voltage dipped
            if (!start_critical) start_critical = TIME_now();
            if (TIME_elapsed(start_critical) > 5)
               {
               printf("power off!\n");
               POWER_off();
               }
            }
         else 
            { // voltage recovered
            start_critical = 0;
            }
               
         // blink "battery needs recharge" warning
         if (BATTERY_low())
            { // voltage dipped
            if (!start_blink) start_blink = TIME_now();
            if (TIME_elapsed(start_blink) > .2)
               {
               LED_toggle();
               start_blink = TIME_now();
               }
            }
         else
            { // voltage recovered
            LED_on();
            start_blink = 0;
            }

         COUNTS stop_cam = COUNTER_get();
         
         // display info
         switch (how)
            {
            // nothing
            case 0: break;
            
            // camera trims
            case 1: printf("\rdc=%u roll=%+6.1f C=%+6.1f L=%+5.2f R=%+5.2f rev=%1u bat=%4.2fV (%c%c %2.0f,%2.0f)",
                          IMU_apply_dc,
                          RAD_TO_DEG(roll),
                          RAD_TO_DEG(SERVO_center),
                          SERVO_lgain,
                          SERVO_rgain,
                          SERVO_reverse,
                          BATTERY_read(),
                          BATTERY_low()      ? 'L' : ' ',
                          BATTERY_critical() ? 'C' : ' ',
                          start_blink        ? TIME_elapsed(start_blink)    : 0,
                          start_critical     ? TIME_elapsed(start_critical) : 0
                          );
                    break;
            
            // statistics
            case 2: {
                    #define LIM (2 * 1000.0 * (1.0 / TICKER_HZ)) // ISR must complete within 2 timer tick intervals in order to avoid lost interrupts and inaccurate imu integration [see "ticker.h"]
                    COUNTS cam_duration = stop_cam - start_cam;
                    printf("\rt=%-5.1f isr=%2u (%4.2fms/%4.2fms, %3.0fHz) cam=%2u (%4.2fms, %4.0fHz)",
                          TIME_elapsed(0), 
                          ISR_Duration, COUNTER_counts_to_ms(ISR_Duration), LIM, 1000. / COUNTER_counts_to_ms(ISR_Duration),
                          cam_duration, COUNTER_counts_to_ms(cam_duration),      1000. / COUNTER_counts_to_ms(cam_duration)
                          );
                    break;
                    }
            }
         }

      switch (USART_get())
         {
         // -------------
         // adjust camera 
         // -------------
         
         case '.': CAMERA_align(); break; // for tilted camera installation
         case 'z': CAMERA_zero();  break; // for level camera installation
         
         // The camera centering and throws can be fine tuned with the +/- keys.
         // If the camera is level...         the +/- keys adjust the centering.
         // If the camera is leaning left...  the +/- keys adjust the left gain.
         // If the camera is leaning right... the +/- keys adjust the right gain.
         // These adjustments must be made with the drift correction turned OFF (using "d" key).
         // Note: "+" turns the lens clockwise as viewed from rear of camera.
         
         case '=':
         case '+': if      (roll < DEG_TO_RAD(-10)) { if (SERVO_reverse) SERVO_lgain += .02; else SERVO_rgain += .02; }
                   else if (roll > DEG_TO_RAD(+10)) { if (SERVO_reverse) SERVO_rgain -= .02; else SERVO_lgain -= .02; }
                   else                                                  SERVO_center -= DEG_TO_RAD(.5);
                   break;
         
         case '-': if      (roll < DEG_TO_RAD(-10)) { if (SERVO_reverse) SERVO_lgain -= .02; else SERVO_rgain -= .02; }
                   else if (roll > DEG_TO_RAD(+10)) { if (SERVO_reverse) SERVO_rgain += .02; else SERVO_lgain += .02; }
                   else                                                  SERVO_center += DEG_TO_RAD(.5);
                   break;

         case 'r': SERVO_reverse = !SERVO_reverse;
                   break;

         case 'Z': SERVO_center  = 0;
                   SERVO_lgain   = 1;
                   SERVO_rgain   = 1;
                   SERVO_reverse = 0;
                   break;
         
         // -----
         // debug
         // -----
         
         case 'v': how = (how + 1) % 3; printf("\n"); break;
         case 'd': IMU_apply_dc = !IMU_apply_dc;      break; // toggle drift correction
         case 'j': BATTERY_k -= .0001;                break; // test battery warning
         case 'k': BATTERY_k += .0001;                break; // "
         
         #if !HAVE_POLOLU
         case '1': set_filter(1); break;
         case '2': set_filter(2); break;
         case '3': set_filter(3); break;
         case '4': set_filter(4); break;
         case '5': set_filter(5); break;
         case '6': set_filter(6); break;
         #endif
         
         case 'q': goto done;     break;
         default:  printf("?\n"); break;
         }
      }
   done:
   printf("\n");
   }

// Inspect and adjust subsystems.
//
void
debug()
   {
   LED_on();
   
   for (;;)
      {
      printf("%u I)nitialize b)attery a)cco g)yro i)imu r)un n)ormal d)ebug s)ave R)eboot >", STACK_free());
      char ch = USART_get();
      printf("\n");
      switch (ch)
         {                                                                   // commands listed in order of new board setup steps
         case 'I': CONFIG_init(); CONFIG_save(); reboot();            break; // setup eeprom
         case 'b': adjust_battery();                                  break; // adjust battery constant
         case 'a': adjust_accelerometers();                           break; // adjust accelerometer biases
         case 'g': adjust_gyros();                                    break; // adjust gyro biases
         case 'i': watch_imu();                                       break; // see if imu is operating properly
         case 'r': run();                                             break; // run camera and adjust trims
         case 'n': CONFIG_Data.state =  CONFIG_READY; printf("ok\n"); break; // mark for normal startup on next boot
         case 'd': CONFIG_Data.state = !CONFIG_READY; printf("ok\n"); break; // mark for debug  startup on next boot
         case 's': CONFIG_save();                     printf("ok\n"); break; // save configuration data to eeprom
         case 'R': case LOADER_REQUEST_REBOOT: reboot();              break; // reboot
         default:  printf("?\n");                                     break;
         }
      }
   }

int
main(int argc, char **argv)
   {
   BYTE mcusr = (argc == BOOTLOADER_MAGIC) ? (WORD)argv : MCUSR;
   MCUSR = 0;

   STACK_init();
   SYSTEM_init();
   USART_init();
   STDOUT_init();
   LED_init();

   LED_on();
   printf("%s\n", VERSION);

   // reason for boot
   printf("mcusr=%02x", mcusr);
   if (mcusr & (1 << PORF))  printf(" power-on-reset");
   if (mcusr & (1 << BORF))  printf(" brownout-reset");
   if (mcusr & (1 << WDRF))  printf(" watchdog-reset");
   if (mcusr & (1 << EXTRF)) printf(" external-reset");
   printf("\n");

   // fuse configuration
   BYTE L = boot_lock_fuse_bits_get(GET_LOW_FUSE_BITS);
   BYTE H = boot_lock_fuse_bits_get(GET_HIGH_FUSE_BITS);
   BYTE E = boot_lock_fuse_bits_get(GET_EXTENDED_FUSE_BITS) & 0x07;
   printf("fuses=(%02x %02x %02x)\n", L, H, E);

   // memory status
   printf("free=%u\n", STACK_free());
   
   // initialize subsystems
   CONFIG_recall();
   COUNTER_init();
   BATTERY_init();
   POWER_init();
   TWI_init(0);
   MPU_init();
   SERVO_init();
   CAMERA_init();
   BUTTON_init();
   TICKER_init();

   // button held at least 1 second at startup means "use current camera orientation as 'home' position"
   if (BUTTON_held(1.0))
      {
      LED_off(); // indicate button recognized

      while (BUTTON_pressed()) ;
      delay_ms(2000);
      CAMERA_align();
      CONFIG_save();

      LED_on();  // indicate camera alignment completed
      }
      
   // if all configuration data is present and ready for use, run main loop
   if (CONFIG_Data.state == CONFIG_READY)
      { 
      printf("starting\n");
      run();
      }
   
   // some configuration data may be missing: run debugger to set it
   debug();
   
   return 0;
   }
