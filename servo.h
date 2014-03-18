// Servo controller.
//
// Units:     TIMER1
// Counters:  TCNT1
// Registers: ICR1A, OCR1A
// Ports:     PORTB1

// --------------------------------------------------------------------
// Interface.
// --------------------------------------------------------------------

// Servo offset required to level the camera, in radians.
//
PUBLIC FLOAT SERVO_center;

// Servo travel volume.
//
PUBLIC FLOAT SERVO_lgain;
PUBLIC FLOAT SERVO_rgain;

// Servo reversal.
//
PUBLIC BOOL SERVO_reverse;

// Prepare servo interface for use.
//
PUBLIC void
SERVO_init()
   {
   // Set TIMER1 to generate 20ms period and nominal 1.5ms pulse width.
   //
   // We use waveform generation mode 8
   // "PWM, phase and frequency correct, TOP=ICR1".
   //
   // The counter, TCNT1, will run from 0 to TOP back down to 0.
   // We drive the OC1A pin "hi" during the portion of time
   // for which TCNT1 <= OCR1A.
   //
   // We use timer clock == system clock divided by 8.
   // For a 16MHz system clock and a TOP value of 20,000
   // this will yield a 50Hz (20ms) waveform:
   //
   // 16e6 / 8 / (2e4 upcounts + 2e4 downcounts) = .5e2 = 50 Hz
   //
   //                     . _ _ _ _ _ _ _ _ _ TCNT1 == TOP == ICR1 = 20,000
   //                   .   .               .
   //                 .       .           .
   //       .       .           .       .
   //         .   . _ _ _ _ _ _ _ . _ . _ _ _ TCNT1 == OCR1A
   // TCNT1:    .                   . _ _ _ _ TCNT1 == BOTTOM == 0
   //
   //         . . .               . . .
   //         .   .               .   .
   // OC1A: . .   . . . . . . . . .   . . . .
   //                             |-+-|  2 x  1,500 counts (1.5ms)
   //           |---------+---------|    2 x 20,000 counts (20ms)
   
   TCCR1A =  
          0
                          // table 15-3
          | (0 << COM1A0) // clear OC1A on match when upcounting...
          | (1 << COM1A1) // ...set on match when downcounting

                          // table 15-4
          | (0 << WGM10)  // waveform generation mode 8
          | (0 << WGM11)  // waveform generation mode 8
          ;
   
   TCCR1B =
          0
                          // table 15-4
          | (0 << WGM12)  // waveform generation mode 8
          | (1 << WGM13)  // waveform generation mode 8

                          // table 15-5
          | (0 << CS10)   // timer clock = system clock / 8
          | (1 << CS11)   // timer clock = system clock / 8
          | (0 << CS12)   // timer clock = system clock / 8
          ;

#if CLOCK_MHZ == 16

   ICR1 = 20000;                        // TOP value for waveform width of 20ms at 16Mhz/8
   #define SERVO_CENTER_COUNTS     1500 // pwm counts for 0 degrees of servo rotation
   #define SERVO_COUNTS_PER_DEGREE 10   // pwm counts per degree of servo rotation
   #define SERVO_LIMIT_TENTHS      900  // +/- travel limit, in tenths of a degree
   
#elif CLOCK_MHZ == 8

   ICR1 = 10000;                       // TOP value for waveform width of 20ms at 8Mhz/8
   #define SERVO_CENTER_COUNTS     750 // pwm counts for 0 degrees of servo rotation
   #define SERVO_COUNTS_PER_DEGREE 5   // pwm counts per degree of servo rotation
   #define SERVO_LIMIT_TENTHS      900 // +/- travel limit, in tenths of a degree

#else
   #error CLOCK_MHZ
#endif
   
   // set center position
   //
   OCR1A = SERVO_CENTER_COUNTS;
   TCNT1 = 0;
   
   // start generating waveform
   //
   DDRB |= (1 << DDB1);   // enable PORTB1 as output for use by OC1A
   }

// Turn servo to specified shaft angle.
// Taken: shaft angle, in radians
//
PUBLIC void
SERVO_setShaftAngle(FLOAT angle)
   {
   angle += SERVO_center;
   
   // convert radians to tenths of a degree
   SWORD target = RAD_TO_DEG(angle) * 10;
   
   // account for servo gearing reversal
   if (SERVO_reverse) target = -target;
   
   // account for servo throw asymmetry
   target *= target > 0 ? SERVO_lgain : SERVO_rgain;
   
   // don't exceed servo hard stops
   if      (target < -SERVO_LIMIT_TENTHS) target = -SERVO_LIMIT_TENTHS;
   else if (target > +SERVO_LIMIT_TENTHS) target = +SERVO_LIMIT_TENTHS;

   // convert to PWM counter value
   OCR1A = SERVO_CENTER_COUNTS + (target * SERVO_COUNTS_PER_DEGREE) / 10;

// printf(" servo: %+6.1f->%+6d\r", RAD_TO_DEG(angle), OCR1A);
   }

#if 0 // UNUSED
// Manual test.
//
PUBLIC void
SERVO_test()
   {
   SERVO_init();
   for (;;)
      {
      switch(USART_get())
         {
         case 'c': OCR1A  = SERVO_CENTER_COUNTS;     break;
         case 'j': OCR1A += SERVO_COUNTS_PER_DEGREE; break;
         case 'k': OCR1A -= SERVO_COUNTS_PER_DEGREE; break;
         case 'q': goto done;                        break;
         }
      printf("%5u\r", OCR1A);
      }
   done:
   printf("\n");
   }
#endif
