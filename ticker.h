// Time base generator and background task dispatcher.
//
// Units:      TIMER0
// Counters:   TCNT0
// Registers:  OCR0A
// Interrupts: TIMER0_COMPA
//
// --------------------------------------------------------------------
// Implementation.
// --------------------------------------------------------------------

// --------------------------------------------------------------------
// Interrupt communication area - updated at TICKER_HZ rate.
//
volatile TICKS  ISR_Ticks;    // number of interrupts
volatile COUNTS ISR_Duration; // time spent in interrupt service routine
// --------------------------------------------------------------------

// Interrupt service routine executed at TICKER_HZ rate.
//
ISR(TIMER0_COMPA_vect)
   {
   // Update timebase.
   ISR_Ticks += 1;
   
   // Dispatch background tasks at IMU_HZ rate.
   //
   // Note that these functions must complete in less than 2 timer tick intervals in order to avoid losing interrupts.
   // Given the way we've configured the interrupt rate and 2/4 divider:
   // - For 16 MHz clock, 2 timer tick intervals = 2 ms. The measured interrupt service duration is ~1.5 ms.
   // - For  8 MHz clock, 2 timer tick intervals = 4 ms. The measured interrupt service duration is ~3.0 ms.
   //
   static BYTE n;
#if   IMU_HZ == TICKER_HZ / 4
   if (++n & 3) return;
#elif IMU_HZ == TICKER_HZ / 2
   if (++n & 1) return;
#else
   #error TICKER
#endif
   
   COUNTS start = COUNTER_get();
   MPU_update();
   IMU_update();
   ISR_Duration = COUNTER_get() - start;
   }

// --------------------------------------------------------------------
// Interface.
// --------------------------------------------------------------------

// Begin generating interrupts for time base and dispatcher.
//
PUBLIC void
TICKER_init()
   {
   // Set timer for "clear on terminal count" operation.
   // The counter continually runs from 0 to TOP (= OCR0A), then resets to 0.
   // An interrupt is generated each time TOP is reached. 
   //
   TCCR0A = 0
          | (0 << WGM00) // CTC wave generation, TOP=OCR0A (mode 2)
          | (1 << WGM01) // "
          ;

   TCCR0B = 0
          | (0 << WGM02) // "
          
          | (1 << CS00)  // prescaler = 64
          | (1 << CS01)  // "
          | (0 << CS02)  // "
          ;

   // Set terminal count value (TOP) at which to generate a
   // "compare match A" interrupt and clear the counter back to zero.
   //
   // The interrupt rate is:
   //    RATE = CLOCK / PRESCALER / (TOP + 1)
   //
   // Where: PRESCALER = 1, 8, 64, 256, 1024
   //
   // The TOP count needed for a given interrupt rate is:
   //    TOP = CLOCK / PRESCALER / RATE - 1
   //
   // For CLOCK = 16 MHz, PRESCALER = 64, RATE = 1000Hz:
   //    TOP = 16,000,000 / 64 / 1000 - 1 = 249
   //
   // For CLOCK =  8 MHz, PRESCALER = 64, RATE =  500Hz:
   //    TOP =  8,000,000 / 64 / 500  - 1 = 249
   //
   // Note that TIMER0 is an 8 bit timer, so TOP must be <= 255.
   //

   // Generate 1000 interrupts per second for 16 MHz clock,
   // or        500 interrupts per second for  8 MHz clock.
   //
   #if (CLOCK_MHZ == 16 && TICKER_HZ == 1000) || (CLOCK_MHZ == 8 && TICKER_HZ == 500)
   OCR0A = 249;  // TOP
   #else
   #error TICKER
   #endif
   
   // Start counter at 0.
   //
   TCNT0 = 0;
   
   // Enable "TIMER0 compare match A" interrupts.
   //
   TIMSK0 |= (1 << OCIE0A);

   printf("clock=(%.3fus,%uMHz) ticker=(%.2fms,%uHz) timestep=(%.2fms,%uHz)\n",
          1e6 / (CLOCK_MHZ * 1e6), CLOCK_MHZ,
          1e3 / TICKER_HZ,         TICKER_HZ,
          1e3 / IMU_HZ,            IMU_HZ
          );
   }
