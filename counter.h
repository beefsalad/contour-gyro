// Cycle counter.
//
// Units:    TIMER2
// Counters: TCNT2
//

// An interval measured by cycle counter.
//
typedef BYTE COUNTS;

// Prepare cycle counter interface for use.
// For 16 MHz clock, resolution is 1 count = .064 ms, limit is 255 counts = 16.320 ms.
//
PUBLIC void
COUNTER_init()
   {
   TCCR2A = 0;           // normal wave generation mode (counter runs from 0 to 255 and wraps)

   TCCR2B = 0            // table 17-9
          | (1 << CS20)  // timer clock = system clock / 1024
          | (1 << CS21)  // "
          | (1 << CS22)  // "
          ;
   
   FLOAT counts_per_second = (CLOCK_MHZ * 1e6) / 1024.;
   FLOAT seconds_per_count = 1 / counts_per_second;
   
   printf("res=%.3fms lim=%.3fms\n", seconds_per_count * 1000., seconds_per_count * 255. * 1000.);
   }

// Fetch current counter.
//
PUBLIC COUNTS
COUNTER_get()
   {
   return TCNT2;
   }

// Convert counts to milliseconds.
//
PUBLIC FLOAT
COUNTER_counts_to_ms(COUNTS counts)
   {
   return counts * 1024. / (CLOCK_MHZ * 1e6) * 1000.;
   }
