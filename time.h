// Timing functions based on real time clock.
//

// --------------------------------------------------------------------
// Interface.
// --------------------------------------------------------------------

// Get current time.
//
PUBLIC TICKS
TIME_now()
   {
   extern volatile TICKS ISR_Ticks;
   DI();
   TICKS t = ISR_Ticks;
   EI();
   return t;
   }

// How much time has elapsed since "start", in seconds?
//
PUBLIC FLOAT
TIME_elapsed(TICKS start)
   {
   return (TIME_now() - start) * (1.0 / TICKER_HZ);
   }

// Pause a moment.
//
PUBLIC void
TIME_pause(FLOAT seconds)
   {
   TICKS start = TIME_now();
   while (TIME_elapsed(start) < seconds) ;
   }
