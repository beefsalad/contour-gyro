// Software delays (approximate, eyeballed against wall clock).
//

#ifndef __OPTIMIZE__
#error These loops must be compiled with "-Os" optimization flag.
#endif

#if CLOCK_MHZ == 16
// Delay, in milliseconds.
//
__attribute__((noinline)) void
delay_ms(WORD ms)
   {
   WORD n;
   while (ms--)
     for (n = 0; n < 3200; ++n)
         __asm__ volatile ("nop");
   }
#endif

#if CLOCK_MHZ == 8
// Delay, in milliseconds.
//
__attribute__((noinline)) void
delay_ms(WORD ms)
   {
   WORD n;
   while (ms--)
     for (n = 0; n < 1600; ++n)
       __asm__ volatile ("nop");
   }
#endif

#if CLOCK_MHZ == 8
// Delay, in microseconds.
//
__attribute__((noinline)) void
delay_us(WORD us)
   {
   while (us--)
      __asm__ volatile ("nop");
   }
#endif
