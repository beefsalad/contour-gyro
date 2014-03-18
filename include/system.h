// Atmega startup.
// Units:      CLOCK, WATCHDOG
// Interrupts: enabled
// Pins:       none
// Clock:      8/16Mhz
//
static void
SYSTEM_init()
   {

/// OBSOLETE: at one time we supported running at 8Mhz with external crystal, but I don't think we have any code that does this anymore
///
/// #if CLOCK_MHZ == 8
///    #if CLOCK_INTERNAL
///    // 8Mhz internal oscillator with no clock prescaler
///    ;
///    #else
///    // 16Mhz crystal oscillator with clock prescaler,
///    // initially set to "divide by 8", yielding a 2Mhz system clock.
///    // We reconfigure the prescaler to yield an 8Mhz system clock.
///    //
///    CLKPR = (1 << CLKPCE); // toggle clock prescaler change enable...
///    CLKPR = (0 << CLKPCE)  // ...and         [ see p. 37
///          | (0 << CLKPS3)  // ...set              of datasheet ]
///          | (0 << CLKPS2)  // ...new
///          | (0 << CLKPS1)  // ...clock prescaler
///          | (1 << CLKPS0)  // ...to "divide by 2"
///          ;
///    #endif
/// #elif CLOCK_MHZ == 16
///    // 16Mhz crystal oscillator with no clock prescaler
///    ;
/// #else
///    #error CLOCK_MHZ
/// #endif

   // check for obsolete flag
   //
   #if defined(CLOCK_INTERNAL)
   #error Please stop using CLOCK_INTERNAL
   #endif
   
   // Turn off watchdog timer in case it was the source of this system reset.
   // Ref: section 10.8.1 of datasheet.
   //
   __asm__ volatile ("wdr");            // restart timer so we can issue the next few instructions before watchdog fires again
   MCUSR  &= ~(1 << WDRF);              // clear flag so watchdog won't fire again when we issue sei()
   WDTCSR |=  (1 << WDCE) | (1 << WDE); // disable...
   WDTCSR  =  0;                        // ...watchdog

   // Run with interrupts enabled.
   // Ref: initialization routine on p.64 of datasheet.
   //
   sei();
   }
