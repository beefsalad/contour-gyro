// Force a processor reset, to allow a new program to be bootloaded.
// Units:      watchdog timer
// Interrupts: system reset
// Pins:       none
// Clock:      any
//
static void
reboot()
   {
   DI();

   // force a watchdog reset
   WDTCSR |= (1 << WDCE) | (1 << WDE);
   WDTCSR  = 0
             | (0 << WDCE)
             | (1 << WDE)  // use "reset mode" not "interrupt mode"

             | (0 << WDP0) // fire reset after 16ms
             | (0 << WDP1)
             | (0 << WDP2)
             | (0 << WDP3)
             ;
   
   // wait for it
   for (;;);
    
   EI();
   }
