// Pushbutton control.
//
// Ports: PORTB0

// --------------------------------------------------------------------
// Interface.
// --------------------------------------------------------------------

// Prepare button for use.
//
PUBLIC void
BUTTON_init()
   {
   // configure pin for input
   DDRB  &= ~(1 << DDB0);
   
   // activate pullup resistor
   PORTB |=  (1 <<  PB0);
   }

// Is button pressed?
//
PUBLIC BOOL
BUTTON_pressed()
   {
   return (PINB & (1 << PINB0)) == 0;
   }

// Has button been pressed for at least N seconds?
//
PUBLIC BOOL
BUTTON_held(FLOAT N)
   {
   TICKS start = TIME_now();
   while (TIME_elapsed(start) < N)
      if (!BUTTON_pressed())
         return 0;
   return 1;
   }
