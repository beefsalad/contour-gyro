// LED control.
//
// Ports: PORTC0
//

inline void
LED_off()
   {
   // drive PORTC0 pin high
   PORTC |= (1 << PC0);
   }

inline void
LED_on()
   {
   // drive PORTC0 pin low
   PORTC &= ~(1 << PC0);
   }

inline void
LED_set(BOOL on)
   {
   if (on) LED_on();
   else    LED_off();
   }

inline void
LED_toggle()
   {
   PORTC ^= (1 << PC0);
   }

static void
LED_init()
   {
   LED_off();
   DDRC |= (1 << DDC0); // configure PORTC0 as output
   }
