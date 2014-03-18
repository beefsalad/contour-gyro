// Interface to Pololu #750 software controllable pushbutton power switch.
//
// Ports: PORTC2
//
// This device consists of a flip-flop latching circuit connected to a MOSFET power switch.
// An external pushbutton toggles the flip-flip to turn power on/off.
// The flip-flop can also be driven by software via a pin on the device: 0V=on, >1V=off.
//
// Operating voltage: 4.5-20V
// Maximum ON current: 10A
// Maximum OFF current: <.01 microamps
//

PUBLIC void
POWER_init()
   {
   // note: at startup, all Atmega I/O pins are configured as tri-stated inputs (high-impedance, no pullup resistor) ie. Pxy=0, DDxy=0
   PORTC &= ~(1 << PC2); // output low (same as power on default)
   DDRC  |= (1 << DDC2); // configure pin as output
   }

PUBLIC void
POWER_off()
   {
   PORTC |= (1 << PC2);  // output high
   }
