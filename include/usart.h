// Interface to ATMEGA168 serial i/o controller.
// Units:      USART0
// Interrupts: USART_RX_vect (optional)
// Pins:       PORTD0, PORTD1
// Clock:      8/16Mhz
//
// For interrupt driven version specify:
//    #define USART_USE_INTERRUPT 1
//    #define USART_SIZE 16 // data buffer size (for example)
//

// Send a byte on usart, via polling.
//
static void
USART_put(BYTE c)
   {
   // wait for usart data register to become empty
   while (!(UCSR0A & (1 << UDRE0))) ;
   
   // fill usart data register
   UDR0 = c;
   }

#if USART_USE_INTERRUPT
 
// Receive data from usart via interrupts.
//

#ifndef USART_SIZE
#error  USART_SIZE
#endif

static volatile BYTE USART_data[USART_SIZE];
static volatile BYTE USART_fill;

// "USART Rx Complete" interrupt handler.
//
ISR(USART_RX_vect) 
   {
   BYTE ch = UDR0; // empty usart data register
   if (USART_fill < USART_SIZE)
      USART_data[USART_fill++] = ch;
   }

// Is a character present in interrupt buffer?
//
static BOOL
USART_ready()
   {
   return USART_fill != 0;
   }

// Empty interrupt buffer.
//
static void
USART_empty()
   {
   USART_fill = 0;
   }
   
// Examine first character in interrupt buffer.
//
static BYTE
USART_peek()
   {
   while (!USART_ready()) ;
   return USART_data[0];
   }

#else

// Receive data from usart via polling.
//

// Check for presence of incoming data.
//
inline BOOL
USART_ready()
   {
   return (UCSR0A & (1<<RXC0));
   }
   
// Receive a byte from usart, via polling.
//
inline BYTE
USART_get()
   {
   // wait for usart data register to fill ("receive complete")
   while (!(UCSR0A & (1<<RXC0))) ;
   
   // empty usart data register
   return UDR0;
   }
#endif

// Initialize for 9600 baud, 8N1.
// Assumptions: power-on usart defaults in effect
//
static void
USART_init()
   {
   // power-on defaults are:
   // - asynchronous mode
   // - 1M baud rate for 16MHz clock, undefined for 8MHz clock
   // - 8N1 frames (10 bits: 1 start, 8 data, 0 parity, 1 stop)
   //

#if CLOCK_MHZ == 8

   // set baud rate to 9600 using UBRR value
   // from table 19-11 of atmega168 datasheet
   // for FOSC=8Mhz, U2Xn=0
   //
   #define MYUBRR  51UL

#elif CLOCK_MHZ == 16

   // set baud rate to 9600 using UBRR value
   // from table 19-12 of atmega168 datasheet
   // for FOSC=16Mhz, U2Xn=0
   //
   #define MYUBRR 103UL

#else
   #error CLOCK_MHZ
#endif

   UBRR0H = MYUBRR >> 8;
   UBRR0L = MYUBRR;
   
   // configure PORTD0,PORTD1 for use as usart RXD,TXD
   UCSR0B |= (1 << RXEN0) | (1 << TXEN0);

   // activate pullup resistor on RX line, so it doesn't generate spurious data if unconnected (floating)
   PORTD |=  (1 <<  PD0);

#if USART_USE_INTERRUPT
   // enable "USART Rx Complete" interrupts
   UCSR0B |= (1 << RXCIE0);
#endif
   }

#if 0 // UNUSED
void
USART_fini()
   {
#if USART_USE_INTERRUPT
   // disable "USART Rx Complete" interrupts
   UCSR0B &= ~(1 << RXCIE0);
#endif
   // unconfigure PORTD pins 0,1 for use as usart RXD,TXD
   UCSR0B &= ~((1 << RXEN0) | (1 << TXEN0));
   }
#endif
