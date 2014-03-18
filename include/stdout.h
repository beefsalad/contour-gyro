// Standard output via ATMEGA168 serial i/o controller.
//
#include <stdio.h>

// Wrapper signature required by FDEV_SETUP_STREAM.
//
static int
STDOUT_putc(char c, FILE *stream)
   {
   USART_put(c);
   return 0;
   }
   
// Hook up stdout to usart, so we can use printf().
//
static void
STDOUT_init()
   {
   static FILE USART_stdout = FDEV_SETUP_STREAM(STDOUT_putc, NULL, _FDEV_SETUP_WRITE);
   stdout = &USART_stdout;
   }
