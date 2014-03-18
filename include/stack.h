// Monitor size of program stack.
//

// Linker-supplied symbol marking end of program data.
//
extern BYTE _end;

// Mark out space available for use by stack.
//
static void
STACK_init()
   {
   BYTE *stack = (BYTE *)SP;
   BYTE *p     = &_end;
   while (p < stack)
      *p++ = 0xAA;
   }
   
// Query free space in data area as yet unclaimed by program stack.
//
static WORD
STACK_free()
   {
   BYTE *p = &_end;
   WORD  n = 0;
   while (*p++ == 0xAA)
      ++n;
   return n;
   }
