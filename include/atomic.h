// Interrupt control for atomic operations.
//

// Save current interrupt status and disable interrupts.
//
#define DI() BYTE sreg = SREG; cli()

// Restore previous interrupt status.
//
#define EI() SREG = sreg
