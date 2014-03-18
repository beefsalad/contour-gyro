// Type declarations.
//
typedef unsigned char BOOL;   //  1 bit    0..1

typedef unsigned char BYTE;   //  8 bits   0..255
typedef signed   char SBYTE;  //  8 bits   -127..128

typedef unsigned int  WORD;   // 16 bits   0..64K
typedef signed   int  SWORD;  // 16 bits   -32K..+32K

typedef unsigned long DWORD;  // 32 bits   0..4G
typedef signed   long SDWORD; // 32 bits   -2G..2G

// Float and double are identical for avr-gcc, but the compiler doesn't
// seem to promote float to double when passing args, eg. to printf.
// So as a workaround, we'll use double everywhere, but call it FLOAT.
//
typedef double FLOAT;         // 32 bits

// Documentation aids.
//
#define PUBLIC  static        // interfaces
#define PRIVATE static        // implementation details

