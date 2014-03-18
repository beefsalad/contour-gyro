// Atmega bootloader interface.
//

// size of xbee input buffer
//
#define XBEE_BUFFER_SIZE 202 

// size of a flash page on atmega target
//
#define LOADER_PAGE_SIZE 128

// make sure declarations match on host and target compilers
//
#ifdef SPM_PAGESIZE 
   // using atmega cross compiler
   #if LOADER_PAGE_SIZE != SPM_PAGESIZE
   #error declaration mismatch!
   #endif
#endif

// max data that atmega bootloader wants per transaction
//
//    LOADER_DATA_LIMIT
//       == LOADER_PAGE_SIZE
//          == SPM_PAGESIZE
//             == 128
//
//
#define LOADER_DATA_LIMIT LOADER_PAGE_SIZE

// loader transaction packet format
//
//    count (1 binary byte), 0 indicates eof
//    |  data (N binary bytes)
//    |  |
//    N  DDDDDDDDDDDDDDDDDDDDDDD
//
#define LOADER_PACKET_SIZE (1 + LOADER_DATA_LIMIT)

// sanity checks
//
#if LOADER_DATA_LIMIT > 255
#error data count would overflow 1 byte!
#endif

#if LOADER_PACKET_SIZE > XBEE_BUFFER_SIZE
#error xbee transmitter overrun risk!
#endif

// messages from atmega bootloader to host
//
#define LOADER_REQUEST_FIRST_BLOCK '!'
#define LOADER_REQUEST_NEXT_BLOCK  '*'
#define LOADER_ERROR               'E'

// messages from host to atmega application program
//
#define LOADER_REQUEST_REBOOT '$'

// parameters passed from bootloader to main(argc, argv)
// argc is BOOTLOADER_MAGIC (indicating that bootloader is present on this chip)
// argv is MCUSR register at time of startup (indicating reason for startup - ie. external-reset, watchdog-reset, etc.)
//
#define BOOTLOADER_MAGIC 0x1234
