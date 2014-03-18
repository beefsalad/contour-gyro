// Talk to devices via TWI (two-wire interface).
//
// Units:      TWI
// Interrupts: none
// Pins:       SDA (PORTC4)
//             SCL (PORTC5)
// Clock:      8Mhz
//
// Usage:      #define TWI_KHZ  50
//        /or/ #define TWI_KHZ 100
//        /or/ #define TWI_KHZ 200
//             #include "twi.h"
//
// 16 Oct 2011 Derek Lieber
//

// --------------------------------------------------------------------
//                        Implementation.
// --------------------------------------------------------------------

#include <util/twi.h> // twi status codes - /usr/lib/avr/include/util/twi.h

// TWI error notifier.
//
typedef void (*TWI_FUNC)();
static TWI_FUNC TWI_notify;

// Recover from a TWI bus error.
// See section 21.7.5 and table 21-6 in databook.
//
static void
TWI_reset()
   {
   TWCR = ((1 << TWINT) | (1 << TWSTO));  // enter "not addressed" slave mode and release SDA/SCL lines
   }

// Report a TWI error.
// Taken:    operation type (exec, start, send, recv, stop)
// Returned: nothing
// Side effect: after reporting error, we reset the bus and attempt to continue execution (with bad data)
//
static void 
TWI_error(const char *op)
   {
   if (!TWI_notify) printf("TWI error: %s\n", op);
   else TWI_notify();
   TWI_reset();
   }

// Execute a TWI operation.
// Taken:    TWCR command bits
// Returned: TWSR status bits
//
static BYTE
TWI_exec(BYTE op)
   {
   // initiate
   TWCR = op;
   
   // wait for completion
   WORD n = 0;
   while (!(TWCR & (1 << TWINT)))
      if (++n == 300) // trial and error timeout value (depends on CLOCK_MHZ, TWI_KHZ, and device being addressed)
         {
         TWI_error("exec");
         break;
         }
   
   // get status bits
   return TWSR & ~((1 << TWPS1) | (1 << TWPS0));
   }

// Start a TWI transaction.
//
static void
TWI_start()
   {
   BYTE status = TWI_exec((1 << TWINT) | (1 << TWEN) | (1 << TWSTA));
   if (status != TW_START && status != TW_REP_START)
      TWI_error("start");
   }

// Send a byte to TWI slave.
// Taken:    byte to send
//           status expected after operation completes
// Returned: nothing
//
static void
TWI_send(BYTE data, BYTE expected)
   {
   TWDR = data;
   BYTE status = TWI_exec((1 << TWINT) | (1 << TWEN));
   if (status != expected)
      TWI_error("send");
   }

// Receive a byte from TWI slave.
// Taken:    should master acknowledge reception?
// Returned: received data
//
static BYTE
TWI_recv(BOOL ack)
   {
   if (ack)
      {
      BYTE status = TWI_exec((1 << TWINT) | (1 << TWEN) | (1 << TWEA));
      if (status != TW_MR_DATA_ACK)
         TWI_error("recv");
      }
   else
      {
      BYTE status = TWI_exec((1 << TWINT) | (1 << TWEN));
      if (status != TW_MR_DATA_NACK)
         TWI_error("recv");
      }
   
   return TWDR;
   }

// Finish a TWI transaction.
//
static void
TWI_stop()
   {
   TWCR = ((1 << TWINT) | (1 << TWEN) | (1 << TWSTO));
   // no wait, no status
   }
   
// --------------------------------------------------------------------
//                          Interface.
// --------------------------------------------------------------------

// Prepare TWI for use.
// Taken:    function to call for TWI error notifications (0=none)
// Returned: nothing
//
static void
TWI_init(TWI_FUNC notify)
   {
   TWI_notify = notify;
   
#if CLOCK_MHZ == 8
   #if TWI_KHZ == 50
      // set TWI clock frequency to 50 KHz
      //
      // CLOCK_HZ      = 8,000,000
      // TWBR          = 72
      // prescaler     = 1
      // SCL frequency = CLOCK_HZ  / (16 + 2 * TWBR * prescaler)
      //               = 8,000,000 / (16 + 2 *  72  * 1)
      //               = 50 KHz

      TWSR = (0 << TWPS1) | (0 << TWPS0); // prescaler = 1
      TWBR = 72;                          // divisor

   #elif TWI_KHZ == 100

      // set TWI clock frequency to 100 KHz
      //
      // CLOCK_HZ      = 8,000,000
      // TWBR          = 32
      // prescaler     = 1
      // SCL frequency = CLOCK_HZ  / (16 + 2 * TWBR * prescaler)
      //               = 8,000,000 / (16 + 2 *  32  * 1)
      //               = 100 KHz

      TWSR = (0 << TWPS1) | (0 << TWPS0); // prescaler = 1
      TWBR = 32;                          // divisor

   #elif TWI_KHZ == 200

      // set TWI clock frequency to 200 KHz
      //
      // CLOCK_HZ      = 8,000,000
      // TWBR          = 12
      // prescaler     = 1
      // SCL frequency = CLOCK_HZ  / (16 + 2 * TWBR * prescaler)
      //               = 8,000,000 / (16 + 2 *  12  * 1)
      //               = 200 KHz

      TWSR = (0 << TWPS1) | (0 << TWPS0); // prescaler = 1
      TWBR = 32;                          // divisor

   #else
   #error TWI_KHZ
   #endif

#elif CLOCK_MHZ == 16
   
   #if TWI_KHZ == 100

      // set TWI clock frequency to 100 KHz
      //
      // CLOCK_HZ      = 16,000,000
      // TWBR          = 72
      // prescaler     = 1
      // SCL frequency = CLOCK_HZ   / (16 + 2 * TWBR * prescaler)
      //               = 16,000,000 / (16 + 2 *  72  * 1)
      //               = 100 KHz

      TWSR = (0 << TWPS1) | (0 << TWPS0); // prescaler = 1
      TWBR = 72;                          // divisor

   #elif TWI_KHZ == 200

      // set TWI clock frequency to 200 KHz
      //
      // CLOCK_HZ      = 16,000,000
      // TWBR          = 32
      // prescaler     = 1
      // SCL frequency = CLOCK_HZ   / (16 + 2 * TWBR * prescaler)
      //               = 16,000,000 / (16 + 2 *  32  * 1)
      //               = 200 KHz

      TWSR = (0 << TWPS1) | (0 << TWPS0); // prescaler = 1
      TWBR = 32;                          // divisor

   #else
   #error TWI_KHZ
   #endif

#else
#error CLOCK_MHZ
#endif
   }
   
// Write one byte to a TWI device.
// Taken:    7 bit device address
//           7 bit register number
//           8 bit value to be written
// Returned: nothing
//
void
TWI_write(BYTE device_address, BYTE register_number, BYTE value)
   {
   // begin transaction (ST)
   TWI_start();

   // send slave address (SLA+W)
   TWI_send((device_address << 1) | TW_WRITE, TW_MT_SLA_ACK);

   // send register number (SUB) 
   TWI_send(register_number, TW_MT_DATA_ACK);
   
   // send register value (DATA)
   TWI_send(value, TW_MT_DATA_ACK);

   // end transaction (SP)
   TWI_stop();
   }

// Read one byte from a TWI device.
// Taken:    7 bit device address
//           7 bit register number
// Returned: value read
//
BYTE
TWI_read(BYTE device_address, BYTE register_number)
   {
   // begin transaction (ST)
   TWI_start();

   // send slave address (SLA+W)
   TWI_send((device_address << 1) | TW_WRITE, TW_MT_SLA_ACK);

   // send register number (SUB) 
   TWI_send(register_number, TW_MT_DATA_ACK);

   // begin transaction (SR)
   TWI_start();

   // send slave address (SLA+R)
   TWI_send((device_address << 1) | TW_READ, TW_MR_SLA_ACK);

   // receive register value (DATA)
   BYTE value = TWI_recv(0); // nack last (only) byte

   // end transaction (SP)
   TWI_stop();

   return value;
   }

#define TWI_AUTO_INCREMENT 0x80 // to auto-increment the register number on multi-byte reads, "or" it with this value

// Read multiple bytes from a TWI device.
// Taken:    7 bit device address
//           7 bit register number (possibly or'ed with TWI_AUTO_INCREMENT)
//           number of bytes to read
//           place to put them
// Returned: nothing
//
void
TWI_read_multi(BYTE device_address, BYTE register_number, BYTE n, BYTE *dst)
   {
   // begin transaction (ST)
   TWI_start();

   // send slave address (SLA+W)
   TWI_send((device_address << 1) | TW_WRITE, TW_MT_SLA_ACK);

   // send register number (SUB) 
   TWI_send(register_number, TW_MT_DATA_ACK);

   // begin transaction (SR)
   TWI_start();

   // send slave address (SLA+R)
   TWI_send((device_address << 1) | TW_READ, TW_MR_SLA_ACK);

   // receive register values (DATA)
   while (n--)
       *dst++ = TWI_recv(n != 0); // ack all bytes but last

   // end transaction (SP)
   TWI_stop();
   }
