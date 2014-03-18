// Interface to ATMEGA168 EEPROM persistent store.
//

// Write byte to eeprom.
// Taken:    address to be written (0..511)
//           data to be written
// Returned: nothing
//
static void
EEPROM_write(WORD address, BYTE data)
   {
   // wait for completion of previous write, if any (~3.3ms)
   while (EECR & (1 << EEPE)) ;
   
   // set up address and data registers
   EEAR = address;
   EEDR = data;
   
   // initiate write operation
   // (master write enable, followed within 4 cycles by write enable)
   DI();
   EECR |= (1 << EEMPE);
   EECR |= (1 << EEPE);
   EI();
   }

// Read byte from eeprom.
// Taken:    address to be read (0..511)
// Returned: data
//
static BYTE
EEPROM_read(WORD address)
   {
   // wait for completion of previous write, if any (~3.3ms)
   while (EECR & (1 << EEPE)) ;

   // set up address register
   EEAR = address;
   
   // initiate read operation
   EECR |= (1 << EERE);

   // fetch data
   return EEDR;
   }

// Write block to eeprom.
// Taken:    address to be written (0..511)
//           place to get the data
//           number of bytes to transfer (0..511)
// Returned: nothing (data is deposited)
//
void
EEPROM_write_block(WORD dst, void *src, WORD cnt)
   {
   BYTE *s = (BYTE *)src;
   while (cnt--) 
      EEPROM_write(dst++, *s++);
   }

// Read block from eeprom.
// Taken:    address to be read (0..511)
//           place to put the data
//           number of bytes to transfer (0..511)
// Returned: nothing (data is deposited)
//
void
EEPROM_read_block(WORD src, void *dst, WORD cnt)
   {
   BYTE *d = (BYTE *)dst;
   while (cnt--) 
      *d++ = EEPROM_read(src++);
   }
