// Battery monitor.
//
// Units: ADC
// Ports: ADC1 (PORTC1)
// Clock: 8 or 16 MHz
//

// --------------------------------------------------------------------
// Implementation.
// --------------------------------------------------------------------

// Take a reading on ADC1 (external battery monitor).
// Taken:    nothing
// Returned: adc counts (0..1023)
//
PRIVATE WORD
ADC_read_battery()
   {
   // wait for previously started conversion to complete
   //
   while (ADCSRA & (1 << ADSC));

   // read out the result
   //
   BYTE lsb = ADCL;
   BYTE msb = ADCH;
   WORD val = (msb << 8) | lsb;
   
   // start next adc conversion
   //
   ADCSRA |= (1 << ADSC);

   return val;
   }

// ADC-digits to volts conversion factor (using 10k over 1.2k resistor divider and internal 1.1v reference)
//
#define BATTERY_K_DEFAULT .01
FLOAT BATTERY_k = BATTERY_K_DEFAULT;

// Convert adc reading to volts.
//
PRIVATE FLOAT
ADC_battery_counts_to_volts(WORD adc_counts)
   {
   return adc_counts * BATTERY_k;
   }

// --------------------------------------------------------------------
// Interface.
// --------------------------------------------------------------------

// Prepare adc for use.
//
PUBLIC void
BATTERY_init()
   {
   // adc needs a 50-200 KHz clock
   // lower clock  = more accurate
   // higher clock = faster conversion
   //
   // for 8 MHz system clock:
   //    prescaler  adc clock
   //       128      62.5 KHz
   //       64       125  KHz
   //       32       250  KHz
   //
   // for 16 MHz system clock:
   //    prescaler  adc clock
   //       128      125  KHz

   #if CLOCK_MHZ != 8 && CLOCK_MHZ != 16
   #error CLOCK_MHZ
   #endif
   
   ADCSRA = 0
          | (1 << ADPS0)  // adc prescaler = divide by 128 (highest available divisor, greatest accuracy)
          | (1 << ADPS1)
          | (1 << ADPS2)
          
          | (1 << ADEN)   // enable adc
          ;

   // select ADC1
   //
   ADMUX  = 0             
          | (1 << REFS0)  // voltage reference = internal 1.1v
          | (1 << REFS1)

          | (1 << MUX0)   // voltage source = ADC1
          | (0 << MUX1)
          | (0 << MUX2)
          | (0 << MUX3)
          ;

   // start first adc conversion
   //
   ADCSRA |= (1 << ADSC);
   
   // read adc once to put it in a clean state
   ADC_read_battery();
   }

// Get battery reading, in volts.
//
PUBLIC FLOAT
BATTERY_read()
   {
   return ADC_battery_counts_to_volts(ADC_read_battery());
   }

//                   LIPO Resting Voltages
// --------------------------------------------------------------
// 1 cell pack     2 cell pack     3 cell pack     4 cell pack
// 4.20v = 100%    8.40v = 100%    12.60v = 100%   16.80v = 100%
// 4.03v = 76%     8.06v = 76%     12.09v = 76%    16.12v = 76%
// 3.86v = 52%     7.72v = 52%     11.58v = 52%    15.44v = 52%
// 3.83v = 42%     7.66v = 42%     11.49v = 42%    15.32v = 42%
// 3.79v = 30%     7.58v = 30%     11.37v = 30%    15.16v = 30%
// 3.70v = 11%     7.40v = 11%     11.10v = 11%    14.80v = 11%
// 3.6?v = 00%     7.2?v = 00%     10.8?v = 00%    14.4?v = 00%

// Is battery voltage lower than "warning" threshold?
//
PUBLIC BOOL
BATTERY_low()
   {
   return BATTERY_read() <= 7.60; // 40% capacity
   }

// Is battery voltage lower than "power off" threshold?
//
PUBLIC BOOL
BATTERY_critical()
   {
   return BATTERY_read() <= 7.45; // 15% capacity
   }
