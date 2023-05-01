#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/atomic.h>

void init() {
  DDRB |= (1<<DDB1) | (1<<DDB2) | (1<<DDB4) | (1<<DDB3);        // DDB0=DI (PB0) DDB1=DO (PB1) DDB2=SCK (PB2) DDB4=CS (PB4) DDBB=RCLK (PB3)
  PORTB = 0;
  USICR |= (1<<USIWM0) | (1<<USICS1) | (1<<USICLK);             // Three Wire mode, External Positive edge Clock Source
}

void clock(int bytes) {
  for(int i=0; i < bytes; i++) {            // Clock for amount of bytes selected
    USISR |= (1<<USIOIF);                   // Clear counter overflow flag
    while((USISR & (1 << USIOIF))==0) {     // While no counter overflow (16 clocks for 8 bits data)
      USICR |= (1<<USITC);                  // Toggle clock
    }
  }
}

uint8_t transfer(uint8_t dataout, int bytes) {
  USIDR = dataout;                          // 8 bits data to be sent at first 8 rising edges clock
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    clock(bytes);
  }
  return USIDR;                             // Return last 8 bits of data received
}

void writeDigit(int digit) {
  // Array with 7 segment display encodings
  uint8_t encodings[] = {0b00111111, 0b00000110, 0b01011011, 0b01001111, 0b01100110, 0b01101101, 0b01111101, 0b00000111, 0b01111111, 0b01101111, 0b01110111, 0b01111100, 0b00111001, 0b01011110, 0b01111001, 0b01110001, 0b00000000, 0b10000000};
  uint8_t data = encodings[digit];
  PORTB &= ~(1<<PORTB3);                    // Chip Select (Shift out)
  transfer(data, 1);                        // Transfer 1 byte via SPI to Shift Out register
  PORTB |= (1<<PORTB3);                     // Chip Deselect (Shift out)
}

void sensorCalibrationData(unsigned int *T1, signed int *T2, signed int *T3) {
  PORTB &= ~(1<<PORTB4);                    // Chip Select (BMP280)
  int T1L = transfer(0x88, 2);              // Start at memory address 0x88 (calibration data location: 0xA1...0x88)
  int T1H = transfer(0, 1);
  int T2L = transfer(0, 1);
  int T2H = transfer(0, 1);
  int T3L = transfer(0, 1);
  int T3H = transfer(0, 1);
  PORTB |= (1<<PORTB4);                     // Chip Deselect (BMP280)
  // 3x 16 bits calibration data
  *T1 = (T1H<<8) | T1L;
  *T2 = (T2H<<8) | T2L;
  *T3 = (T3H<<8) | T3L;
}

void sensorOptions() {
  PORTB &= ~(1<<PORTB4);                    // Chip Select (BMP280)
  transfer(0x74, 1);                        // Write to register 0xF4
  transfer(0b01000011, 1);                  // Transfer value to be written
  PORTB |= (1<<PORTB4);                     // Chip Deselect (BMP280)
}

float sensorReadTemp(unsigned int T1, signed int T2, signed int T3) {
  PORTB &= ~(1<<PORTB4);
  long adc_TH = transfer(0xFA, 2);          // Starting at data register with MSB of value
  long adc_TM = transfer(0, 1);
  long adc_TL = transfer(0, 1);
  PORTB |= (1<<PORTB4);
  long adc_T = (adc_TH<<12) | (adc_TM<<4) | (adc_TL>>4);        // 20 bits value for calculating temperature

  // Algorithm to calculate temperature
  long var1 = ( ((adc_T>>3) - ((long)T1<<1)) * ((long)T2) ) >> 11;
  long var2 = ( (( ((adc_T>>4) - ((long)T1)) * ((adc_T>>4) - ((long)T1)) )>>12) * ((long)T3) ) >> 14;
  float temp = ( var1 + var2 ) / 5120.0;

  return temp;                              // Temperature as float 25,08 °C
}

void setupTimerInterrupt(int freq) {
  cli();                                    // Disable interrupts
  int prescaler = 16384;                    // Value to be used in formula (should be same as set prescaler in TCCR1)
  TCCR1 |= (1<<CS13) | (1<<CS12) | (1<<CS11) | (1<<CS10) | (1<<CTC1);        // Set up prescaler
  OCR1A = F_CPU / prescaler / freq - 1;     // Formula for interrupt value
  OCR1C = OCR1A;                            // Interrupt when match with OCR1A register
  TIMSK |= (1<<OCIE1A);                     // Activate the interrupt
  sei();                                    // Enable interrupts
}

unsigned int T1; signed int T2; signed int T3;

int main() {
  // Setup
  init();
  sensorCalibrationData(&T1, &T2, &T3);
  sensorOptions();
  setupTimerInterrupt(2);

  while(true) {}                            // Halt

  return 0;
}

ISR(TIM1_COMPA_vect) {                      
  // Interrupt on Timer 1
  // Read out temperature (float) as 4 digits and write to 4 7 segment displays
  int tiental = (int)(sensorReadTemp(T1, T2, T3) / 10);
  int eenheid = (int)sensorReadTemp(T1, T2, T3) % 10;
  int tiende = (int)(sensorReadTemp(T1, T2, T3) * 10) % 10;
  int honderdste = (int)(sensorReadTemp(T1, T2, T3) * 100) % 10;
  writeDigit(honderdste);
  writeDigit(tiende);
  writeDigit(eenheid);  
  writeDigit(tiental);
}
