#include <xc.h>
#include <stdint.h>

/**
 \file    dtmfpulse.c
 \brief   PIC16-based pulse-to-DTMF converter with memories
 \version 1.5
 \date    August 2015, modified June 2022
 \author  Alessandro Crespi <alessandro.crespi@epfl.ch>
*/

/*
    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program (see license.txt). If not, see
    <http://www.gnu.org/licenses/>.
*/

#pragma config CPD=OFF     // Data memory code protection is disabled
#pragma config BOREN=OFF   // Brown-out Reset disabled
#pragma config IESO=OFF    // Internal/External Switchover mode is disabled
#pragma config FOSC=INTOSC // INTOSC oscillator: I/O function on CLKIN pin
#pragma config FCMEN=OFF   // Fail-Safe Clock Monitor is disabled (no ext oscillator)
#pragma config MCLRE=ON    // MCLR/VPP pin function is MCLR
#pragma config WDTE=SWDTEN // WDT controlled by the SWDTEN bit in the WDTCON register
#pragma config CP=OFF      // Program memory code protection is disabled
#pragma config CLKOUTEN=OFF// CLKOUT function is disabled. I/O or oscillator function on the CLKOUT pin
#pragma config PWRTE=ON    // PWRT enabled
#pragma config PLLEN=OFF   // 4x PLL disabled
#pragma config WRT=OFF     // Write protection off
#pragma config STVREN=ON   // Stack Overflow or Underflow will cause a Reset
#pragma config BORV=LO     // Brown-out Reset Voltage (Vbor), low trip point selected
#pragma config LVP=OFF     // Low-voltage programming disabled, must use VPP high voltage

#define _XTAL_FREQ  16000000

#define SWITCH     (PORTCbits.RC0)
#define DIAL       (PORTCbits.RC1)

// comment to disable extended features (*/#, redial and memories)
#define USE_EXTENDED

#define TONE_LEN   6942    // DTMF tone length in PWM periods (= 40 us)
#define TONE_LEN_M 6000    // DTMF tone length when dialling from memory/redial
#define MEMORY_LEN   20    // max number of digits in a memory
#define BEEP_FREQ  1750    // frequency of beep tone
#define BEEP_LEN   2121    // length of beep when time for shift has been passed
#define SHIFT_TIME  100    // wait time for additional digits

/// single period of a sine wave, centered at 50, amplitude 100, 64 samples
const int8_t sine_64[64] = {
  50, 55, 60, 65, 69, 74, 78, 82, 85, 89, 92, 94, 96, 98, 99, 100, 100,
  100, 99, 98, 96, 94, 92, 89, 85, 82, 78, 74, 69, 65, 60, 55, 50, 45,
  40, 35, 31, 26, 22, 18, 15, 11, 8, 6, 4, 2, 1, 0, 0, 0, 1, 2, 4, 6, 8,
  11, 15, 18, 22, 26, 31, 35, 40, 45
};

// PWM frequency = 25 kHz
// BDA_period = Fout * 64 * 65536 [24 bit] / Fpwm (= 41.943127 * Fout)
const uint32_t waves_A[13] = { 39468, 29234, 29234, 29234, 32296, 32296, 32296, 35735, 35735, 35735, 0, 39468, 39468 };
const uint32_t waves_B[13] = { 56036, 50709, 56036, 61950, 50709, 56036, 61950, 50709, 56036, 61950, 0, 50709, 61950 };

void init()
{
  // configures internal oscillator for 16 MHz
  OSCCONbits.SPLLEN = 0;
  OSCCONbits.IRCF = 0b1111;
  OSCCONbits.SCS = 0b10;

  // waits for the oscillator to be stable
  while (!HFIOFS);

  // no analog inputs
  ANSELA = 0;
  ANSELC = 0;

  // outputs are all 0 by default
  LATA = 0;
  LATC = 0;

  // all pins input by default
  TRISA = 0xFF;
  TRISC = 0xFF;

  WDTCON = 0b00011011;  // enable WDT, 8.192s reset time

  CCPTMRS0 = (CCPTMRS0 & 0b11111100);  // make sure CCP1 uses timer 2
  CCP1CON = 0b00001100;  // PWM mode, single output
  CCP1AS = 0;            // disable PWM auto-shutdown
  T2CON = 0b00000100;    // enable timer 2, prescaler 1:1, 1:1 postscaler
  PR2 = 159;             // sets timer period for 25 kHz PWM  (Fosc / 4 / prescale / (PR2 + 1))

  T4CON = 0b01001110;    // enable timer 4, 1:16 prescaler & 1:10 postscaler (250 kHz @ 16 MHz)
  PR4 = 249;             // sets timer4 interrupt period to 10 ms (postscaler is 10)

  // selects RC5 (PWM) as output
  TRISC = TRISC & ~(1 << 5);

  TMR2IE = 0;
  TMR2IF = 0;
  TMR4IE = 1;
  TMR4IF = 0;
  PEIE = 1;
  ei();

  CCPR1L = 0;
}

// timer to measure event length (10 ms resolution)
volatile int16_t time_counter = 0;

void interrupt isr()
{
  if (TMR4IF) {
    time_counter++;
    TMR4IF = 0;
  }
}

uint8_t read_eeprom(uint8_t addr)
{
  EECON1 = 0;
  EEADRL = addr;
  EECON1bits.RD = 1;
  return EEDATL;
}

void write_eeprom(uint8_t addr, uint8_t value)
{
  EECON1 = 0;
  EEADRL = addr;
  EEDATL = value;
  EECON1bits.WREN = 1;
  EECON2 = 0x55;
  EECON2 = 0xAA;
  EECON1bits.WR = 1;
  EECON1bits.WREN = 0;
  while (EECON1bits.WR != 0);
  EEIF = 0;
}

// 24-bit counters and indexes (MSB of counter) directly declared with memory
// locations so we can very quickly access the MSB (the synthesis function is
// time critical at 16 MHz) without relying on the compiler properly optimizing
// bit shift operations
uint24_t counter_A @ 0x22;
uint24_t counter_B @ 0x26;
uint8_t idx_A @ 0x24;
uint8_t idx_B @ 0x28;

// generates a sine wave using a PWM DDS
// see algorithm: http://www.romanblack.com/one_sec.htm#BDA
void do_sine(uint16_t freq, uint16_t len)
{
  uint8_t idx;
  uint16_t pwm;
  uint32_t wave_A;

  // approximation of floor(freq * 167.77216)
  wave_A = 49250L * freq;
  wave_A = wave_A >> 8;
  counter_A = 0;

  di();
  while (len != 0)
  {
    while (!TMR2IF);  // sync with PWM (wait for timer2 interrupt flag)
    TMR2IF = 0;

    counter_A += wave_A;
    idx = (idx_A) & 0x3f;   // extract high byte of 24-bit counter
    pwm = sine_64[idx];
    pwm = (pwm * 2) + 319;

    CCP1CONbits.DC1B = (pwm & 0x03);
    CCPR1L = (pwm >> 2);
    len--;
  }
  CCPR1L = 0;
  CCP1CONbits.DC1B = 0;
  ei();
}

// same as do_sine(), but for DTMF digits
void do_digit(uint8_t digit, uint16_t len)
{
  uint8_t idx;
  uint16_t pwm;
  uint32_t wave_A, wave_B;

  wave_A = waves_A[digit] << 2;
  wave_B = waves_B[digit] << 2;

  counter_A = 0;
  counter_B = 0;

  di();  // you really don't want interrupts now
  while (len != 0)
  {
    while (!TMR2IF);  // sync with PWM (wait for timer2 interrupt flag)
    TMR2IF = 0;

    counter_A += wave_A;
    counter_B += wave_B;
    idx = (idx_A) & 0x3f;   // extract high byte of 24-bit counter
    pwm = sine_64[idx];
    idx = (idx_B) & 0x3f;
    pwm += sine_64[idx];
    pwm = pwm + 319;

    CCP1CONbits.DC1B = (pwm & 0x03);
    CCPR1L = (pwm >> 2);
    len--;
  }
  CCPR1L = 0;
  CCP1CONbits.DC1B = 0;
  ei();
}

// reads a digit from the pulse input
int8_t get_digit()
{
  int8_t c = 0, state = 0;
  int8_t shift = 0;

  while (SWITCH == 1) {
    CLRWDT();
  }
  time_counter = 0;

  __delay_ms(10);
  while (SWITCH == 0) {
    CLRWDT();
#ifdef USE_EXTENDED
    if (c == 0 && time_counter == SHIFT_TIME) {
      shift = 1;
      do_sine(BEEP_FREQ, BEEP_LEN);
    }
 #endif
    if (DIAL == 0 && state == 1) {
      state = 0;
      c++;
      __delay_ms(10);
    }
    if (DIAL == 1 && state == 0)  {
      state = 1;
      __delay_ms(10);
    }
  }
  if (state == 1) {
    c++;
  }
#ifdef USE_EXTENDED
  // time based "shift" to have more than 10 digits (*, #, redial, memories)
  if (shift) {
    c += 10;
  }
#endif
  return c;
}

#ifdef USE_EXTENDED
/// \brief dials the contents of a memory
/// \warning no check of parameter validity
void dial_memory(uint8_t mem)
{
  mem = MEMORY_LEN * mem;
  for (uint8_t i = 0; i < MEMORY_LEN; i++) {
    uint8_t digit = read_eeprom(mem + i);
    if (digit != 0xff) {
      do_digit(digit, TONE_LEN_M);
      __delay_ms(30);
    } else {
      break;
    }
  }
}

/// \brief saves the specified number to a memory
/// \warning no check of parameter
void save_memory(uint8_t mem, uint8_t* data, const uint8_t len)
{
  mem = MEMORY_LEN * mem;
  for (uint8_t i = 0; i < len; i++) {
    write_eeprom(mem + i, data[i]);
  }
  if (len < MEMORY_LEN) {
    write_eeprom(mem + len, 0xff);
  }
}
#endif

int main()
{
  uint8_t digit, cnt = 0;
  uint8_t last[MEMORY_LEN];

  init();

  while (1) {
    digit = get_digit();
    if (digit != 0) {
      if (digit == 10) {
        digit = 0;
      }
      if (digit <= 12) {
        do_digit(digit, TONE_LEN);             // makes the DTMF tone of the digit
        last[cnt] = digit;
#ifdef USE_EXTENDED        
        write_eeprom(cnt, digit);              // updates memory 0 with each new digit
        write_eeprom(cnt + 1, 0xff);           // (for last number redial)
#endif
        cnt++;
      }
#ifdef USE_EXTENDED      
      else if (digit == 20) {
        dial_memory(0);                        // dial memory 0 (= last dialled number)
      } else if (digit >= 13) {
        if (cnt == 0) {
          dial_memory(digit - 12);             // dial memory
        } else {
          if (cnt <= MEMORY_LEN) {             // save to memory
            save_memory(digit - 12, last, cnt);
            do_sine(880, TONE_LEN / 2);
            do_sine(1319, TONE_LEN / 2);
          } else {
            for (uint8_t i = 0; i < 3; i++) {  // too many digits, can't save
              do_sine(880, TONE_LEN / 2);
              __delay_ms(40);
            }
          }
        }
      }
#endif      
    }
    CLRWDT();
  }
}
