/*
 * metal_detector_avr_register.ino
 *
 * AVR register-based rewrite of metal_detector_fixed.ino
 *
 * Main idea:
 * - Timer1 counts external pulses from detector oscillator on D5 / PD5 / T1.
 * - Timer2 creates a 1 ms system tick.
 * - Every 100 ms, Timer1 count is read and reset.
 * - During calibration, several 100 ms readings are averaged.
 * - Serial output format remains:
 *
 *      difference,hit
 *
 * Example:
 *      0,0
 *      2,0
 *      7,1
 *
 * Board:
 * - Arduino Nano / Uno with ATmega328P, 16 MHz
 */

#ifndef F_CPU
#define F_CPU 16000000UL
#endif

#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdint.h>

// ===================== Configuration =====================

#define BAUD_RATE 115200UL

#define THRESHOLD 3UL

#define GATE_MS 100UL
#define CALIBRATION_MS 1500UL

// LED pins:
// A5 = PC5 = red
// A4 = PC4 = green
// A2 = PC2 = blue
#define RED_LED_BIT    PC5
#define GREEN_LED_BIT  PC4
#define BLUE_LED_BIT   PC2

// Detector oscillator input:
// D5 = PD5 = T1 external clock input for Timer1
#define DETECTOR_INPUT_BIT PD5

// ===================== Global variables =====================

volatile uint32_t system_ms = 0;
volatile uint32_t timer1_overflows = 0;

uint32_t baseline = 0;
uint8_t calibrated = 0;

uint32_t gate_start_ms = 0;

// ===================== Interrupts =====================

// Timer2 Compare Match A interrupt: fires every 1 ms
ISR(TIMER2_COMPA_vect) {
  system_ms++;
}

// Timer1 overflow interrupt: used only as safety if count exceeds 65535
ISR(TIMER1_OVF_vect) {
  timer1_overflows++;
}

// ===================== Small time functions =====================

uint32_t millis_reg() {
  uint32_t value;

  uint8_t old_sreg = SREG;
  cli();
  value = system_ms;
  SREG = old_sreg;

  return value;
}

void delay_ms_reg(uint32_t ms) {
  uint32_t start = millis_reg();

  while ((millis_reg() - start) < ms) {
    // busy wait
  }
}

// ===================== USART0 functions =====================

void uart_init() {
  /*
   * Use double-speed mode for better 115200 baud accuracy.
   *
   * UBRR = F_CPU / (8 * BAUD) - 1
   * For 16 MHz and 115200:
   * UBRR ≈ 16
   */

  uint16_t ubrr = (F_CPU / (8UL * BAUD_RATE)) - 1;

  UCSR0A = (1 << U2X0);                 // double speed mode
  UBRR0H = (uint8_t)(ubrr >> 8);
  UBRR0L = (uint8_t)(ubrr);

  UCSR0B = (1 << RXEN0) | (1 << TXEN0); // enable RX and TX

  // 8 data bits, no parity, 1 stop bit
  UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);
}

uint8_t uart_available() {
  return (UCSR0A & (1 << RXC0));
}

char uart_read_char() {
  return UDR0;
}

void uart_write_char(char c) {
  while (!(UCSR0A & (1 << UDRE0))) {
    // wait until transmit buffer is empty
  }

  UDR0 = c;
}

void uart_print(const char *s) {
  while (*s) {
    uart_write_char(*s++);
  }
}

void uart_print_u32(uint32_t value) {
  char buffer[11];
  uint8_t i = 0;

  if (value == 0) {
    uart_write_char('0');
    return;
  }

  while (value > 0 && i < sizeof(buffer)) {
    buffer[i++] = '0' + (value % 10);
    value /= 10;
  }

  while (i > 0) {
    uart_write_char(buffer[--i]);
  }
}

void uart_println(const char *s) {
  uart_print(s);
  uart_write_char('\r');
  uart_write_char('\n');
}

void uart_println_u32(uint32_t value) {
  uart_print_u32(value);
  uart_write_char('\r');
  uart_write_char('\n');
}

// ===================== LED functions =====================

void leds_init() {
  // Set PC5, PC4, PC2 as outputs
  DDRC |= (1 << RED_LED_BIT) | (1 << GREEN_LED_BIT) | (1 << BLUE_LED_BIT);

  // Start with all LEDs off
  PORTC &= ~((1 << RED_LED_BIT) | (1 << GREEN_LED_BIT) | (1 << BLUE_LED_BIT));
}

void silence() {
  PORTC &= ~((1 << RED_LED_BIT) | (1 << GREEN_LED_BIT) | (1 << BLUE_LED_BIT));
}

void red_on() {
  PORTC |= (1 << RED_LED_BIT);
  PORTC &= ~((1 << GREEN_LED_BIT) | (1 << BLUE_LED_BIT));
}

void green_on() {
  PORTC |= (1 << GREEN_LED_BIT);
  PORTC &= ~((1 << RED_LED_BIT) | (1 << BLUE_LED_BIT));
}

void blue_on() {
  PORTC |= (1 << BLUE_LED_BIT);
  PORTC &= ~((1 << RED_LED_BIT) | (1 << GREEN_LED_BIT));
}

void startup_led_test() {
  red_on();
  delay_ms_reg(150);
  silence();

  green_on();
  delay_ms_reg(150);
  silence();

  blue_on();
  delay_ms_reg(150);
  silence();
}

// ===================== Timer setup =====================

void timer2_1ms_init() {
  /*
   * Timer2 CTC mode for 1 ms interrupt.
   *
   * F_CPU = 16 MHz
   * Prescaler = 64
   * Timer frequency = 16,000,000 / 64 = 250,000 Hz
   * 1 ms = 250 counts
   * OCR2A = 249 because counter starts from 0
   */

  TCCR2A = 0;
  TCCR2B = 0;
  TCNT2 = 0;

  OCR2A = 249;

  TCCR2A |= (1 << WGM21);   // CTC mode
  TCCR2B |= (1 << CS22);    // prescaler 64

  TIMSK2 |= (1 << OCIE2A);  // enable compare match interrupt
}

void timer1_external_counter_init() {
  /*
   * Timer1 counts external pulses on T1 pin.
   *
   * T1 pin on ATmega328P:
   * - Arduino D5
   * - Port D, bit 5
   *
   * CS12:CS10 = 111 means external clock source on T1, rising edge.
   */

  // D5 / PD5 as input
  DDRD &= ~(1 << DETECTOR_INPUT_BIT);

  // Disable internal pull-up
  PORTD &= ~(1 << DETECTOR_INPUT_BIT);

  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1 = 0;

  timer1_overflows = 0;

  // Enable Timer1 overflow interrupt
  TIMSK1 |= (1 << TOIE1);

  // External clock source on T1 pin, rising edge
  TCCR1B |= (1 << CS12) | (1 << CS11) | (1 << CS10);
}

uint32_t timer1_read_and_reset() {
  uint32_t total;
  uint16_t count16;
  uint32_t overflows;

  uint8_t old_sreg = SREG;
  cli();

  count16 = TCNT1;
  overflows = timer1_overflows;

  /*
   * If an overflow happened but ISR has not executed yet,
   * include it manually.
   */
  if ((TIFR1 & (1 << TOV1)) && count16 < 65535) {
    overflows++;
  }

  total = (overflows * 65536UL) + count16;

  TCNT1 = 0;
  timer1_overflows = 0;

  // Clear pending overflow flag by writing logic 1
  TIFR1 |= (1 << TOV1);

  SREG = old_sreg;

  return total;
}

void start_new_gate() {
  timer1_read_and_reset();
  gate_start_ms = millis_reg();
}

// ===================== Detector logic =====================

void calibrate() {
  silence();
  blue_on();   // blue = calibrating

  uint32_t sum = 0;
  uint16_t samples = 0;

  start_new_gate();

  uint32_t start = millis_reg();
  uint32_t gate_start = start;

  while ((millis_reg() - start) < CALIBRATION_MS) {
    uint32_t now = millis_reg();

    if ((now - gate_start) >= GATE_MS) {
      uint32_t count = timer1_read_and_reset();

      sum += count;
      samples++;

      gate_start = now;
    }
  }

  if (samples > 0) {
    baseline = sum / samples;
    calibrated = 1;

    silence();

    // Green flash = calibration OK
    green_on();
    delay_ms_reg(400);
    silence();

    uart_print("BASELINE=");
    uart_println_u32(baseline);
  } else {
    calibrated = 0;

    // Red flash = calibration failed
    red_on();
    delay_ms_reg(400);
    silence();

    uart_println("BASELINE_FAIL");
  }

  start_new_gate();
}

void process_detector_reading() {
  uint32_t now = millis_reg();

  if ((now - gate_start_ms) < GATE_MS) {
    return;
  }

  uint32_t count = timer1_read_and_reset();
  gate_start_ms = now;

  uint32_t difference;

  if (baseline >= count) {
    difference = baseline - count;
  } else {
    difference = count - baseline;
  }

  uint8_t hit = (difference >= THRESHOLD) ? 1 : 0;

  // Serial format: difference,hit
  uart_print_u32(difference);
  uart_write_char(',');
  uart_write_char(hit ? '1' : '0');
  uart_write_char('\r');
  uart_write_char('\n');

  // LED feedback
  if (hit) {
    if (difference > 10) {
      red_on();      // strong hit
    } else {
      green_on();    // weak hit
    }
  } else {
    silence();
  }
}

// ===================== Arduino setup / loop =====================

void setup() {
  cli();

  leds_init();
  uart_init();
  timer2_1ms_init();
  timer1_external_counter_init();

  sei();

  startup_led_test();

  calibrate();
}

void loop() {
  // Recalibrate when Jetson / Serial Monitor sends 'r' or 'R'
  if (uart_available()) {
    char c = uart_read_char();

    if (c == 'r' || c == 'R') {
      uart_println("RECALIBRATING...");
      calibrate();
      return;
    }
  }

  if (!calibrated) {
    return;
  }

  process_detector_reading();
}