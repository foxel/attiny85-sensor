#include <avr/power.h>
#include <avr/wdt.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>

#include <noolite_tx.h>
#include "dht.h"

#define PWRPIN      1
#define RFPIN       0
#define DHTPIN      3
#define PIRPIN      2
#define BINDPIN     1
#define SENSOR_ADDR 0xAA56
#define TX_REPEATS  3

// each period is about 8 seconds
#define SLEEP_PERIODS 6

dht dht22;
NooliteTX tx(RFPIN, SENSOR_ADDR, TX_REPEATS);

int8_t cntr = 0;
int8_t oldPIR = HIGH;

void setup()
{
  DDRB = 0x00; // Set direction to input on all pins
  PORTB = 0xFF; // Enable pull-ups on pins 
  ADCSRA &= ~_BV(ADEN); // disable ADC

  // bind
  pinMode(BINDPIN, INPUT);
  if (digitalRead(BINDPIN) == HIGH) {
    pinMode(PWRPIN, OUTPUT);
    digitalWrite(PWRPIN, HIGH);
    tx.send_command(15);
    digitalWrite(PWRPIN, LOW);
  }
  pinMode(PIRPIN, INPUT_PULLUP);
  if (digitalRead(BINDPIN) == LOW) {
      // assume PIR sensor is in place and we don't need pull-up protection
      pinMode(PIRPIN, INPUT);
  }

  digitalWrite(PWRPIN, LOW);
  pinMode(PWRPIN, OUTPUT);

  power_all_disable();

  wdt_reset();
  wdt_enable(WDTO_8S);
  WDTCR |= _BV(WDIE);
  GIMSK |= _BV(PCIE);
  PCMSK |= _BV(PIRPIN);
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);
  sei();
}

ISR (WDT_vect) {
  WDTCR |= _BV(WDIE);
  if (cntr) {
    cntr--;
  }
}

// PIR sensor
ISR(PCINT0_vect) {
  // force sensor data transmission if the last sleep period is in progress
  if (cntr == 1) {
    cntr--;
  }
}

void loop()
{
  wdt_reset();
  lock_interrupts();
  // PIR
  int newPIR = digitalRead(PIRPIN);
  if (newPIR ^ oldPIR) {
    digitalWrite(PWRPIN, HIGH);
    tx.send_command(newPIR == HIGH ? 2 : 0);
    oldPIR = newPIR;
  }

  // sensor
  if (!cntr) {
    uint8_t battery_value = read_battery();

    if (battery_value < 150) {  // 3 volts
      digitalWrite(PWRPIN, HIGH);
      tx.send_command(20);
    } else if (dht22.read(DHTPIN) == DHTLIB_OK) {
      int deci_temp = int(dht22.temperature * 10);
      byte deci_hum = byte(dht22.humidity);

      uint8_t tx_args[] = {
        deci_temp & 0xff,
        (deci_temp >> 8) & 0x0f | 0x20,
        deci_hum,
        battery_value
      };

      wait(100); // Wait for power to settle
      digitalWrite(PWRPIN, HIGH);
      tx.send_command(21, tx_args, 4);
    } else {
      digitalWrite(PWRPIN, HIGH);
      tx.send_command(255);
    }
    cntr = SLEEP_PERIODS;
  }

  digitalWrite(PWRPIN, LOW);
  unlock_interrupts();
  power_all_disable();
  sleep_enable();
  sleep_bod_disable();
  sleep_cpu();
}

void lock_interrupts() {
  GIMSK &= ~_BV(PCIE);
}

void unlock_interrupts() {
  GIMSK |= _BV(PCIE);
}

uint8_t read_battery() {
  power_adc_enable();
  ADCSRA = _BV(ADEN); // eanable ADC
  ADMUX = _BV(MUX3) | _BV(MUX2); // Vcc as ref, 1.1 as input
  wait(100); // Wait for Vref to settle
  ADCSRA |= _BV(ADSC); // start ADC
  while(ADCSRA & (1<<ADSC)); // Wait for conversion to complete.
  uint8_t low = ADCL; // read value
  uint8_t high = ADCH; // ...
  int raw_value = (high << 8) | low;
  ADCSRA = 0x00; // disable ADC
  power_adc_disable();

  return 50 * 1.1 * 1024.0/raw_value;
}

// rough one
void wait(unsigned int ms) {
  if (ms < 65) {
    delayMicroseconds(ms * 1000);
  } else {
    while (ms--) delayMicroseconds(1000);
  }
}
