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

// each period is about 8 seconds
#define SLEEP_PERIODS 3

dht dht22;
NooliteTX tx(RFPIN, SENSOR_ADDR);

int8_t cntr = 0;
int8_t oldPIR = 0;

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
ISR(PCINT0_vect) {}

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
    digitalWrite(PWRPIN, HIGH);
    if (dht22.read(DHTPIN) == DHTLIB_OK) {
      int deci_temp = int(dht22.temperature * 10);
      byte deci_hum = byte(dht22.humidity);
  
      uint8_t tx_args[] = {
        deci_temp & 0xff,
        (deci_temp >> 8) & 0x0f | 0x20,
        deci_hum,
        0xff // can be some analog sensor
      };
    
      tx.send_command(21, tx_args, 4);
    } else {
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

