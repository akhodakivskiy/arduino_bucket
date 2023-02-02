#include <Arduino.h>

#include "sbus.h"
#include "PinChangeInt.h"

#define PIN_1 0
#define PIN_2 1
#define PIN_3 2
#define PIN_4 3
#define PIN_5 4
#define PIN_6 5
#define PIN_7 6
#define PIN_8 7

#define CHANNEL_COUNT 8

#define PIN_SUBS 10

uint8_t rc_pins[] { PIN_1, PIN_2, PIN_3, PIN_4, PIN_5, PIN_6, PIN_7, PIN_8 };
uint8_t rc_flags[] { 1, 2, 4, 8, 16, 32, 64, 128 };
uint16_t rc_values[] { 0, 0, 0, 0, 0, 0, 0, 0 };

volatile uint8_t rc_shared_flags;
volatile uint16_t rc_shared_values[CHANNEL_COUNT];
volatile uint32_t rc_shared_ts[CHANNEL_COUNT];

bfs::SbusTx sbus(&Serial, true);
bfs::SbusData sbusData;

void rc_channel_change(uint8_t id) {
  if (digitalRead(rc_pins[id]) == HIGH) {
    rc_shared_ts[id] = micros();
  }
  else {
    rc_shared_values[id] = (uint16_t)(micros() - rc_shared_ts[id]);
    rc_shared_flags |= rc_flags[id];
  }
}

void rc_aileron_change()  { rc_channel_change(0); }
void rc_elevator_change() { rc_channel_change(1); }
void rc_throttle_change() { rc_channel_change(2); }
void rc_rudder_change()   { rc_channel_change(3); }
void rc_aux1_change()     { rc_channel_change(4); }
void rc_aux2_change()     { rc_channel_change(5); }
void rc_aux3_change()     { rc_channel_change(6); }
void rc_aux4_change()     { rc_channel_change(7); }

void rc_setup_interrupts() {
  PCintPort::attachInterrupt(rc_pins[0], &rc_aileron_change, CHANGE);
  PCintPort::attachInterrupt(rc_pins[1], &rc_elevator_change, CHANGE);
  PCintPort::attachInterrupt(rc_pins[2], &rc_throttle_change, CHANGE);
  PCintPort::attachInterrupt(rc_pins[3], &rc_rudder_change, CHANGE);
  PCintPort::attachInterrupt(rc_pins[4], &rc_aux1_change, CHANGE);
  PCintPort::attachInterrupt(rc_pins[5], &rc_aux2_change, CHANGE);
  PCintPort::attachInterrupt(rc_pins[6], &rc_aux3_change, CHANGE);
  PCintPort::attachInterrupt(rc_pins[7], &rc_aux4_change, CHANGE);
}

void rc_process_channels() {
  static uint8_t flags;

  if (rc_shared_flags) {
    noInterrupts();
    flags = rc_shared_flags;

    if (flags & rc_flags[0]) rc_values[0] = rc_shared_values[0];
    if (flags & rc_flags[1]) rc_values[1] = rc_shared_values[1];
    if (flags & rc_flags[2]) rc_values[2] = rc_shared_values[2];
    if (flags & rc_flags[3]) rc_values[3] = rc_shared_values[3];
    if (flags & rc_flags[4]) rc_values[4] = rc_shared_values[4];
    if (flags & rc_flags[5]) rc_values[5] = rc_shared_values[5];
    if (flags & rc_flags[6]) rc_values[6] = rc_shared_values[6];
    if (flags & rc_flags[7]) rc_values[7] = rc_shared_values[7];

    rc_shared_flags = 0;
    interrupts();
  }

  flags = 0;
}

void setup() {
  rc_setup_interrupts();

  sbus.Begin();
}

long nextWriteMillis = 0;

void loop()
{
  rc_process_channels();

  long time = millis();

  if (nextWriteMillis < time) {

    sbusData.failsafe = false;
    sbusData.lost_frame = false;
    sbusData.ch17 = false;
    sbusData.ch18 = false;

    for (int i = 0; i < CHANNEL_COUNT; i++) {
      sbusData.ch[i] = map(rc_values[i], 1000, 2000, 172, 1811);
    }

    sbus.data(sbusData);

    sbus.Write();

    nextWriteMillis = time + 7;
  }
}
