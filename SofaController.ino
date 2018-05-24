#include <Arduino.h>
#include <Wire.h>
#include <ArduinoNunchuk.h>


#define PPM_NUM_CHANNELS 6
#define PPM_CHANNELS_FRAME_LENGHT_uS 22500
#define PPM_CHANNELS_PULSE_LENGTH_uS 500
#define OUTPUT_PIN 11

#define PPM_MIN_VALUE 500
#define PPM_MAX_VALUE 2500

#define WHEEL_FRONT_LEFT 0
#define WHEEL_BACK_LEFT 1

#define WHEEL_BACK_RIGHT 2
#define WHEEL_FRONT_RIGHT 3

#define FILTER 0.1
#define STEER_COEFFICIENT_MIN 0.30
#define STEER_COEFFICIENT_MAX 0.55
#define SPEED_COEFFICIENT 0.4

#define MAX_SPEED_FORWARD 1000
#define MAX_SPEED_BACKWARD -500

float steerCoefficient = 0.4;

byte currentChannelNumber;
uint16_t elapsedUs;
boolean state;
int16_t PPM_CHANNELS[PPM_NUM_CHANNELS];

int cmd1;
int cmd2;

int steer;
int speed;

int speedL;
int speedR;

ArduinoNunchuk nunchuk = ArduinoNunchuk();

uint16_t consecutiveFails = 0;

void setSpeed(uint8_t channel, int16_t speed) {
  setPPM(channel, map(speed, -1000, 1000, PPM_MIN_VALUE, PPM_MAX_VALUE));
}

void setPPM(uint8_t channel, uint16_t value) {
  PPM_CHANNELS[channel] = constrain(value, PPM_MIN_VALUE, PPM_MAX_VALUE);
}

float mapf(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void setup() {
  Serial.begin(115200);

  // Set everything to off!
  for (uint8_t i = 0; i < PPM_NUM_CHANNELS; i++) {
    PPM_CHANNELS[i] = 1500;
  }

  delay(100);

  nunchuk.begin();

  pinMode(OUTPUT_PIN, OUTPUT);
  digitalWrite(OUTPUT_PIN, LOW);

  // PPM SIGNAL GENERATION interrupt setup
  cli();
  TCCR1A = 0; // set entire TCCR1 register to 0
  TCCR1B = 0;

  OCR1A = 100;  // compare match register, change this
  TCCR1B |= (1 << WGM12);  // turn on CTC mode
  TCCR1B |= (1 << CS11);  // 8 prescaler: 0,5 microseconds at 16mhz
  TIMSK1 |= (1 << OCIE1A); // enable timer compare interrupt

  elapsedUs = 0;
  currentChannelNumber = 0;
  state = true;
  sei();
}

void drive(int cmd1, int cmd2) {
  steer = steer * (1.0 - FILTER) + cmd1 * FILTER;
  speed = speed * (1.0 - FILTER) + cmd2 * FILTER;

  // Change steering behaviour depending on speed
  steerCoefficient = mapf(constrain(abs(speed), 0, 1000), 0, 1000, STEER_COEFFICIENT_MAX, STEER_COEFFICIENT_MIN);

  speedR = constrain(speed * SPEED_COEFFICIENT - steer * steerCoefficient, MAX_SPEED_BACKWARD, MAX_SPEED_FORWARD);
  speedL = constrain(speed * SPEED_COEFFICIENT + steer * steerCoefficient, MAX_SPEED_BACKWARD, MAX_SPEED_FORWARD);

  setSpeed(WHEEL_FRONT_LEFT, speedL);
  setSpeed(WHEEL_BACK_LEFT, speedL * -1);

  setSpeed(WHEEL_FRONT_RIGHT, speedR);
  setSpeed(WHEEL_BACK_RIGHT, speedR * -1);
}

void loop() {
  delay(50);

  if (nunchuk.update()) {
    consecutiveFails = 0;

    cmd1 = constrain((nunchuk.analogX - 127) * 8, -1000, 1000);
    cmd2 = constrain((nunchuk.analogY - 128) * 8, -1000, 1000);

    // If backwards, invert steering
    if (cmd2 < 0) {
      cmd1 *= -1;
    }

    return drive(cmd1, cmd2);
  }

  // Failed to read nunchuck!
  consecutiveFails++;
  Serial.println("FAIL!");
  delay(20);

  if (consecutiveFails > 15) {
    Serial.println("SAFETY");

    // Ramp speed down as long as there's no signal
    while (!nunchuk.update()) {
      cmd1 = (cmd1 < 0) ? max(0, cmd1 + 10) : min(0, cmd1 - 10);
      cmd2 = (cmd2 < 0) ? max(0, cmd2 + 10) : min(0, cmd2 - 10);

      drive(cmd1, cmd2);
      delay(6);
    }

    // Try to reinitialize nunchuck!
    delay(1000);
    nunchuk.begin();
    delay(1000);
  }
}

// PPM SIGNAL GENERATION
// Please don't touch. Was hard to code :P
ISR(TIMER1_COMPA_vect) {
  TCNT1 = 0;

  if (state) {
    digitalWrite(OUTPUT_PIN, HIGH);
    OCR1A = PPM_CHANNELS_PULSE_LENGTH_uS * 2;

  } else {
    digitalWrite(OUTPUT_PIN, LOW);

    if (currentChannelNumber >= PPM_NUM_CHANNELS) {
      currentChannelNumber = 0;
      elapsedUs = elapsedUs + PPM_CHANNELS_PULSE_LENGTH_uS;
      OCR1A = (PPM_CHANNELS_FRAME_LENGHT_uS - elapsedUs) * 2;
      elapsedUs = 0;
    } else {
      OCR1A = (PPM_CHANNELS[currentChannelNumber] - PPM_CHANNELS_PULSE_LENGTH_uS) * 2;
      elapsedUs = elapsedUs + PPM_CHANNELS[currentChannelNumber];

      currentChannelNumber++;
    }
  }

  state = !state;
}

