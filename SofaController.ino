#include <Arduino.h>
#include <Wire.h>
#include <ArduinoNunchuk.h>


#define PPM_NUM_CHANNELS 6
#define PPM_CHANNELS_FRAME_LENGHT_uS 22500
#define PPM_CHANNELS_PULSE_LENGTH_uS 500
#define OUTPUT_PIN 11

#define PPM_MIN_VALUE 1000
#define PPM_MAX_VALUE 2500

#define WHEEL_FRONT_LEFT 0
#define WHEEL_BACK_LEFT 1

#define WHEEL_BACK_RIGHT 2
#define WHEEL_FRONT_RIGHT 3

#define FILTER 0.1
#define STEER_COEFFICIENT_MIN 0.30
#define STEER_COEFFICIENT_MAX 0.55

float STEER_COEFFICIENT = 0.4;
#define SPEED_COEFFICIENT 0.4


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

inline void setValue(uint8_t channel, uint16_t value) {
  PPM_CHANNELS[channel] = constrain(value, PPM_MIN_VALUE, PPM_MAX_VALUE);
}

float mapf(float x, float in_min, float in_max, float out_min, float out_max){
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

  state = true;
  elapsedUs = 0;
  currentChannelNumber = 0;

  cli();
  TCCR1A = 0; // set entire TCCR1 register to 0
  TCCR1B = 0;

  OCR1A = 100;  // compare match register, change this
  TCCR1B |= (1 << WGM12);  // turn on CTC mode
  TCCR1B |= (1 << CS11);  // 8 prescaler: 0,5 microseconds at 16mhz
  TIMSK1 |= (1 << OCIE1A); // enable timer compare interrupt

  state = true;
  sei();
}


void setSpeedLeft(int speed) {
  PPM_CHANNELS[WHEEL_FRONT_LEFT] = map(speed, -1000, 1000, 500, 2500);
  PPM_CHANNELS[WHEEL_BACK_LEFT] = map(speed * -1, -1000, 1000, 500, 2500);
}


void setSpeedRight(int speed) {
  PPM_CHANNELS[WHEEL_FRONT_RIGHT] = map(speed, -1000, 1000, 500, 2500);
  PPM_CHANNELS[WHEEL_BACK_RIGHT] = map(speed * -1, -1000, 1000, 500, 2500);
}

void drive(int cmd1, int cmd2) {
  steer = steer * (1.0 - FILTER) + cmd1 * FILTER;
  speed = speed * (1.0 - FILTER) + cmd2 * FILTER;

  speedR = constrain(speed * SPEED_COEFFICIENT - steer * STEER_COEFFICIENT, -1000, 1000);
  speedL = constrain(speed * SPEED_COEFFICIENT + steer * STEER_COEFFICIENT, -1000, 1000);

  setSpeedLeft(speedL);
  setSpeedRight(speedR);
}

void loop() {
  //put main code here


  if (nunchuk.update()) {
    cmd1 = constrain((nunchuk.analogX - 127) * 8, -500, 1000);
    cmd2 = constrain((nunchuk.analogY - 128) * 8, -500, 1000);

    STEER_COEFFICIENT = mapf(constrain(cmd2, 0, 1000), 0, 1000, STEER_COEFFICIENT_MAX, STEER_COEFFICIENT_MIN);

    if (cmd2 < 0) {
      cmd1 *= -1
    }

    Serial.print("X: ");
    Serial.println(nunchuk.analogX);

    Serial.print("Y: ");
    Serial.println(nunchuk.analogY);

    consecutiveFails = 0;

    drive(cmd1, cmd2);
  } else {

    consecutiveFails++;
    Serial.println("FAIL!");
    delay(20);

    if (consecutiveFails > 15) {

      Serial.println("SAFETY");
      

      while (!nunchuk.update()) {

        if (cmd1 < 0) {
          cmd1 = max(0, cmd1 + 10);
        } else if (cmd1 > 0) {
          cmd1 = min(0, cmd1 - 10);
        }

        if (cmd2 < 0) {
          cmd2 = max(0, cmd2 + 10);
        } else if (cmd2 > 0) {
          cmd2 = min(0, cmd2 - 10);
        }

        drive(cmd1, cmd2);
        delay(6);
      }

      delay(1000);
      nunchuk.begin();
      delay(1000);
    }
  }

  steer = steer * (1.0 - FILTER) + cmd1 * FILTER;
  speed = speed * (1.0 - FILTER) + cmd2 * FILTER;

  speedR = constrain(speed * SPEED_COEFFICIENT - steer * STEER_COEFFICIENT, -1000, 1000);
  speedL = constrain(speed * SPEED_COEFFICIENT + steer * STEER_COEFFICIENT, -1000, 1000);

  setSpeedLeft(speedL);
  setSpeedRight(speedR);

  delay(50);
}

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

