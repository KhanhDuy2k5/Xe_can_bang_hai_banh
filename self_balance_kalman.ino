// Khai bao //
#include<Wire.h>
#include "stmpu6050_kalman.h"
SMPU6050 mpu6050;

#define Step_R 12 //PORTD 5 //
#define Dir_R 13
#define En_R 8
#define Step_L 4 //PORTB 2 //
#define Dir_L 7
#define En_L 2
float input, output, input_last, Integral, outputL, outputR, M_L, M_R, MotorL, MotorR;
unsigned long lastTime = 0;
float offset = 1.2;
float Kp = 10;
float Ki = 0.1;
float Kd = 0.005;
int16_t currentSpeed = 0;
float filteredAngle = 0;

void pin_ID(){
pinMode(Step_R, OUTPUT);
pinMode(Dir_R, OUTPUT);
pinMode(En_R, OUTPUT);
pinMode(Step_L, OUTPUT);
pinMode(Dir_L, OUTPUT);
pinMode(En_L, OUTPUT);
digitalWrite(En_R, LOW);
digitalWrite(En_L, LOW);
}

void timer_INT(){
TCCR2A = 0;
TCCR2B = 0;
TCCR2B |= (1 << CS21);
OCR2A = 39;
TCCR2A |= (1 << WGM21);
TIMSK2 |= (1 << OCIE2A);
}

int8_t Dir_M1, Dir_M2; // if Dir_M1 = 0 => Mottor k chay //
volatile int16_t Count_timer1, Count_timer2; //Dung volatile int de khong bi sai lech gia tri bien//
volatile int32_t Step1, Step2;
int16_t Count_TOP1, Count_BOT1, Count_TOP2, Count_BOT2;

ISR(TIMER2_COMPA_vect){
  if(Dir_M1 != 0){
    Count_timer1++;

    if(Count_timer1 <= Count_TOP1)
      PORTB |= 0b00010000;      // D12 HIGH
    else
      PORTB &= 0b11101111;      // D12 LOW

    if(Count_timer1 > Count_BOT1){
      Count_timer1 = 0;

      if(Dir_M1 > 0)      Step1--;
      else if(Dir_M1 < 0) Step1++;
    }
  }

  if(Dir_M2 != 0){
    Count_timer2++;

    if(Count_timer2 <= Count_TOP2)
      PORTD |= 0b00010000;      // D4 HIGH
    else
      PORTD &= 0b11101111;      // D4 LOW

    if(Count_timer2 > Count_BOT2){
      Count_timer2 = 0;

      if(Dir_M2 > 0)      Step2++;
      else if(Dir_M2 < 0) Step2--;
    }
  }
}

void Speed_RIGHT(int16_t x){

  if(x < 0){
    Dir_M1 = -1;
    PORTB |= 0b00100000;        // D13 HIGH
  }
  else if(x > 0){
    Dir_M1 = 1;
    PORTB &= 0b11011111;        // D13 LOW
  }
  else{
    Dir_M1 = 0;
  }

  Count_BOT1 = abs(x);
  Count_TOP1 = abs(x)/2;
}

void Speed_LEFT(int16_t x){

  if(x < 0){
    Dir_M2 = -1;
    PORTD &= 0b01111111;        // D7 LOW
  }
  else if(x > 0){
    Dir_M2 = 1;
    PORTD |= 0b10000000;        // D7 HIGH
  }
  else{
    Dir_M2 = 0;
  }

  Count_BOT2 = abs(x);
  Count_TOP2 = abs(x)/2;
}

void setup() {
  mpu6050.init(0x68);
  Serial.begin(115200);
  pin_ID();
  timer_INT();
  delay(500);
  mpu6050.calibrate(500);
  delay(500);

  // de Kalman filter on dinh truoc khi vao loop chinh
  for(int i = 0; i < 500; i++)
  {
    mpu6050.getYAngle();
    delay(5);
  }
}

void loop() {
  unsigned long now = millis();
  if(now - lastTime < 5) return; // chi chay moi 5ms
  lastTime = now;

  float Angle_Y = mpu6050.getYAngle();
  if(isnan(Angle_Y) || isinf(Angle_Y) || abs(Angle_Y) > 90) {
      Wire.end();
      Wire.begin();
      mpu6050.init(0x68);
      return;
  }

  static float lastAngle = 0;
  if(abs(Angle_Y - lastAngle) > 10) {
      lastAngle = Angle_Y;
      return;
  }
  lastAngle = Angle_Y;
  Serial.println(Angle_Y);
  input = Angle_Y + offset;

  Integral += input;
  Integral = constrain(Integral, -1000, 1000); 

  output = Kp*input + Ki*Integral + Kd*(input - input_last);
  input_last = input;

  output = constrain(output, -400, 400);

  int16_t motorSpeed = 0;
  int16_t delay_x = 0;

  // 1. Giu nguyen dai toc do da tune chuan
  if (abs(output) > 4) {
    if (abs(output) <= 100) {
      motorSpeed = map(abs(output), 2, 100, 120, 40);
    } else {
      motorSpeed = map(abs(output), 100, 400, 40, 15);
    }

    motorSpeed = constrain(motorSpeed, 15, 120);

    // Gan lai dau de phan biet tien/lui
    delay_x = (output > 0) ? motorSpeed : -motorSpeed;
  }

  // 2. Khoi phuc tinh nang chong giat
  static int16_t last_delay_x = 0;
  if ((delay_x > 0 && last_delay_x < 0) || (delay_x < 0 && last_delay_x > 0)) {
    // Ep motor dung 1 nhip (5ms) khi qua moc 0 do de xa da
    delay_x = 0;
  }
  last_delay_x = delay_x;

  // 3. Day lenh ra 2 banh
  Speed_LEFT(delay_x);
  Speed_RIGHT(delay_x);

} // Ket thuc ham loop()
