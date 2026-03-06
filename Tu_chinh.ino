#include <Wire.h>
#include <MPU6050_tockn.h>
MPU6050 mpu6050(Wire);

#define PWM_1         9
#define DIR_1         7
#define ENC_1         2
#define ENC_2         3
#define BRAKE         8
#define BUZZER        12
#define VBAT          A7

bool vertical = false;
bool calibrating = false;
bool calibrated = false;

float loop_time = 10;
float e=0,e_d=0,e_i=0,RoAngle, RoAngle_offset,RoAngle_sum,preRoAngle;

float Kp=100;
//float Ki=0.009;
float Kd = 0.13;


volatile int  motor_speed=0, enc_count = 0;
volatile byte pos;

int bat_divider = 58; // this value needs to be adjusted to measure the battery voltage correctly

long currentT, previousT_1, previousT_2 = 0;  

void writeTo(byte device, byte address, byte value) {
  Wire.beginTransmission(device);
  Wire.write(address);
  Wire.write(value);
  Wire.endTransmission(true);
}

void beep() {
    digitalWrite(BUZZER, HIGH);
    delay(70);
    digitalWrite(BUZZER, LOW);
    delay(80);
}

void Motor1_control(int sp) {
  if (sp > 0) digitalWrite(DIR_1, LOW);
    else digitalWrite(DIR_1, HIGH);
  analogWrite(PWM_1, 255 - abs(sp));
}

void printRoAngle() {
    Serial.print("Robot Angle current value: ");
    Serial.println(RoAngle);
}

void angle_calc() {
  mpu6050.update();
  RoAngle=mpu6050.getAngleX();
  if (abs(e) > 15) {vertical = false;  digitalWrite(DIR_1, !digitalRead(DIR_1)); delay(100);}
  if (abs(e) < 2) vertical = true;
  if (abs(e) >= 30) {vertical = false; Motor1_control(0); digitalWrite(BRAKE, LOW); }
  printRoAngle();
}

void angle_setup() {
  Wire.begin();
  delay(100);
  for (int i = 0; i < 1024; i++) {
    angle_calc();
    RoAngle_sum += RoAngle;
  }
  RoAngle_offset = RoAngle_sum/1024;
  beep();
  beep();
  Serial.print("Robot angle offset: ");  Serial.println(RoAngle_offset);
  calibrated = true;
}

void battVoltage(double voltage) {
  if (voltage > 8 && voltage <= 9.5) {
    digitalWrite(BUZZER, HIGH);
  } else {
    digitalWrite(BUZZER, LOW);
  }
}

void ENC_READ() {
  byte cur = (!digitalRead(ENC_1) << 1) + !digitalRead(ENC_2);
  byte old = pos & B00000011;
  byte dir = (pos & B00110000) >> 4;
 
  if (cur == 3) cur = 2;
  else if (cur == 2) cur = 3;
 
  if (cur != old) {
  if (dir == 0) {
    if (cur == 1 || cur == 3) dir = cur;
    } else {
      if (cur == 0) {
        if (dir == 1 && old == 3) enc_count--;
        else if (dir == 3 && old == 1) enc_count++;
        dir = 0;
      }
    }
    pos = (dir << 4) + (old << 2) + cur;
  }
}

void setup() {
  Serial.begin(115200);
  Wire.begin();
  mpu6050.begin();
  mpu6050.calcGyroOffsets(true);

  // Pins D9 and D10 - 7.8 kHz
  TCCR1A = 0b00000001; 
  TCCR1B = 0b00001010; 


  pinMode(DIR_1, OUTPUT);  
  pinMode(BRAKE, OUTPUT);
  pinMode(BUZZER, OUTPUT);
  pinMode(ENC_1, INPUT);
  pinMode(ENC_2, INPUT);
  
  Motor1_control(0);

  attachInterrupt(0, ENC_READ, CHANGE);
  attachInterrupt(1, ENC_READ, CHANGE);
  beep();
  angle_setup();
  delay(1000);
}

void loop() {
  currentT = millis();

  if (currentT - previousT_1 >= loop_time) { 
    angle_calc();
    motor_speed = -enc_count;
    enc_count = 0;

    if (vertical && calibrated && !calibrating) {
      digitalWrite(BRAKE, HIGH);
      e=RoAngle-RoAngle_offset;
      e_d=(RoAngle-preRoAngle)/0.001;
      e_i=e_i+e*0.001;
      int pwm = constrain(Kp*e + Kd*e_d , -255, 255); 
      
      Motor1_control(-pwm);

    previousT_1 = currentT;
    preRoAngle = RoAngle;
  }
  
  if (currentT - previousT_2 >= 2000) {    
    battVoltage((double)analogRead(VBAT) / bat_divider); 
    if (!calibrated && !calibrating) {
      Serial.println("first you need to calibrate the balancing point...");
    }
    previousT_2 = currentT;
  }
}
}