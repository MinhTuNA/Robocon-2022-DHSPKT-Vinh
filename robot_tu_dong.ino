#include <EEPROM.h>
#include <Arduino.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <Wire.h>
//#include <MPU6050_tockn.h>

#define ACCEL_XOUTH 0x3B
#define ACCEL_XOUTL 0x3C
#define ACCEL_YOUTH 0x3D
#define ACCEL_YOUTL 0x3E
#define ACCEL_ZOUTH 0x3F
#define ACCEL_ZOUTL 0x40

#define GYRO_XH 0x43
#define GYRO_XL 0x44

#define PWM_1 2
#define PWM_2 3
#define PWM_3 4
#define PWM_4 5

#define PWM_5 11 // dong co nang
#define PWM_6 12// dong co quay

#define DIR_1 26
#define DIR_2 27
#define DIR_3 28
#define DIR_4 29

#define DIR_5 31 
#define DIR_6 30

#define LEDY0 A0
#define LEDY1 A1
#define LEDY2 A2
#define LEDY3 A3
#define LEDY4 A4
#define LEDY5 A5
#define LEDY6 A6
#define LEDY7 A7

#define LEDX0 A8
#define LEDX1 A9
#define LEDX2 A10
#define LEDX3 A11
#define LEDX4 A12
#define LEDX5 A13
#define LEDX6 A14
#define LEDX7 A15


#define buttonY 38 // nut bam do line Y
#define sttY 41 // led bao trang thai do line Y
#define buttonX 39 // nut bam do line X
#define sttX 42 // led bao trang thai do line X
#define configMode 40

#define area 32
#define reset2 33
#define reset3 34
#define reset4 39
#define reset5 36

#define green 1
#define red 0

#define set_point 22
#define left 35
#define right 37
#define mid 50

#define sensor  53
#define sensor_X0 24
#define sensor_Y0 51
#define sensor_Y1 48
#define sensor_X1 52

#define ct_cot1 15
#define ct_cot2 16
#define ct_cot3 17

#define KN1 9// kn gap
#define KN2 10// kn tha
#define KN3 14// kn nang ha


#define CHANNEL_B_PIN_Y 2 //
#define CHANNEL_B_PIN_X 3 //

#define EXTERNAL_CLOCK_PIN 47  // Định nghĩa chân nguồn xung ngoài cho Timer 5
#define IN_T5 43


int i = 255; // độ rộng băm xung

bool att_KN6 = 1; 
bool tay_gap = 1;

int countY = 0; // biến đếm số lần nhấn nút X
int countX = 0; // biến đếm số lần nhấn nút Y

int ledYGR[8] = {}; // mảng giá trị ledY màu xanh 
int ledYWT[8] = {}; // mảng giá trị ledY màu trắng 
int numY[8] = {};// mảng sai số giữa 2 màu Y 


int nledY[8] = {}; // mảng giá trị ledY hiện tại 
int nledX[8] = {}; // mảng giá trị ledX hiện tại


int ADDRESS_LEDYGR = 1; // địa chỉ đầu lưu giá trị ledY màu xanh vào eeprom
int ADDRESS_NUMY = 10; // địa chỉ đầu lưu giá trị sai số giữa 2 màu tại ledY

int ADDRESS_LEDXGR = 20; // địa chỉ đầu lưu giá trị ledX màu xanh vào eeprom
int ADDRESS_NUMX = 30; // địa chỉ đầu lưu giá trị sai số giữa 2 màu tại ledX

int en = 1;

int encoderX = 0;
int encoderY = 0;

const int MPU = 0x68; // I2C MPU6050 addres
float GyroX, GyroY, GyroZ;
float roll, pitch, yaw, froll, fpitch, fyaw = 0;
float preYaw;
float elapsedTime, currentTime, previousTime;
float dt = 0.015;
bool turn_fillter = false;

 //MPU6050 mpu6050(Wire);

void setup() {

  pinMode(PWM_1,OUTPUT); // chân băm xung điều khiển tốc độ động cơ
  pinMode(PWM_2,OUTPUT);
  pinMode(PWM_3,OUTPUT);
  pinMode(PWM_4,OUTPUT);
  pinMode(PWM_5,OUTPUT);
  pinMode(PWM_6,OUTPUT);

  pinMode(DIR_1,OUTPUT);// chân điều khiển chiều quay động cơ
  pinMode(DIR_2,OUTPUT);
  pinMode(DIR_3,OUTPUT);
  pinMode(DIR_4,OUTPUT);
  pinMode(DIR_5,OUTPUT);
  pinMode(DIR_6,OUTPUT);

  pinMode(LEDY0,INPUT);
  pinMode(LEDY1,INPUT);
  pinMode(LEDY2,INPUT);
  pinMode(LEDY3,INPUT);
  pinMode(LEDY4,INPUT);
  pinMode(LEDY5,INPUT);
  pinMode(LEDY6,INPUT);
  pinMode(LEDY7,INPUT);
  
  pinMode(LEDX0,INPUT);
  pinMode(LEDX1,INPUT);
  pinMode(LEDX2,INPUT);
  pinMode(LEDX3,INPUT);
  pinMode(LEDX4,INPUT);
  pinMode(LEDX5,INPUT);
  pinMode(LEDX6,INPUT);
  pinMode(LEDX7,INPUT);

  pinMode(sensor_X0,INPUT);
  pinMode(sensor_X1,INPUT);
  pinMode(sensor_Y0,INPUT);
  pinMode(sensor_Y1,INPUT);
  pinMode(sensor,INPUT);

  pinMode(area,INPUT_PULLUP);
  pinMode(reset2,INPUT_PULLUP);
  pinMode(reset3,INPUT_PULLUP);
  pinMode(reset4,INPUT_PULLUP);
  pinMode(reset5,INPUT_PULLUP);

  pinMode(set_point,INPUT_PULLUP);
  pinMode(left,INPUT_PULLUP);
  pinMode(right,INPUT_PULLUP);
  pinMode(mid,INPUT_PULLUP);
  
  pinMode(ct_cot1,INPUT_PULLUP);
  pinMode(ct_cot2,INPUT_PULLUP);
  pinMode(ct_cot3,INPUT_PULLUP);

  pinMode(buttonY,INPUT_PULLUP);
  pinMode(sttY,OUTPUT);
  digitalWrite(sttY,HIGH);
  pinMode(buttonX,INPUT_PULLUP);
  pinMode(sttX,OUTPUT);
  digitalWrite(sttX,HIGH);
  pinMode(configMode,INPUT_PULLUP);

  pinMode(KN1,OUTPUT); // khí nén gắp 
  pinMode(KN2,OUTPUT); // khí nén thả
  pinMode(KN3,OUTPUT); // khí nén nâng hạ

  pinMode(IN_T5,OUTPUT);
  digitalWrite(IN_T5,LOW);

  digitalWrite(27,LOW);

  readSensorNumber(ledYGR,ADDRESS_LEDYGR); // đọc dữ liệu ledY green trong eeprom
  readSensorNumber(numY,ADDRESS_NUMY); // đọc sai số và truyền vào mảng numY
  //init_inputcapture5();
  init_timer5();
  //enICP5();
  sei();
  dung();
  digitalWrite(KN3,HIGH);
  gap();
  Serial.begin(9600);
  Wire.begin();
  Wire.beginTransmission(MPU);
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission();
}

void loop() {
  while(digitalRead(configMode)) // vao config mode
  {
    if(digitalRead(left) == 0)
    {
      quayphaiTaygap();
    }
    if(digitalRead(right) == 0)
    {
      quaytraiTaygap();
    }
    if(digitalRead(left) == 1 && digitalRead(right) == 1)
    {
      analogWrite(PWM_6,0);
    }
    scancolor();
  }
  

  while(Area() == red && !(digitalRead(configMode)) ) // san do
  {
    if(Reset4() && Reset5())
    {
    Serial.println("san do");
      Serial.println("khoi dong san do");
      if(turn_fillter == false)
      {
        filter_gyro();
        turn_fillter = true;
      }
      while(digitalRead(set_point) == 1) ha();
      analogWrite(PWM_5,0);
      if(Reset2() && Reset3() && Reset4() && Reset5() )
      {
      while(digitalRead(sensor) == 1)
      { 
      }
      }
      while(digitalRead(ct_cot1) == 1)
      {
        nang();
      }
      analogWrite(PWM_5,0);
      lui();
      tangtoc();
      do_line_Y_lui(4170);
      //while(TCNT5 <= 4170);
      dung();
      //delay(500);
      do_line_X();
      do_line_Y();
      CheckGyro();
      dung();
      //while(1);
      if(Reset2()) // neu cong tac 2 3 4 5khong gat
      {
      quaytraiTaygap();
      while(digitalRead(left) == 1);
      analogWrite(PWM_6,0);
      delay(200);
      dung();
      ha_tha_vat();
      delay(500);
      Check_sensor_TZ1();
      dung();
      delay(1000);
      tha(); // tha vat vao cot
      delay(500);
      gap();
      delay(500);
      tha();
      dung();
      nang_ct2();
      do_line_X();
      do_line_Y();
      CheckGyro();
      }
      //while(1);
      delay(500);
      digitalWrite(IN_T5,HIGH);
      delay(300);
      TCNT5 = 0;
      phai();
      tangtoc();
      delay(500);
      do_line_X_phai(1850);
      //while(TCNT5 <= 1850);
      dung(); 
      delay(500);
      CheckGyro();
      do_line_X(); 
      do_line_Y();
      //CheckGyro();
      delay(200);
      if(Reset3()) // neu cong tac 2 khong gat 
      {
      quayphaiTaygap();
      while(digitalRead(mid) == 1);
      analogWrite(PWM_6,0);
      delay(100);
      //nang_ct1();
      ha_ct1();
      delay(600);
      nang_ct1();
      gap();
      while(digitalRead(sensor) == 1); // nhận vật 2 từ robot cầm tay
      delay(1500);
      //ha_ct1();
      }
      TCNT5 = 0;
      phai();
      tangtoc();
      delay(500);
      do_line_X_phai(1750);
      dung();
      dung();
      delay(500);
      //CheckGyro();
      do_line_X();
      do_line_Y(); 
      //CheckGyro(); 
      delay(500);   
      dung();
      if(Reset3()) // neu cong tac 3 4 5 khong gat
      {
      quayphaiTaygap();
      while(digitalRead(right) == 1);
      analogWrite(PWM_6,0);
      delay(500);
      //CheckGyro();_sensor_TZ2();
      //ha_ct1();
      ha_tha_vat();
      delay(500);
      Check_sensor_TZ2();
      delay(1000);
      tha(); // tha vat cot 2///////////////////////////////
      delay(500);
      gap();
      delay(500);
      tha();
      delay(1000);
      do_line_X();
      do_line_Y();
      //CheckGyro();
      }
      nang_ct2();
      digitalWrite(IN_T5,LOW);
      delay(500);
      TCNT5 = 0;
      tien();
      tangtoc();
      delay(300);
      do_line_Y_tien(2320);
      //dung();
      dung();
      delay(500);
      //CheckGyro();
      do_line_Y();
      do_line_X(); 
      //CheckGyro();
      dung();
      quaytraiTaygap();
      while(digitalRead(left) == 1);
      analogWrite(PWM_6,0);
      ha_ct1();
      delay(500);
      nang_ct1();
      gap();
      while(digitalRead(sensor) == 1); // nhận vật 3
      delay(1500);
      
      nang_ct2();
      TCNT5 = 0;
      tien();
      tangtoc();
      delay(500);
      do_line_Y_tien(2320);
      dung();
      delay(200);
      //CheckGyro();
      do_line_Y();
      do_line_X();
      //CheckGyro();
      dung();
      //while(1);
      delay(300);
      ha_tha_vat();
      delay(500);
      Check_sensor_TZ1();
      delay(1000);
      tha(); // thả cột 3 //////////////////////
      delay(500);
      gap();
      delay(500);
      tha();
      delay(200);
      nang_ct3();
      do_line_X();
      do_line_Y();
      quayphaiTaygap();
      while(digitalRead(mid) == 1);
      analogWrite(PWM_6,0);
      ha_ct1();
      delay(500);
      nang_ct1();
      gap();
      while(digitalRead(sensor) == 1 ); // nhan vat 4
      digitalWrite(IN_T5,HIGH);
      delay(500);
      ha_ct2();
      delay(500);
      nang_ct2();
      TCNT5 = 0;
      phai();
      tangtoc();
      delay(1000);
      do_line_X_phai(1850);
      dung();
      delay(100);
      //CheckGyro();
      do_line_X(); 
      do_line_Y();
      //CheckGyro();
      delay(100);
      TCNT5 = 0;
      phai();
      tangtoc();
      delay(500);
      do_line_X_phai(1850);
      dung();
      dung(); 
      //CheckGyro1();
      do_line_X(); 
      do_line_Y();
      //CheckGyro1();
      delay(100);
      dung();
      //ha_ct2();
      ha_tha_vat();
      delay(500);
      Check_sensor_TZ4();
      delay(1000);
      tha(); // tha vat 4/////////////////////
      delay(500);
      gap();
      delay(500);
      tha();
      delay(100);
      do_line_X();
      do_line_Y();
      //CheckGyro1();
      nang_ct3();
      TCNT5 = 0;
      trai();
      tangtoc();
      while(TCNT5 <= 500);
      TCNT5 = 0;
      do_line_X_trai(1350);
      dung();
      delay(500);
      //CheckGyro1();
      do_line_X();
      do_line_Y();
      //CheckGyro1();
      do_line_X();
      do_line_Y();
      ha_ct1();
      delay(500);
      nang_ct1();
      gap();
      while(digitalRead(sensor) == 1 );
      delay(1500);
      nang_ct3();
      TCNT5 = 0;
      phai();
      tangtoc();
      do_line_X_phai(1350);
      TCNT5 = 0;
      while(TCNT5 <= 500);
      dung();
      //dung(); 
      delay(100);
      //CheckGyro1();
      do_line_X(); 
      do_line_Y();
      //CheckGyro1();
      
      digitalWrite(IN_T5,LOW);
      delay(100);
      TCNT5 = 0;
      lui();
      tangtoc();
      do_line_Y_lui_sanxanh(3720);
      dung();
      dung();
      delay(200);
      CheckGyro();
      //do_line_X();
      do_line_Y();
      CheckGyro();
      delay(500);
      quayphaiTaygap();
      while(digitalRead(right) == 1);
      analogWrite(PWM_6,0);
      delay(100);
      ha_tha_vat();
      delay(500);
      Check_sensor_TZ2();
      delay(1000);
      tha(); // thả vật vào cột 5
      delay(500);
      gap();
      delay(500);
      tha();
      delay(500);
      gap();
      delay(500);
      tha();
      nang_ct3();
      trai();
      speed(247,245,245,247);
      delay(700);
      dung();
      while(1);
    delay(1000);
    }
    if(!Reset4() && Reset5()) ///////////////// reset cot 4
    {
      if(turn_fillter == false)
      {
        filter_gyro();
        turn_fillter = true;
      }
      while(digitalRead(set_point) == 1) ha();
      analogWrite(PWM_5,0);
      while(digitalRead(sensor) == 1)
      { 
      }
      while(digitalRead(ct_cot1) == 1)
      {
        nang();
      }
      analogWrite(PWM_5,0);
      digitalWrite(IN_T5,HIGH);
      //delay(500);
      //ha_ct2();
      //delay(500);
      nang_ct2();
      TCNT5 = 0;
      phai();
      tangtoc();
      delay(1000);
      do_line_X_phai(1850);
      dung();
      delay(100);
      //CheckGyro();
      do_line_X(); 
      do_line_Y();
      //CheckGyro();
      delay(100);
      TCNT5 = 0;
      phai();
      tangtoc();
      delay(500);
      do_line_X_phai(1850);
      dung();
      dung(); 
      //CheckGyro1();
      do_line_X(); 
      do_line_Y();
      //CheckGyro1();
      delay(100);
      dung();
      //ha_ct2();
      ha_tha_vat();
      delay(500);
      Check_sensor_TZ4();
      delay(1000);
      tha(); // tha vat 4/////////////////////
      delay(500);
      gap();
      delay(500);
      tha();
      delay(100);
      do_line_X();
      do_line_Y();
      //CheckGyro1();
      nang_ct3();
      TCNT5 = 0;
      trai();
      tangtoc();
      while(TCNT5 <= 500);
      TCNT5 = 0;
      do_line_X_trai(1350);
      dung();
      delay(500);
      //CheckGyro1();
      do_line_X();
      do_line_Y();
      //CheckGyro1();
      do_line_X();
      do_line_Y();
      ha_ct1();
      delay(500);
      nang_ct1();
      gap();
      while(digitalRead(sensor) == 1 );
      delay(1500);
      nang_ct3();
      TCNT5 = 0;
      phai();
      tangtoc();
      do_line_X_phai(1350);
      TCNT5 = 0;
      while(TCNT5 <= 500);
      dung();
      //dung(); 
      delay(100);
      //CheckGyro1();
      do_line_X(); 
      do_line_Y();
      //CheckGyro1();
      
      digitalWrite(IN_T5,LOW);
      delay(100);
      TCNT5 = 0;
      lui();
      tangtoc();
      do_line_Y_lui_sanxanh(3720);
      dung();
      dung();
      delay(200);
      CheckGyro();
      //do_line_X();
      do_line_Y();
      CheckGyro();
      delay(500);
      quayphaiTaygap();
      while(digitalRead(right) == 1);
      analogWrite(PWM_6,0);
      delay(100);
      ha_tha_vat();
      delay(500);
      Check_sensor_TZ2();
      delay(1000);
      tha(); // thả vật vào cột 5
      delay(500);
      gap();
      delay(500);
      tha();
      delay(500);
      gap();
      delay(500);
      tha();
      nang_ct3();
      trai();
      speed(247,245,245,247);
      delay(700);
      dung();
      while(1);
    delay(1000);
    }
    if(Reset4() && !Reset5())
    {
        if(turn_fillter == false)
      {
        filter_gyro();
        turn_fillter = true;
      }
      while(digitalRead(set_point) == 1) ha();
      analogWrite(PWM_5,0);
      while(digitalRead(sensor) == 1)
      { 
      }
      while(digitalRead(ct_cot1) == 1)
      {
        nang();
      }
      analogWrite(PWM_5,0);
      digitalWrite(IN_T5,HIGH);
      gap();
      while(digitalRead(sensor) == 1 );
      delay(1500);
      nang_ct3();
      TCNT5 = 0;
      phai();
      tangtoc();
      do_line_X_phai(1350);
      TCNT5 = 0;
      while(TCNT5 <= 500);
      dung();
      //dung(); 
      delay(100);
      //CheckGyro1();
      do_line_X(); 
      do_line_Y();
      //CheckGyro1();
      
      digitalWrite(IN_T5,LOW);
      delay(100);
      TCNT5 = 0;
      lui();
      tangtoc();
      do_line_Y_lui_sanxanh(3720);
      dung();
      dung();
      delay(200);
      CheckGyro();
      //do_line_X();
      do_line_Y();
      CheckGyro();
      delay(500);
      quayphaiTaygap();
      while(digitalRead(right) == 1);
      analogWrite(PWM_6,0);
      delay(100);
      ha_tha_vat();
      delay(500);
      Check_sensor_TZ2();
      delay(1000);
      tha(); // thả vật vào cột 5
      delay(500);
      gap();
      delay(500);
      tha();
      delay(500);
      gap();
      delay(500);
      tha();
      nang_ct3();
      trai();
      speed(247,245,245,247);
      delay(700);
      dung();
      while(1);
    delay(1000);
    }
  }

  while(Area() == green && !(digitalRead(configMode)) ) // san xanh
  {
    Serial.println("san xanh");
    if(Reset4() && Reset5())
    {
    if(turn_fillter == false)
      {
        filter_gyro();
        turn_fillter = true;
      }
      while(digitalRead(set_point) == 1) ha();
      analogWrite(PWM_5,0);
      
      if(Reset2() && Reset3() )
      {
      while(digitalRead(sensor) == 1)
      { 
      }
      }
     
      while(digitalRead(ct_cot1) == 1)
      {
        nang();
      }
      analogWrite(PWM_5,0);
      lui();
      tangtoc();
      do_line_Y_lui_sanxanh(3170);
      //while(TCNT5 <= 4170);
      dung();
      TCNT5 = 0;
      lui();
      speed(245,241,241,245);
      while(TCNT5 <= 1000);
      dung();
      //delay(500);
      do_line_X();
      do_line_Y();
      CheckGyro();
      dung();
      //while(1);
      if(Reset2()) // neu cong tac 2 3 4 5khong gat
      {
      quayphaiTaygap();
      while(digitalRead(right) == 1);
      analogWrite(PWM_6,0);
      delay(200);
      dung();
      ha_tha_vat();
      delay(500);
      Check_sensor_TZ2();
      dung();
      delay(1000);
      tha(); // tha vat vao cot
      delay(500);
      gap();
      delay(500);
      tha();
      dung();
      }
      nang_ct2();
      do_line_X();
      do_line_Y();
      CheckGyro();
      //while(1);
      delay(500);
      digitalWrite(IN_T5,HIGH);
      delay(300);
      TCNT5 = 0;
      trai();
      tangtoc();
      delay(500);
      do_line_X_trai(1850);
      //while(TCNT5 <= 1850);
      dung(); 
      delay(500);
      CheckGyro();
      do_line_X(); 
      do_line_Y();
      //CheckGyro();
      delay(200);
      if(Reset3())
      {
      quaytraiTaygap();
      ha_ct1();
      while(digitalRead(mid) == 1);
      analogWrite(PWM_6,0);
      delay(100);
      nang_ct1();
      gap();
      while(digitalRead(sensor) == 1); // nhận vật 2 từ robot cầm tay
      delay(1500);
      }
      //ha_ct1();
      TCNT5 = 0;
      trai();
      tangtoc();
      do_line_X_trai(1750);
      dung();
      dung();
      delay(500);
      //CheckGyro();
      do_line_X();
      do_line_Y(); 
      //CheckGyro(); 
      delay(500);   
      dung();
      if(Reset3())
      {
      quaytraiTaygap();
      while(digitalRead(left) == 1);
      analogWrite(PWM_6,0);
      delay(500);
      //CheckGyro();_sensor_TZ2();
      //ha_ct1();
      ha_tha_vat();
      delay(500);
      Check_sensor_TZ1();
      delay(1000);
      tha(); // tha vat cot 2///////////////////////////////
      delay(500);
      gap();
      delay(500);
      tha();
      delay(1000);
      }
      nang_ct2();
      //delay(500);
      do_line_X();
      do_line_Y();
      //CheckGyro();
      digitalWrite(IN_T5,LOW);
      delay(500);
      TCNT5 = 0;
      tien();
      tangtoc();
      do_line_Y_tien(2320);
      //dung();
      dung();
      delay(500);
      //CheckGyro();
      do_line_Y();
      do_line_X(); 
      //CheckGyro();
      dung();
      if(Reset4())
      { // neu cong tac 3 4 5 khong gat
      quayphaiTaygap();
      while(digitalRead(right) == 1);
      analogWrite(PWM_6,0);
      gap();
      delay(500);
      ha_ct1();
      delay(500);
      nang_ct1();
      while(digitalRead(sensor) == 1); // nhận vật 3
      delay(1500);
      }
      nang_ct2();
      TCNT5 = 0;
      tien();
      tangtoc();
      do_line_Y_tien_sanxanh(2320);
      dung();
      //TCNT5 = 0;
      // tien();
      // speed(245,241,241,245);
      // while(TCNT5 < 1000);
      // dung();
      TCNT5 = 0;
      //CheckGyro();
      do_line_Y();
      do_line_X();
      //CheckGyro();
      dung();
      if(Reset4())
      { // neu cong tac 3 4 5 khong ga
      delay(300);
      ha_tha_vat();
      delay(500);
      Check_sensor_TZ2();
      delay(1000);
      tha(); // thả cột 3 //////////////////////
      delay(500);
      gap();
      delay(500);
      tha();
      delay(200);
      }
      nang_ct3();
      quaytraiTaygap();
      while(digitalRead(mid) == 1);
      analogWrite(PWM_6,0);
      gap();
      do_line_X();
      do_line_Y();
      delay(500);
      ha_ct1();
      delay(500);
      nang_ct1();
      while(digitalRead(sensor) == 1 ); // nhan vat 4
      digitalWrite(IN_T5,HIGH);
      delay(1500);
      TCNT5 = 0;
      nang_ct2();
      delay(500);
      trai();
      tangtoc();
      delay(1000);
      do_line_X_trai(1850);
      dung();
      dung(); 
      delay(100);
      //CheckGyro();
      do_line_X(); 
      do_line_Y();
      //CheckGyro();
      delay(100);
      TCNT5 = 0;
      trai();
      tangtoc();
      delay(1000);
      do_line_X_trai(1250);
      dung();
      //dung(); 
      //CheckGyro1();
      //do_line_X(); 
      TCNT5 = 0;
      trai();
      speed(245,241,241,245);
      while(TCNT5 <= 600);
      dung();
      do_line_Y();
      //CheckGyro1();
      delay(100);
      //ha_ct2();
      ha_tha_vat();
      delay(500);
      Check_sensor_TZ4();
      delay(1000);
      tha(); // tha vat 4/////////////////////
      delay(500);
      gap();
      delay(500);
      tha();
      delay(100);
      //do_line_X();
      do_line_Y();
      //CheckGyro1();
      nang_ct3();
      TCNT5 = 0;
      phai();
      tangtoc();
      delay(1000);
      do_line_X_phai(1850);
      dung();
      delay(500);
      CheckGyro();
      do_line_X();
      do_line_Y();
      //CheckGyro1();
      ha_ct1();
      delay(500);
      nang_ct1();
      gap();
      while(digitalRead(sensor) == 1 ); ///// nhan vat 5
      delay(1500);
      nang_ct3();
      TCNT5 = 0;
      trai();
      tangtoc();
      delay(1000);
      do_line_X_trai(1350);
      dung();
      TCNT5 = 0;
      trai();
      speed(245,241,241,245);
      while(TCNT5 <= 500);
      //dung(); 
      delay(100);
      CheckGyro();
      //do_line_X(); 
      do_line_Y();
      //CheckGyro1();
      digitalWrite(IN_T5,LOW);
      delay(100);
      TCNT5 = 0;
      lui();
      tangtoc();
      do_line_Y_lui(3720);
      dung();
      dung();
      delay(200);
      CheckGyro();
      do_line_X();
      do_line_Y();
      CheckGyro();
      do_line_Y();
      CheckGyro();
      delay(500);
      quaytraiTaygap();
      while(digitalRead(left) == 1);
      analogWrite(PWM_6,0);
      delay(100);
      ha_tha_vat();
      delay(500);
      Check_sensor_TZ1();
      delay(1000);
      tha(); // thả vật vào cột 5
      delay(500);
      gap();
      delay(500);
      tha();
      delay(500);
      //gap();
      nang_ct3();
      phai();
      speed(247,245,245,247);
      delay(700);
      dung();
      while(1);
    delay(1000);
    }
    if(!Reset4() && Reset5())
    {
      if(turn_fillter == false)
      {
        filter_gyro();
        turn_fillter = true;
      }
      while(digitalRead(set_point) == 1) ha();
      analogWrite(PWM_5,0);
      while(digitalRead(sensor) == 1 ); // nhan vat 4
      digitalWrite(IN_T5,HIGH);
      delay(1500);
      TCNT5 = 0;
      nang_ct2();
      delay(500);
      trai();
      tangtoc();
      delay(1000);
      do_line_X_trai(1850);
      dung();
      dung(); 
      delay(100);
      //CheckGyro();
      do_line_X(); 
      do_line_Y();
      //CheckGyro();
      delay(100);
      TCNT5 = 0;
      trai();
      tangtoc();
      delay(1000);
      do_line_X_trai(1650);
      dung();
      //dung(); 
      //CheckGyro1();
      //do_line_X(); 
      TCNT5 = 0;
      trai();
      speed(245,241,241,245);
      while(TCNT5 <= 200);
      dung();
      do_line_Y();
      //CheckGyro1();
      delay(100);
      //ha_ct2();
      ha_tha_vat();
      delay(500);
      Check_sensor_TZ4();
      delay(1000);
      tha(); // tha vat 4/////////////////////
      delay(500);
      gap();
      delay(500);
      tha();
      delay(100);
      //do_line_X();
      do_line_Y();
      //CheckGyro1();
      nang_ct3();
      TCNT5 = 0;
      phai();
      tangtoc();
      delay(1000);
      do_line_X_phai(1850);
      dung();
      delay(500);
      CheckGyro();
      do_line_X();
      do_line_Y();
      //CheckGyro1();
      ha_ct1();
      delay(500);
      nang_ct1();
      gap();
      while(digitalRead(sensor) == 1 ); ///// nhan vat 5
      delay(1500);
      nang_ct3();
      TCNT5 = 0;
      trai();
      tangtoc();
      delay(1000);
      do_line_X_trai(1350);
      dung();
      TCNT5 = 0;
      trai();
      speed(245,241,241,245);
      while(TCNT5 <= 500);
      dung(); 
      TCNT5 = 0;
      delay(100);
      CheckGyro();
      //do_line_X(); 
      do_line_Y();
      //CheckGyro1();
      digitalWrite(IN_T5,LOW);
      delay(100);
      TCNT5 = 0;
      lui();
      tangtoc();
      do_line_Y_lui(3720);
      dung();
      dung();
      delay(200);
      CheckGyro();
      do_line_X();
      do_line_Y();
      CheckGyro();
      do_line_Y();
      CheckGyro();
      delay(500);
      quaytraiTaygap();
      while(digitalRead(left) == 1);
      analogWrite(PWM_6,0);
      delay(100);
      ha_tha_vat();
      delay(500);
      Check_sensor_TZ1();
      delay(1000);
      tha(); // thả vật vào cột 5
      delay(500);
      gap();
      delay(500);
      tha();
      delay(500);
      //gap();
      nang_ct3();
      phai();
      speed(247,245,245,247);
      delay(700);
      dung();
      while(1);
    delay(1000);

    }
    if( Reset4() && !Reset5() )
    {
      if(turn_fillter == false)
      {
        filter_gyro();
        turn_fillter = true;
      }
      while(digitalRead(set_point) == 1) ha();
      analogWrite(PWM_5,0);
      gap();
      while(digitalRead(sensor) == 1 ); ///// nhan vat 5
      delay(1500);
      digitalWrite(IN_T5,HIGH);
      nang_ct3();
      TCNT5 = 0;
      trai();
      tangtoc();
      delay(1000);
      do_line_X_trai(1350);
      dung();
      TCNT5 = 0;
      trai();
      speed(245,241,241,245);
      while(TCNT5 <= 500);
      //dung(); 
      delay(100);
      CheckGyro();
      //do_line_X(); 
      do_line_Y();
      //CheckGyro1();
      digitalWrite(IN_T5,LOW);
      delay(100);
      TCNT5 = 0;
      lui();
      tangtoc();
      do_line_Y_lui(3720);
      dung();
      dung();
      delay(200);
      CheckGyro();
      do_line_X();
      do_line_Y();
      CheckGyro();
      do_line_Y();
      CheckGyro();
      delay(500);
      quaytraiTaygap();
      while(digitalRead(left) == 1);
      analogWrite(PWM_6,0);
      delay(100);
      ha_tha_vat();
      delay(500);
      Check_sensor_TZ1();
      delay(1000);
      tha(); // thả vật vào cột 5
      delay(500);
      gap();
      delay(500);
      tha();
      delay(500);
      //gap();
      nang_ct3();
      phai();
      speed(247,245,245,247);
      delay(700);
      dung();
      while(1);
    delay(1000);
    }
  }
  
}


void gyro()
{
  previousTime = millis();
  Wire.beginTransmission(MPU);
  Wire.write(GYRO_XH);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU,6,true);
  GyroX = (Wire.read() << 8 | Wire.read()) / 131.0;
  GyroY = (Wire.read() << 8 | Wire.read()) / 131.0;
  GyroZ = (Wire.read() << 8 | Wire.read()) / 131.0;
  roll  += (GyroX - froll)   * dt;
  pitch += (GyroY - fpitch)  * dt;
  yaw   += (GyroZ - fyaw)    * dt;
    // Serial.print("X : "); Serial.print((int)roll);
    // Serial.print(", Y : "); Serial.print((int)pitch);
    // Serial.print(", Z : "); Serial.println((int)yaw);
  currentTime = millis();
  elapsedTime = currentTime - previousTime;
  dt == elapsedTime / 1000;
}


void filter_gyro()
{
  Serial.println("start probe filter");
  delay(500);
  for(int x = 0; x < 10; x++)
  {
    Serial.print("*");
  }
  Serial.println("*");
  for(int x = 0; x <1000;x++)
  {
    Wire.beginTransmission(MPU);
    Wire.write(0x43);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU,6,true);
    GyroX = (Wire.read() << 8 | Wire.read()) / 131.0;
    GyroY = (Wire.read() << 8 | Wire.read()) / 131.0;
    GyroZ = (Wire.read() << 8 | Wire.read()) / 131.0;
    froll   += GyroX;
    fpitch  += GyroY;
    fyaw    += GyroZ;
  }
  froll = froll/1000;
  fpitch = fpitch/1000;
  fyaw = fyaw/1000;
  Serial.println("Probe filter done!");
  Serial.print("X filter: "); Serial.print(froll);
  Serial.print(", Y filter: "); Serial.print(fpitch);
  Serial.print(", Z filter: "); Serial.println(fyaw);
  
}

void nang_ct1()
{
  while(digitalRead(ct_cot1) == 1)
    {
      nang();
    }
  analogWrite(PWM_5,0);
}

void nang_ct2()
{
  while(digitalRead(ct_cot2) == 1)
    {
      nang();
    }
    analogWrite(PWM_5,0);
}

void nang_ct3()
{
  while(digitalRead(ct_cot3) == 1)
    {
      nang();
    }
    analogWrite(PWM_5,0);
}

void ha_ct1()
{
  while(digitalRead(ct_cot1) == 0)
  {
    ha();
  }
  analogWrite(PWM_5,0);
}

void ha_ct2()
{
  while(digitalRead(ct_cot2) == 0)
  {
    ha();
  }
  analogWrite(PWM_5,0);
}

void ha_tha_vat()
{
  ha();
  delay(500);
  analogWrite(PWM_5,0);
}

void Check_sensor_Y1()
{
  delay(100);
  while(1)
  {
  if(digitalRead(sensor_Y1) == 1)
  { // lech phia truoc
  phai();
  speed(247,245,245,247);
  while(digitalRead(sensor_Y1) == 1);
  dung();
  return 0;
  }
  while(digitalRead(sensor_Y0) == 1)
  {
  trai();
  speed(247,245,245,247);
  while(digitalRead(sensor_Y0) == 1);
  dung();
  return 0;
  //return 0;
  }
  dung();
  break;
  }
  
}


void Check_sensor_Y2()
{
  delay(100);
  while(1)
  {
  if(digitalRead(sensor_Y1) == 1)
  { // lech phia truoc
  trai();
  speed(247,245,245,247);
  while(digitalRead(sensor_Y1) == 1);
  dung();
  return 0;
  }
  if(digitalRead(sensor_Y0) == 1)
  {
  phai();
  speed(247,245,245,247);
  while(digitalRead(sensor_Y0) == 1);
  dung();
  return 0;
  }
  dung();
  break;
  }
  
}


void Check_sensor_Y4()
{
  delay(100);
  while(1)
  {
  if(digitalRead(sensor_Y1) == 1)
  { // lech phia truoc
  lui();
  speed(247,245,245,247);
  while(digitalRead(sensor_Y1) == 1);
  dung();
  return 0;
  }
  while(digitalRead(sensor_Y0) == 1)
  {
  tien();
  speed(247,245,245,247);
  while(digitalRead(sensor_Y0) == 1);
  dung();
  return 0;
  }
  dung();
  break;
  }
  
}

void Check_sensor_TZ1()
{
  Check_sensor_Y1();
  while(1){
    if(digitalRead(sensor_X0) == 0 && digitalRead(sensor_X1) == 0)
    {
      return 0;
    }
    if(digitalRead(sensor_X0) == 1)
    {
      tien();
      speed(247,245,245,247);
      while(digitalRead(sensor_X0));  
      dung();
      return 0;
    }
    if(digitalRead(sensor_X1) == 1)
    {
      lui();
      speed(247,245,245,247);
      while(digitalRead(sensor_X1));
      dung();
      return 0;
    }
    dung();
    break;
  }
}
  //while(1);



void Check_sensor_TZ2()
{
  Check_sensor_Y2();
  while(1){
    if(digitalRead(sensor_X0) == 0 && digitalRead(sensor_X1) == 0)
    {
      return 0;
    }
    if(digitalRead(sensor_X0) == 1)
    {
      lui();
      speed(247,245,245,247);
      while(digitalRead(sensor_X0) == 1);
      dung();
      return 0;
    }
    if(digitalRead(sensor_X1) == 1)
    {
            
      tien();
      speed(247,245,245,247);
      while(digitalRead(sensor_X1) == 1);
      dung();
      return 0;
    }
    dung();
    break;
  }
}


void Check_sensor_TZ4()
{
  Check_sensor_Y4();
  while(1){
    if(digitalRead(sensor_X0) == 0 && digitalRead(sensor_X1) == 0)
    {
      return 0;
    }
    if(digitalRead(sensor_X0) == 1 && digitalRead(sensor_X1) == 1)
    {
      return 0;
    }
    if(digitalRead(sensor_X0) == 1)
    {
    phai();
    speed(247,245,245,247);
    while(digitalRead(sensor_X0) == 1);
    dung();
    return 0;
    }
    if(digitalRead(sensor_X1) == 1)
    {
      trai();
      speed(247,245,245,247);
      while(digitalRead(sensor_X1) == 1);
      dung();
      return 0;
    }
    dung();
    break;
  }
}


void init_timer5()
{
  pinMode(EXTERNAL_CLOCK_PIN, INPUT);

  // Tắt ngắt toàn cục
  cli();

  // Reset cài đặt của Timer 5
  TCCR5A = 0;
  TCCR5B = 0;

  // Cài đặt chế độ Normal cho Timer 5
  TCCR5A |= (0 << WGM50) | (0 << WGM51);
  TCCR5B |= (0 << WGM52) | (0 << WGM53);

  // Cài đặt nguồn xung ngoài cho Timer 5 (chân PL2, chân số 47)
  // CS52:0 = 111 cho phép nguồn xung ngoài trên cạnh lên
  TCCR5B |= (1 << CS52) | (1 << CS51) | (1 << CS50);

  TCNT5 = 0;

}

ISR(TIMER5_CAPT_vect) {
  // Đọc giá trị hiện tại của kênh A và B từ ICR5
  uint8_t channel_a = PINE & (1 << PINE3);
  uint8_t channel_b = PINB & (1 << CHANNEL_B_PIN_X);

  if (channel_a != channel_b) { // Nếu kênh A và B khác nhau
    encoderX--; // lùi
  } else {
    encoderX++; //tiến
  }
}


void scancolor()
{
  countButton();
    if(countY == 1)
    { 
      digitalWrite(sttY,LOW);
      Serial.println("bat dau do line Y");
      Serial.println("dat vao mau xanh");
      Read_LEDY(ledYGR);
      delay(500);
      saveSensor(ledYGR, ADDRESS_LEDYGR);
      Serial.println("da luu ledY green vao eeprom");
      digitalWrite(sttY,HIGH);
      while(countY == 1) countButton();
    }
    if(countY == 2)
    {
      digitalWrite(sttY,LOW);
      Serial.println("dat vao mau trang");
      Read_LEDY(ledYWT);
      Serial.println("bat dau tinh toan");
      for(int i = 0; i < 8;i++)
      {
        numY[i] = ledYGR[i] - ledYWT[i] - 2;
      }
      delay(500);
      saveSensor(numY, ADDRESS_NUMY);
      Serial.println("da luu sai so vao eeprom");
      digitalWrite(sttY,HIGH);
      while(countY == 2) countButton();
      digitalWrite(sttY,LOW);
    }
}

void countButton()
{
  if(digitalRead(buttonY) == 0)
  {
    delay(50);
    if(digitalRead(buttonY) == 0)
    {
    countY++;
    if(countY == 10) countY = 0;
    delay(500);
    }
  }
}

void saveSensor( int SensorNumber[],int address ) { 
  // Xóa khu vực lưu trữ cảm biến trước khi ghi dữ liệu mới vào
  for (int i = 0; i < 9; ++i) {
    EEPROM.write(address + i, 0); // Ghi giá trị 0 vào từng byte trong từng byte lưu cảm biến
    delay(5);
  }

  // Ghi cảm biến vào EEPROM
  for (int i = 0; i < 9; ++i) {
    EEPROM.write(address + i, SensorNumber[i]);
    delay(5);
  }

  // Lưu thay đổi vào EEPROM (không bỏ qua bước này)
  //EEPROM.commit();
}

void readSensorNumber(int SensorNumber[], int address) {
  for (int i = 0; i < 9; ++i) {
    SensorNumber[i] = EEPROM.read(address+i);
    Serial.print(SensorNumber[i]);
  }
  SensorNumber[9 - 1] = '\0'; // Kết thúc chuỗi bằng ký tự null (NULL-terminated string)
}


bool Area()
{
  return digitalRead(area);
}

bool Reset2()
{
  return digitalRead(reset2);
}

bool Reset3()
{
  return digitalRead(reset3);
}

bool Reset4()
{
  return digitalRead(reset4);
}

bool Reset5()
{
  return digitalRead(reset5);
}

void gap()
{
  digitalWrite(KN1,LOW); // gap 
  digitalWrite(KN2,HIGH);
}

void tha()
{
  digitalWrite(KN1,HIGH); //tha
  digitalWrite(KN2,LOW);
}

void nang()
{
  analogWrite(PWM_5,160);
  digitalWrite(DIR_5,LOW);
}

void quayphaiTaygap()
{
  digitalWrite(DIR_6,HIGH);
  analogWrite(PWM_6,60);
}

void quaytraiTaygap()
{
  digitalWrite(DIR_6,LOW);
  analogWrite(PWM_6,60);
}

void ha()
{
  analogWrite(PWM_5,150);
  digitalWrite(DIR_5,HIGH);
}

void dung()
{
  analogWrite(PWM_1,255);
  analogWrite(PWM_2,255);
  analogWrite(PWM_3,255);
  analogWrite(PWM_4,255);
}


void tien()
{
  digitalWrite(DIR_1,HIGH);
  digitalWrite(DIR_2,LOW);
  digitalWrite(DIR_3,HIGH);
  digitalWrite(DIR_4,LOW);
}
void lui()
{
  digitalWrite(DIR_1,LOW);
  digitalWrite(DIR_2,HIGH);
  digitalWrite(DIR_3,LOW);
  digitalWrite(DIR_4,HIGH);
}
void trai()
{
  digitalWrite(DIR_1,LOW);
  digitalWrite(DIR_2,LOW);
  digitalWrite(DIR_3,HIGH);
  digitalWrite(DIR_4,HIGH);
}
void phai()
{
  digitalWrite(DIR_1,HIGH);
  digitalWrite(DIR_2,HIGH);
  digitalWrite(DIR_3,LOW);
  digitalWrite(DIR_4,LOW);
}
void quayphai()
{
  digitalWrite(DIR_1,HIGH);
  digitalWrite(DIR_2,HIGH);
  digitalWrite(DIR_3,HIGH);
  digitalWrite(DIR_4,HIGH);
}
void quaytrai()
{
  digitalWrite(DIR_1,LOW);
  digitalWrite(DIR_2,LOW);
  digitalWrite(DIR_3,LOW);
  digitalWrite(DIR_4,LOW);
}

void speed(int pwm1, int pwm2, int pwm3, int pwm4) // khong truyen i < 100 
{
  analogWrite(PWM_1,pwm1);
  analogWrite(PWM_2,pwm2);
  analogWrite(PWM_3,pwm3);
  analogWrite(PWM_4,pwm4);
}
/*
void tangtoc(int sp)
{
  i = 255;
  while(i>=sp)
  { 
    analogWrite(PWM_1,i);
    analogWrite(PWM_2,i);
    analogWrite(PWM_3,i);
    analogWrite(PWM_4,i);
    delay(20);
    i--;
  }
}
*/

void tangtoc()
{
    speed(245,241,241,245);
    delay(200);
    speed(240,235,235,240);
}




void do_line_X_phai(int encoder)
{
  //int i = 0;
  while(TCNT5 <= encoder)
  {
  Read_LEDX(nledX);
  gyro();
  if( !(nledX[3] == 1) || !(nledX[4] == 1) || yaw > 5 || yaw <-5 )
  {
    while( (nledX[5] == 1) || (nledX[6] == 1) || (nledX[7] == 1) ) // lech truoc
    {
      Read_LEDX(nledX);
      gyro();
      //phai();
      analogWrite(PWM_1,240);
      analogWrite(PWM_2,230);
      analogWrite(PWM_3,230);
      analogWrite(PWM_4,240);
    }
    while((nledX[0] == 1) || (nledX[1] == 1) || (nledX[2] == 1) ) // lech sau
    {
      Read_LEDX(nledX);
      gyro();
      //phai();
      analogWrite(PWM_1,235);
      analogWrite(PWM_2,240);
      analogWrite(PWM_3,240);
      analogWrite(PWM_4,235);
    }
    
    while(yaw > 5 || yaw <-5)  // lech
    {
      gyro();
      if(yaw > 15) yaw = 15;
      if(yaw <-15) yaw = -15;
      //analogWrite(PWM_1,248);
      //analogWrite(PWM_2,i-(yaw));
      analogWrite(PWM_2,235-(yaw/2));
      analogWrite(PWM_4,235+(yaw/2));
      if(TCNT5 > encoder) break;
    }
    
    
  }else
  {
    gyro();
    phai();
    analogWrite(PWM_1,240);
    analogWrite(PWM_2,235);
    analogWrite(PWM_3,235);
    analogWrite(PWM_4,240);
  }
  }
  
}


void do_line_X_trai(int encoder)
{
  //int i = 0;
  while(TCNT5 <= encoder)
  {
  Read_LEDX(nledX);
  gyro();
  if( !(nledX[3] == 1) || !(nledX[4] == 1) || yaw > 5 || yaw <-5)
  {
    while( (nledX[5] == 1) || (nledX[6] == 1) || (nledX[7] == 1) ) // lech truoc
    {
      Read_LEDX(nledX);
      gyro();
      trai();
      analogWrite(PWM_1,235);
      analogWrite(PWM_2,240);
      analogWrite(PWM_3,240);
      analogWrite(PWM_4,235);
    }
    while((nledX[0] == 1) || (nledX[1] == 1)|| (nledX[2] == 1) ) // lech sau
    {
      Read_LEDX(nledX);
      gyro();
      trai();
      analogWrite(PWM_1,240);
      analogWrite(PWM_2,230);
      analogWrite(PWM_3,230);
      analogWrite(PWM_4,240);
    }
    while(yaw > 5 || yaw <-5)  // lech
    {
      trai();
      gyro();
       if(yaw > 15) yaw = 15;
       if(yaw <-15) yaw = -15;
      //analogWrite(PWM_1,248);
      //analogWrite(PWM_2,i-(yaw));
      analogWrite(PWM_2,235+(yaw));
      analogWrite(PWM_4,235-(yaw));
      if(TCNT5 > encoder) break;
    }
    
  }else
  {
    gyro();
    trai();
    analogWrite(PWM_1,240);
    analogWrite(PWM_2,235);
    analogWrite(PWM_3,235);
    analogWrite(PWM_4,240);
  }
  }
  
}



void do_line_X()
{
  int i = 0;
  while(1)
  {
  gyro();
  Read_LEDX(nledX);
  if( !(nledX[3] == 1) && !(nledX[4] == 1) )
  {
    while( (nledX[5] == 1) || (nledX[6] == 1) || (nledX[7] == 1) )
    {
      gyro();
      Read_LEDY(nledX);
     // Serial.println("lech truoc");
      lui();
      speed(245,241,241,245);
    }
    while((nledX[0] == 1) || (nledX[1] == 1) || (nledX[2] == 1) )
    {
      gyro();
      Read_LEDX(nledX);
     // Serial.println("lech sau");
      tien();
      speed(245,241,241,245);
    }
  }else
  {
    dung();
    break;
  }
  }
  
}

void do_line_Y()
{
  //int i = 0;
  while(1)
  {
  gyro();
  Read_LEDY(nledY);
  if( !(ledYGR[3] - nledY[3] >= numY[3]) && !(ledYGR[4] - nledY[4] >= numY[4]) )
  {
    while( (ledYGR[5] - nledY[5] >= numY[5]) || (ledYGR[6] - nledY[6] >= numY[6]) || (ledYGR[7] - nledY[7] >= numY[7]) )
    {
      
      Read_LEDY(nledY);
      gyro();
      trai();
      speed(245,241,241,245);
    }
    while((ledYGR[0] - nledY[0] >= numY[0]) || (ledYGR[1] - nledY[1] >= numY[1]) || (ledYGR[2] - nledY[2] >= numY[2]) )
    {
      
      Read_LEDY(nledY);
      gyro();
      phai();
      speed(245,241,241,245);
    }
    //dung();
  }else
  {
    dung();
    break;
  }
  }
  
}


void do_line_Y_tien(int encoder)
{
  //int i = 0;
  while(TCNT5 <= encoder)
  {
  gyro();
  Read_LEDY(nledY);
  if( !(ledYGR[3] - nledY[3] >= numY[3]) || !(ledYGR[4] - nledY[4] >= numY[4]) || yaw > 5 || yaw <-5 )
  {
    while( (ledYGR[5] - nledY[5] >= numY[5]) || (ledYGR[6] - nledY[6] >= numY[6]) || (ledYGR[7] - nledY[7] >= numY[7]) )
    {
      gyro();
      Read_LEDY(nledY);
      //Serial.println("lech phai");
      tien();
      analogWrite(PWM_1,240);
      analogWrite(PWM_2,230);
      analogWrite(PWM_3,230);
      analogWrite(PWM_4,240);
    }
    while((ledYGR[0] - nledY[0] >= numY[0]) || (ledYGR[1] - nledY[1] >= numY[1]) || (ledYGR[2] - nledY[2] >= numY[2]) )
    {
      gyro();
      Read_LEDY(nledY);
      //Serial.println("lech trai");
      tien();
      analogWrite(PWM_1,235);
      analogWrite(PWM_2,235);
      analogWrite(PWM_3,235);
      analogWrite(PWM_4,235);
    }
    while(yaw > 5 || yaw <-5)  // lech
    {
      gyro();
       if(yaw > 15) yaw = 15;
       if(yaw <-15) yaw = -15;
      //analogWrite(PWM_1,i+(yaw));
      //analogWrite(PWM_2,i-(yaw));
      analogWrite(PWM_3,235-(yaw));
      analogWrite(PWM_4,235+(yaw));
      if(TCNT5 > encoder) break;
    }
    
    //dung();
    //dung();
  }else
  {
    gyro();
    tien();
    analogWrite(PWM_1,240);
    analogWrite(PWM_2,235);
    analogWrite(PWM_3,235);
    analogWrite(PWM_4,240);
  }
  }
  
}


void do_line_Y_tien_sanxanh(int encoder)
{
  //int i = 0;
  while(TCNT5 <= encoder)
  {
  gyro();
  Read_LEDY(nledY);
  if( !(ledYGR[2] - nledY[2] >= numY[2]) || !(ledYGR[3] - nledY[3] >= numY[3]) || yaw > 5 || yaw <-5 )
  {
    while( (ledYGR[4] - nledY[4] >= numY[4]) || (ledYGR[5] - nledY[5] >= numY[5]) || (ledYGR[6] - nledY[6] >= numY[6]) || (ledYGR[7] - nledY[7] >= numY[7]) )
    {
      gyro();
      Read_LEDY(nledY);
      //Serial.println("lech phai");
      tien();
      analogWrite(PWM_1,240);
      analogWrite(PWM_2,230);
      analogWrite(PWM_3,230);
      analogWrite(PWM_4,240);
    }
    while((ledYGR[0] - nledY[0] >= numY[0]) || (ledYGR[1] - nledY[1] >= numY[1])  )
    {
      gyro();
      Read_LEDY(nledY);
      //Serial.println("lech trai");
      tien();
      analogWrite(PWM_1,235);
      analogWrite(PWM_2,235);
      analogWrite(PWM_3,235);
      analogWrite(PWM_4,235);
    }
    while(yaw > 5 || yaw <-5)  // lech
    {
      gyro();
       if(yaw > 15) yaw = 15;
       if(yaw <-15) yaw = -15;
      //analogWrite(PWM_1,i+(yaw));
      //analogWrite(PWM_2,i-(yaw));
      analogWrite(PWM_3,235-(yaw));
      analogWrite(PWM_4,235+(yaw));
      if(TCNT5 > encoder) break;
    }
    
    //dung();
    //dung();
  }else
  {
    gyro();
    tien();
    analogWrite(PWM_1,240);
    analogWrite(PWM_2,235);
    analogWrite(PWM_3,235);
    analogWrite(PWM_4,240);
  }
  }
  
}

void do_line_Y_lui(int encoder)
{
  //int i = 0;
  while(TCNT5 <= encoder)
  {
  gyro();
  Read_LEDY(nledY);
  if( !(ledYGR[3] - nledY[3] >= numY[3]) || !(ledYGR[4] - nledY[4] >= numY[4]) || yaw > 5 || yaw <-5 )
  {
    while( (ledYGR[5] - nledY[5] >= numY[5]) || (ledYGR[6] - nledY[6] >= numY[6]) || (ledYGR[7] - nledY[7] >= numY[7]) )
    {
      gyro();
      Read_LEDY(nledY);
      //Serial.println("lech phai");
      //lui();
      analogWrite(PWM_1,235);
      analogWrite(PWM_2,240);
      analogWrite(PWM_3,240);
      analogWrite(PWM_4,235);
      if(TCNT5 > encoder) break;
    }
    while((ledYGR[0] - nledY[0] >= numY[0]) || (ledYGR[1] - nledY[1] >= numY[1]) || (ledYGR[2] - nledY[2] >= numY[2]) )
    {
      gyro();
      Read_LEDY(nledY);
      //Serial.println("lech trai");
      //lui();
      analogWrite(PWM_1,240);
      analogWrite(PWM_2,230);
      analogWrite(PWM_3,230);
      analogWrite(PWM_4,240);
      if(TCNT5 > encoder) break;
    }
    while(yaw > 5 || yaw <-5)  // lech
    {
      gyro();
       if(yaw > 15) yaw = 15;
       if(yaw <-15) yaw = -15;
      //analogWrite(PWM_1,i+(yaw));
      //analogWrite(PWM_2,i-(yaw));
      analogWrite(PWM_3,235+(yaw));
      analogWrite(PWM_4,235-(yaw));
      if(TCNT5 > encoder) break;
    }
    
    //dung();
    //giamtoc();
  }else
  {
    gyro();
    lui();
    analogWrite(PWM_1,240);
    analogWrite(PWM_2,235);
    analogWrite(PWM_3,235);
    analogWrite(PWM_4,240);
  }
  }
}


void do_line_Y_lui_sanxanh(int encoder)
{
  //int i = 0;
  while(TCNT5 <= encoder)
  {
  gyro();
  Read_LEDY(nledY);
  if( !(ledYGR[2] - nledY[2] >= numY[2]) || !(ledYGR[3] - nledY[3] >= numY[3]) || yaw > 5 || yaw <-5 )
  {
    while( (ledYGR[4] - nledY[4] >= numY[4]) || (ledYGR[5] - nledY[5] >= numY[5]) || (ledYGR[6] - nledY[6] >= numY[6]) || (ledYGR[7] - nledY[7] >= numY[7]) )
    {
      gyro();
      Read_LEDY(nledY);
      //Serial.println("lech phai");
      //lui();
      analogWrite(PWM_1,235);
      analogWrite(PWM_2,235);
      analogWrite(PWM_3,235);
      analogWrite(PWM_4,235);
      if(TCNT5 > encoder) break;
    }
    while((ledYGR[0] - nledY[0] >= numY[0]) || (ledYGR[1] - nledY[1] >= numY[1]) )
    {
      gyro();
      Read_LEDY(nledY);
      //Serial.println("lech trai");
      //lui();
      analogWrite(PWM_1,240);
      analogWrite(PWM_2,230);
      analogWrite(PWM_3,230);
      analogWrite(PWM_4,240);
      if(TCNT5 > encoder) break;
    }
    while(yaw > 5 || yaw <-5)  // lech
    {
      gyro();
       if(yaw > 15) yaw = 15;
       if(yaw <-15) yaw = -15;
      //analogWrite(PWM_1,i+(yaw));
      //analogWrite(PWM_2,i-(yaw));
      analogWrite(PWM_3,235+(yaw));
      analogWrite(PWM_4,235-(yaw));
      if(TCNT5 > encoder) break;
    }
    
    //dung();
    //giamtoc();
  }else
  {
    gyro();
    lui();
    analogWrite(PWM_1,240);
    analogWrite(PWM_2,235);
    analogWrite(PWM_3,235);
    analogWrite(PWM_4,240);
  }
  }
}


void CheckGyro1()
{
  while(1)
  {
  
  if(yaw > -1|| yaw < -3 )
  {
    while(yaw<-3) // lech phai
    {
      gyro();
      quaytrai();
      speed(245,245,245,245);
    }
    while(yaw>-1) // lech trai
    {
      gyro();
      quayphai();
      speed(245,245,245,245);
    }
  }else {
    dung();
    break;
  }
  }
  
}


void CheckGyro()
{
  while(1)
  {
  
  if(yaw > 1|| yaw < 0 )
  {
    while(yaw<0) // lech phai
    {
      gyro();
      quaytrai();
      speed(245,245,245,245);
    }
    while(yaw>1) // lech trai
    {
      gyro();
      quayphai();
      speed(245,245,245,245);
    }
  }else {
    dung();
    break;
  }
  }
  
}


void Read_LEDY(int arrayLed[])
{
  arrayLed[0] = analogRead(LEDY0)/4;
  arrayLed[1] = analogRead(LEDY1)/4;
  arrayLed[2] = analogRead(LEDY2)/4;
  arrayLed[3] = analogRead(LEDY3)/4;
  arrayLed[4] = analogRead(LEDY4)/4;
  arrayLed[5] = analogRead(LEDY5)/4;
  arrayLed[6] = analogRead(LEDY6)/4;
  arrayLed[7] = analogRead(LEDY7)/4;
}

void Read_LEDX(int arrayLed[])
{
  arrayLed[0] = digitalRead(LEDX0);
  arrayLed[1] = digitalRead(LEDX1);
  arrayLed[2] = digitalRead(LEDX2);
  arrayLed[3] = digitalRead(LEDX3);
  arrayLed[4] = digitalRead(LEDX4);
  arrayLed[5] = digitalRead(LEDX5);
  arrayLed[6] = digitalRead(LEDX6);
  arrayLed[7] = digitalRead(LEDX7);
}
