#define PWM_1 2
#define PWM_2 3
#define PWM_3 4
#define PWM_4 5

#define EN11 29
#define EN12 31
#define EN13 33
#define EN14 35

#define EN21 32
#define EN22 34
#define EN23 28
#define EN24 30

#define DIR1 37
#define PWM_5 6


#define KN8 51 
#define KN7 50 
#define KN6 53 


#include <PS2X_lib.h>

// Khai báo các chân kết nối tay game PS2 với Arduino Mega
#define PS2_DAT        22
#define PS2_CMD        23
#define PS2_SEL        24
#define PS2_CLK        25

// Khởi tạo đối tượng PS2X
PS2X ps2x;


int i = 0; // độ rộng băm xung

bool att_KN6 = 1; 
bool tay_gap = 1;

bool speed = 1;

void setup() {
  pinMode(PWM_1,OUTPUT); // chân băm xung điều khiển tốc độ động cơ
  pinMode(PWM_2,OUTPUT);
  pinMode(PWM_3,OUTPUT);
  pinMode(PWM_4,OUTPUT);
  pinMode(PWM_5,OUTPUT);


  pinMode(EN11,OUTPUT);// chân điều khiển chiều quay động cơ
  pinMode(EN12,OUTPUT);
  pinMode(EN13,OUTPUT);
  pinMode(EN14,OUTPUT);

  pinMode(EN21,OUTPUT);// chân điều khiển chiều quay động cơ
  pinMode(EN22,OUTPUT);
  pinMode(EN23,OUTPUT);
  pinMode(EN24,OUTPUT);

  pinMode(DIR1,OUTPUT); // chân điều khiển chiều quay động cơ nâng hạ

  pinMode(KN6,OUTPUT); // khí nén nâng hạ
  pinMode(KN7,OUTPUT); // khí nén gắp thả
  pinMode(KN8,OUTPUT); // 

  digitalWrite(KN6,att_KN6);
  digitalWrite(KN7,tay_gap);
  digitalWrite(KN7,!tay_gap);

  Serial.begin(9600);

  // Khởi tạo kết nối với tay game PS2
  ps2x.config_gamepad(PS2_CLK, PS2_CMD, PS2_SEL, PS2_DAT, true, true);

  // Kiểm tra xem tay game PS2 có kết nối thành công không
  if (ps2x.config_gamepad(PS2_CLK, PS2_CMD, PS2_SEL, PS2_DAT)) {
    Serial.println("PS2 Controller is connected!");
  } else {
    Serial.println("No PS2 Controller found!");
    while (true); // Dừng chương trình nếu không tìm thấy tay game PS2
  }
}

void loop() {
  // Đọc dữ liệu từ tay game PS2
  ps2x.read_gamepad();
  if (ps2x.Button(PSB_SELECT)) {
      while (ps2x.Button(PSB_SELECT)) ps2x.read_gamepad();
      speed = !speed;
      Serial.println("Select ");
  }
  // Kiểm tra các nút trên tay game PS2
  while(speed == 1)
  {
  //Serial.println("Mode nhanh");
  // ps2x.read_gamepad();
  ps2x.read_gamepad();
  if (ps2x.Button(PSB_PAD_UP) && ps2x.Button(PSB_PAD_RIGHT)==0 && ps2x.Button(PSB_PAD_DOWN)==0 && ps2x.Button(PSB_PAD_LEFT)==0){
      
      tien();
      tangtoc();
      Serial.println("UP ");
      while(ps2x.Button(PSB_PAD_UP))
      {
        ps2x.read_gamepad();
      }
      giamtoc();
      Serial.println("GIAM TOC ");
    }
  if(ps2x.Button(PSB_PAD_UP)==0 && ps2x.Button(PSB_PAD_RIGHT) && ps2x.Button(PSB_PAD_DOWN)==0 && ps2x.Button(PSB_PAD_LEFT)==0) 
    {
      
      phai();
      tangtoc();
      Serial.println("RIGHT ");
      while(ps2x.Button(PSB_PAD_RIGHT))
      {
      ps2x.read_gamepad();
      }
      giamtoc();
      Serial.println("GIAM TOC ");
      
    }

  if (ps2x.Button(PSB_PAD_UP)==0 && ps2x.Button(PSB_PAD_RIGHT) ==0 && ps2x.Button(PSB_PAD_DOWN) && ps2x.Button(PSB_PAD_LEFT)==0)
    {
      lui();
      tangtoc();
      Serial.println("DOWN ");
      while(ps2x.Button(PSB_PAD_DOWN) )
      {
      ps2x.read_gamepad();
      }
      giamtoc();
      Serial.println("GIAM TOC ");
    }
    
    if (ps2x.Button(PSB_PAD_UP)==0 && ps2x.Button(PSB_PAD_RIGHT) ==0 && ps2x.Button(PSB_PAD_DOWN) == 0 && ps2x.Button(PSB_PAD_LEFT) == 1 )
    {
      trai();
      tangtoc();
      Serial.println("LEFT ");
      while(ps2x.Button(PSB_PAD_LEFT))
      {
      ps2x.read_gamepad();
      }
      giamtoc();
      Serial.println("GIAM TOC ");
    }
    if (ps2x.Button(PSB_R1))
    {
      delay(500);
      if(ps2x.Button(PSB_R1))
      {
      Serial.println("R1 ");
      quayphai();
      tangtocquay();
      while(ps2x.Button(PSB_R1))
      {
      ps2x.read_gamepad();
      }
      giamtoc();
      Serial.println("GIAM TOC");
      }
      
    }
    if (ps2x.Button(PSB_L1))
    {
      delay(500);
      if(ps2x.Button(PSB_L1))
      {
      Serial.println("L1");
      quaytrai();
      tangtocquay();
      while(ps2x.Button(PSB_L1))
      {
      ps2x.read_gamepad();
      }
      giamtoc();
      Serial.println("GIAM TOC");
      }
    }

    if (ps2x.Button(PSB_R2))
    {
      delay(300);
      if(ps2x.Button(PSB_R2))
      {
      Serial.println("R2");
      while( ps2x.Button(PSB_R2))
      {
      ps2x.read_gamepad();
      }
      Serial.println("GIAM TOC");
      }
    }

    if (ps2x.Button(PSB_L2))
    {
      delay(200);
      if(ps2x.Button(PSB_L2))
      {
      Serial.println("L2");
      
      while(ps2x.Button(PSB_L2))
      {
      ps2x.read_gamepad();
      }
      Serial.println("GIAM TOC");
      }
    }

    
  if(ps2x.Button(PSB_PAD_UP) == 0 && ps2x.Button(PSB_PAD_DOWN) == 0 && ps2x.Button(PSB_PAD_LEFT) == 0 && ps2x.Button(PSB_PAD_RIGHT) ==0 && ps2x.Button(PSB_R1) == 0 && ps2x.Button(PSB_R2) == 0 && ps2x.Button(PSB_L1) == 0 && ps2x.Button(PSB_L2) == 0){dung();}
  
  }
  if(speed == 0) // mode cham
  { 
  //Serial.println("Mode cham");
  ps2x.read_gamepad();
  if (ps2x.Button(PSB_PAD_UP) && ps2x.Button(PSB_PAD_RIGHT)==0 && ps2x.Button(PSB_PAD_DOWN)==0 && ps2x.Button(PSB_PAD_LEFT)==0){
      
      tien();
      mode_cham();
      Serial.println("UP ");
      while(ps2x.Button(PSB_PAD_UP))
      {
        ps2x.read_gamepad();
      }
      
      Serial.println("GIAM TOC ");
    }
  if(ps2x.Button(PSB_PAD_UP)==0 && ps2x.Button(PSB_PAD_RIGHT) && ps2x.Button(PSB_PAD_DOWN)==0 && ps2x.Button(PSB_PAD_LEFT)==0) 
    {
      
      phai();
      mode_cham();
      Serial.println("RIGHT ");
      while(ps2x.Button(PSB_PAD_RIGHT))
      {
      ps2x.read_gamepad();
      }
      
      Serial.println("GIAM TOC ");
      
    }

  if (ps2x.Button(PSB_PAD_UP)==0 && ps2x.Button(PSB_PAD_RIGHT) ==0 && ps2x.Button(PSB_PAD_DOWN) && ps2x.Button(PSB_PAD_LEFT)==0)
    {
      lui();
      mode_cham();
      Serial.println("DOWN ");
      while(ps2x.Button(PSB_PAD_DOWN) )
      {
      ps2x.read_gamepad();
      }
      
      Serial.println("GIAM TOC ");
    }
    
    if (ps2x.Button(PSB_PAD_UP)==0 && ps2x.Button(PSB_PAD_RIGHT) ==0 && ps2x.Button(PSB_PAD_DOWN) == 0 && ps2x.Button(PSB_PAD_LEFT) == 1 )
    {
      trai();
      mode_cham();
      Serial.println("LEFT ");
      while(ps2x.Button(PSB_PAD_LEFT))
      {
      ps2x.read_gamepad();
      }
      giamtoc();
      Serial.println("GIAM TOC ");
    }
    if (ps2x.Button(PSB_R1))
    {
      delay(500);
      if(ps2x.Button(PSB_R1))
      {
      Serial.println("R1 ");
      quayphai();
      mode_cham_quay();
      while(ps2x.Button(PSB_R1))
      {
      ps2x.read_gamepad();
      }
      giamtoc();
      Serial.println("GIAM TOC");
      }
      
    }
    if (ps2x.Button(PSB_L1))
    {
      delay(500);
      if(ps2x.Button(PSB_L1))
      {
      Serial.println("L1");
      quaytrai();
      mode_cham_quay();
      while(ps2x.Button(PSB_L1))
      {
      ps2x.read_gamepad();
      }
      
      Serial.println("GIAM TOC");
      }
    }

    if (ps2x.Button(PSB_R2))
    {
      delay(300);
      if(ps2x.Button(PSB_R2))
      {
      Serial.println("R2");
      while( ps2x.Button(PSB_R2))
      {
      ps2x.read_gamepad();
      }
      Serial.println("GIAM TOC");
      }
    }

    if (ps2x.Button(PSB_L2))
    {
      delay(200);
      if(ps2x.Button(PSB_L2))
      {
      Serial.println("L2");
      
      while(ps2x.Button(PSB_L2))
      {
      ps2x.read_gamepad();
      }
      Serial.println("GIAM TOC");
      }
    }

    
    if(ps2x.Button(PSB_PAD_UP) == 0 && ps2x.Button(PSB_PAD_DOWN) == 0 && ps2x.Button(PSB_PAD_LEFT) == 0 && ps2x.Button(PSB_PAD_RIGHT) ==0 && ps2x.Button(PSB_R1) == 0 && ps2x.Button(PSB_R2) == 0 && ps2x.Button(PSB_L1) == 0 && ps2x.Button(PSB_L2) == 0){dung();}
    

  }

  
  
  if (ps2x.NewButtonState()) {
    if (ps2x.Button(PSB_TRIANGLE)) 
    {
      nang();
      while (ps2x.Button(PSB_TRIANGLE)) 
      {
        ps2x.read_gamepad();
      }
    analogWrite(PWM_5,255);
    Serial.println("Tam Giac");
    }
  }

  // Tròn
  if (ps2x.ButtonPressed(PSB_CIRCLE)) {
    tay_gap = !tay_gap;
    while (ps2x.Button(PSB_CIRCLE)) {
    ps2x.read_gamepad();
    }
    digitalWrite(KN7,tay_gap);
    digitalWrite(KN8,!tay_gap);
    Serial.println("Tron ");
  }

  // X
  if (ps2x.ButtonPressed(PSB_CROSS)) 
  {
    ha();
    while (ps2x.Button(PSB_CROSS)) 
    {
      ps2x.read_gamepad();
    }
    analogWrite(PWM_5,255);
    Serial.println("X ");
  }
 // vuong
  if (ps2x.ButtonPressed(PSB_SQUARE)) {
    att_KN6 = !att_KN6; 
    while (ps2x.Button(PSB_SQUARE)) {
      ps2x.read_gamepad();
    }
    digitalWrite(KN6,att_KN6);
    Serial.println("Vuong ");
  }

  // Đọc các giá trị analog từ các cần ghi tốc độ
  

  // Xử lý giá trị analog
  // ...

  delay(2); // Delay giữa các lần đọc dữ liệu từ tay game PS2
}



void dung()
{
  analogWrite(PWM_1,0);
  analogWrite(PWM_2,0);
  analogWrite(PWM_3,0);
  analogWrite(PWM_4,0);

  digitalWrite(EN11,HIGH);
  digitalWrite(EN12,HIGH);
  digitalWrite(EN13,HIGH);
  digitalWrite(EN14,HIGH);

  digitalWrite(EN21,HIGH);
  digitalWrite(EN22,HIGH);
  digitalWrite(EN23,HIGH);
  digitalWrite(EN24,HIGH);

  analogWrite(PWM_5,255);

}
void tien()
{
  
  digitalWrite(EN11,HIGH);
  digitalWrite(EN12,LOW);

  digitalWrite(EN13,LOW);
  digitalWrite(EN14,HIGH);

  digitalWrite(EN21,LOW);
  digitalWrite(EN22,HIGH);

  digitalWrite(EN23,HIGH);
  digitalWrite(EN24,LOW);
}
void lui()
{
  digitalWrite(EN11,LOW);
  digitalWrite(EN12,HIGH);

  digitalWrite(EN13,HIGH);
  digitalWrite(EN14,LOW);

  digitalWrite(EN21,HIGH);
  digitalWrite(EN22,LOW);
  
  digitalWrite(EN23,LOW);
  digitalWrite(EN24,HIGH);
}
void trai()
{
  digitalWrite(EN11,LOW);
  digitalWrite(EN12,HIGH);

  digitalWrite(EN13,LOW);
  digitalWrite(EN14,HIGH);

  digitalWrite(EN21,HIGH);
  digitalWrite(EN22,LOW);
  
  digitalWrite(EN23,HIGH);
  digitalWrite(EN24,LOW);
}
void phai()
{
  
  digitalWrite(EN11,HIGH);
  digitalWrite(EN12,LOW);

  digitalWrite(EN13,HIGH);
  digitalWrite(EN14,LOW);

  digitalWrite(EN21,LOW);
  digitalWrite(EN22,HIGH);
  
  digitalWrite(EN23,LOW);
  digitalWrite(EN24,HIGH);
}
void quaytrai()
{
  digitalWrite(EN11,LOW);
  digitalWrite(EN12,HIGH);

  digitalWrite(EN13,LOW);
  digitalWrite(EN14,HIGH);

  digitalWrite(EN21,LOW);
  digitalWrite(EN22,HIGH);
  
  digitalWrite(EN23,LOW);
  digitalWrite(EN24,HIGH);
}
void quayphai()
{
  
  digitalWrite(EN11,HIGH);
  digitalWrite(EN12,LOW);

  digitalWrite(EN13,HIGH);
  digitalWrite(EN14,LOW);

  digitalWrite(EN21,HIGH);
  digitalWrite(EN22,LOW);
  
  digitalWrite(EN23,HIGH);
  digitalWrite(EN24,LOW);
}

void nang()
{
  digitalWrite(DIR1,HIGH);
  analogWrite(PWM_5,0);
}

void ha()
{
  digitalWrite(DIR1,LOW);
  analogWrite(PWM_5,0);
}

void tangtoc()
{
  i = 20;
  while(i<=80)
  { 
    ps2x.read_gamepad();
    analogWrite(PWM_1,i);
    analogWrite(PWM_2,i);
    analogWrite(PWM_3,i);
    analogWrite(PWM_4,i);
    if(ps2x.Button(PSB_PAD_UP)==0 && ps2x.Button(PSB_PAD_DOWN)==0 && ps2x.Button(PSB_PAD_LEFT)==0 && ps2x.Button(PSB_PAD_RIGHT)==0) break;
    //if(ps2x.Button(PSB_L1) || ps2x.Button(PSB_R1) || ps2x.Button(PSB_L2) || ps2x.Button(PSB_R2)) break;
    delay(5);
    i+=5;
  }
}


void tangtocquay()
{
  i = 10;
  while (i <= 40 )
  { 
    ps2x.read_gamepad();
    analogWrite(PWM_1,i);
    analogWrite(PWM_2,i);
    analogWrite(PWM_3,i);
    analogWrite(PWM_4,i);
    if(ps2x.Button(PSB_L1) == 0 && ps2x.Button(PSB_R1) == 0 && ps2x.Button(PSB_L2) == 0 && ps2x.Button(PSB_R2) ==0 ) break;
    if(ps2x.Button(PSB_PAD_UP) || ps2x.Button(PSB_PAD_DOWN) || ps2x.Button(PSB_PAD_LEFT) || ps2x.Button(PSB_PAD_RIGHT)) break;
    delay(7);
    if(i < 100) i+=5;
    else if(i >= 100) i+= 10;
  }
}

void giamtoc()
{
  while(i>0)
  { 
    ps2x.read_gamepad();
    analogWrite(PWM_1,i);
    analogWrite(PWM_2,i);
    analogWrite(PWM_3,i);
    analogWrite(PWM_4,i);
    if(ps2x.Button(PSB_PAD_UP) || ps2x.Button(PSB_PAD_DOWN) || ps2x.Button(PSB_PAD_LEFT) || ps2x.Button(PSB_PAD_RIGHT)) break;
    if(ps2x.Button(PSB_L1) || ps2x.Button(PSB_R1) || ps2x.Button(PSB_L2) || ps2x.Button(PSB_R2)) break;
    delay(4);
    if(i>150) i-=10;
    else if(i <= 150) i-=10;
  }
}

void mode_cham()
{
  analogWrite(PWM_1,20);
  analogWrite(PWM_2,20);
  analogWrite(PWM_3,20);
  analogWrite(PWM_4,20);
}

void mode_cham_quay()
{
  analogWrite(PWM_1,10);
  analogWrite(PWM_2,10);
  analogWrite(PWM_3,10);
  analogWrite(PWM_4,10);
}
