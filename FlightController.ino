#include <Wire.h>
//////////////////////////////////////////////////
int receiver_channel_1,receiver_channel_2,receiver_channel_3,receiver_channel_4,receiver_channel_5,receiver_channel_6;
byte last_channel_1,last_channel_2,last_channel_3,last_channel_4,last_channel_5,last_channel_6;
long timer_1,timer_2,timer_3,timer_4,timer_5,timer_6;
int start,battery,throttle;
int esc_1,esc_2,esc_3,esc_4;

////////////////////////////////////////////////
long acc_x,acc_y,acc_z;
int gyro_x,gyro_y,gyro_z,temp;
long gyro_x_cal,gyro_y_cal,gyro_z_cal;
float gyro_roll,gyro_pitch,gyro_yaw;
long loop_time;
double gyro_cal_time;
float acc_total_vector,acc_roll,acc_pitch;
float roll,pitch;
boolean set_gyro_angles;
float auto_level_roll,auto_level_pitch;

///////////////////////////////////////////////
float roll_setpoint,pitch_setpoint,yaw_setpoint;
float error,roll_output,pitch_output,yaw_output;
float i_mem_roll,i_mem_pitch,i_mem_yaw;
float pid_roll,pid_pitch,pid_yaw;
float last_roll_d_error,last_pitch_d_error,last_yaw_d_error;

///////////////////////////////////////////////
float roll_Kp = 0.5;
float roll_Ki = 0.01;
float roll_Kd = 2.0;
int roll_max_pid = 100;

float pitch_Kp=roll_Kp;
float pitch_Ki=roll_Ki;
float pitch_Kd=roll_Kd;
int pitch_max_pid = 100;

float yaw_Kp=1.0;
float yaw_Ki=0.03;
float yaw_Kd=0.0;
int yaw_max_pid = 100;

void setup() {
   TWBR = 12;
   Wire.begin();
   setup_mpu();
   
   DDRD |= 1<<DDD3;//pwm
   DDRB |= 1<<DDB1;//pwm
   DDRB |= 1<<DDB2;//pwm
   DDRB |= 1<<DDB3;//pwm

   DDRB |= 1<<DDB4;//Stearing led
   DDRB |= 1<<DDB5;//Status led
   
   PCICR |=1<<PCIE2;
   PCMSK2 |=1<<PCINT20;
   PCMSK2 |=1<<PCINT21;
   PCMSK2 |=1<<PCINT22;
   PCMSK2 |=1<<PCINT23;
   
   PCICR |=1<<PCIE1;
   PCMSK1 |=1<<PCINT10;
   PCMSK1 |=1<<PCINT11;

   TCCR1A =0;
   TCCR1B =0;
   TCCR1A |=(1<<COM1A1)|(1<<COM1B1); // non inverting pwm
   TCCR1A |=(1<<WGM10); //phase correct 8bit pwm
   TCCR1B |=(1<<CS10)|(1<<CS11);//490Hz pwm pwm prescaler 256
   
   TCCR2A =0;
   TCCR2B =0;
   TCCR2A |=(1<<COM2A1)|(1<<COM2B1);//non inverting pwm
   TCCR2A |=(1<<WGM20); // phase correct pwm
   TCCR2B |=(1<<CS22);// 490Hz pwm prescaler 256
 
    OCR1AL = 125;
    OCR1BL = 125;
    OCR2A = 125;
    OCR2B =125;
    
    delay(5000);
    
   for(int cal=0;cal<2000;cal++){
    if(cal % 100 == 0)digitalWrite(12,!digitalRead(12));
    read_mpu();
    gyro_x_cal +=gyro_x;
    gyro_y_cal +=gyro_y;
    gyro_z_cal +=gyro_z;
  }
    gyro_x_cal /=2000;
    gyro_y_cal /=2000;
    gyro_z_cal /=2000;
    
    
    
    while(receiver_channel_3 > 1015 || receiver_channel_3 <995){
      start++;
      if(start % 150 == 0){
        PORTB |=1<<PORTB4;
        delay(50);
        PORTB &=!(1<<PORTB4);
        delay(50);
        start=0;
      }
    }
    start = 0;
    //PORTB &=!(1<<PORTB4);
    battery = (float)(analogRead(A0) *1.2831);
    
    loop_time = micros();
}

void loop() {
  //PORTB |=(1<<PORTB4);
  read_mpu();
  
  gyro_x -=gyro_x_cal;
  gyro_y -=gyro_y_cal;
  gyro_z -=gyro_z_cal;

  roll_output = (roll_output *0.7) + (gyro_x/65.5)*0.3;
  pitch_output = (pitch_output *0.7) + (gyro_y/65.5)*0.3;
  yaw_output = (yaw_output *0.7) + (gyro_z/65.5)*0.3;
  
  gyro_cal_time = (double)(micros() -loop_time)/1000000;
  loop_time= micros();
  
  gyro_roll +=gyro_x * gyro_cal_time *0.015266 ;
  gyro_pitch +=gyro_y * gyro_cal_time *0.015266;

  gyro_roll += gyro_pitch * sin(gyro_z *gyro_cal_time*0.00026647);
  gyro_pitch -= gyro_roll * sin(gyro_z *gyro_cal_time*0.00026647);

  acc_total_vector = sqrt((acc_x*acc_x)+(acc_y*acc_y)+(acc_z*acc_z));

  if(abs(acc_y)<acc_total_vector){
    acc_roll = asin((float)acc_y/acc_total_vector)*57.2884;
  }
  if(abs(acc_x)<acc_total_vector){
    acc_pitch = asin((float)acc_x/acc_total_vector)*57.2884;
  }
  
  if(set_gyro_angles){
  roll = (gyro_roll*0.9) + (acc_roll*0.1);
  pitch = (gyro_pitch*0.9) + (acc_pitch*0.1);
  }

  auto_level_roll = roll * 15;
  auto_level_pitch = pitch *15;
  
  if(start == 0 && receiver_channel_3 <1010 && receiver_channel_4<1050 )start = 1;

  if(start ==1 && receiver_channel_3 <1010 && receiver_channel_4>1450){
    start =2;
    roll = acc_roll;
    pitch = acc_pitch;
    set_gyro_angles = true;

    i_mem_roll=0;
    last_roll_d_error =0;
    i_mem_pitch=0;
    last_pitch_d_error =0;
    i_mem_yaw=0;
    last_yaw_d_error =0;
  }
  if(start == 2 && receiver_channel_3<1020 && receiver_channel_4> 1980){
    start=0;
    set_gyro_angles = false;
  }

  roll_setpoint =0;
  if(receiver_channel_1>1508)roll_setpoint = receiver_channel_1 - 1508;
  else if(receiver_channel_1<1492)roll_setpoint = receiver_channel_1 - 1492;
  roll_setpoint -=auto_level_roll;
  roll_setpoint /=3.0;
 
  pitch_setpoint =0;
  if(receiver_channel_2>1508)pitch_setpoint = receiver_channel_2 - 1508;
  else if(receiver_channel_2<1492)pitch_setpoint = receiver_channel_2 - 1492;
  pitch_setpoint -=auto_level_pitch;
  pitch_setpoint /=3.0;

  yaw_setpoint =0;
  if(receiver_channel_3>1020){
  if(receiver_channel_4>1508)yaw_setpoint = (receiver_channel_4 - 1508)/3;
  else if(receiver_channel_4<1492)yaw_setpoint = (receiver_channel_4 - 1492)/3;
  }
  
  pid_calculate();

  battery = battery*0.92 * ((float)(analogRead(A0) *1.2831))*0.08;
  
  //if(battery <1020)PORTB |=1<<PORTB4;

  throttle = receiver_channel_3;

  if(start == 2){
    if(throttle>1800)throttle=1800;
    esc_1 = throttle + pid_roll - pid_pitch - pid_yaw;
    esc_2 = throttle + pid_roll + pid_pitch + pid_yaw;
    esc_3 = throttle - pid_roll + pid_pitch - pid_yaw;
    esc_4 = throttle - pid_roll - pid_pitch + pid_yaw;
    
    
    
    if(esc_1<1050)esc_1 = 1050;
    if(esc_2<1050)esc_2 = 1050;
    if(esc_3<1050)esc_3 = 1050;
    if(esc_4<1050)esc_4 = 1050;

    if(esc_1>2000)esc_1 = 2000;
    if(esc_2>2000)esc_2 = 2000;
    if(esc_3>2000)esc_3 = 2000;
    if(esc_4>2000)esc_4 = 2000;
    
  }

  else{
    esc_1=1000;
    esc_2=1000;
    esc_3=1000;
    esc_4=1000;
  }
  
 OCR2B = map(esc_1,1000,2000,125,249);
 OCR1AL = map(esc_2,1000,2000,125,249);
 OCR1BL = map(esc_3,1000,2000,125,249);
 OCR2A =map(esc_4,1000,2000,125,249);
PORTB &=!(1<<PORTB4);
}
void pid_calculate(){
  error = roll_output - roll_setpoint;
  i_mem_roll += roll_Ki*error;
  if(i_mem_roll > roll_max_pid)i_mem_roll=roll_max_pid;
  else if(i_mem_roll < roll_max_pid*-1)i_mem_roll = roll_max_pid*-1;

  pid_roll = roll_Kp*error + i_mem_roll + roll_Kd*(error - last_roll_d_error);
  if(pid_roll > roll_max_pid)pid_roll=roll_max_pid;
  else if(pid_roll < roll_max_pid*-1)pid_roll=roll_max_pid *-1;

  last_roll_d_error = error;
///////////////////////////////////////////////////////////////////////
  error = pitch_output - pitch_setpoint;
  i_mem_pitch += pitch_Ki*error;
  if(i_mem_pitch > pitch_max_pid)i_mem_pitch=pitch_max_pid;
  else if(i_mem_pitch < pitch_max_pid*-1)i_mem_pitch = pitch_max_pid*-1;

  pid_pitch = pitch_Kp*error + i_mem_pitch + pitch_Kd*(error - last_pitch_d_error);
  if(pid_pitch > pitch_max_pid)pid_pitch=pitch_max_pid;
  else if(pid_pitch < pitch_max_pid*-1)pid_pitch=pitch_max_pid *-1;

  last_pitch_d_error = error;
//////////////////////////////////////////////////////////////////
  error = yaw_output - yaw_setpoint;
  i_mem_yaw += yaw_Ki*error;
  if(i_mem_yaw > yaw_max_pid)i_mem_yaw=yaw_max_pid;
  else if(i_mem_yaw < yaw_max_pid*-1)i_mem_yaw = yaw_max_pid*-1;

  pid_yaw = yaw_Kp*error + i_mem_yaw + yaw_Kd*(error - last_yaw_d_error);
  if(pid_yaw > yaw_max_pid)pid_yaw=yaw_max_pid;
  else if(pid_yaw < yaw_max_pid*-1)pid_yaw=yaw_max_pid *-1;

  last_yaw_d_error = error;
}

ISR(PCINT2_vect){
 
  if(last_channel_1 == 0 && PIND & B00010000){
    last_channel_1 =1;
    timer_1 = micros();
  }
  else if(last_channel_1 ==1 && !(PIND & B00010000)){
    last_channel_1 =0;
    receiver_channel_1=micros() - timer_1;
  }
  
///////////////////////////////////////////////////////
    if(last_channel_2 == 0 && PIND & B00100000){
    last_channel_2 =1;
    timer_2 = micros();
  }
  else if(last_channel_2 ==1 && !(PIND & B00100000)){
    last_channel_2 =0;
    receiver_channel_2=micros() - timer_2;
  }
////////////////////////////////////////////////////////  
  if(last_channel_3 == 0 && PIND & B01000000){
    last_channel_3 =1;
    timer_3 = micros();
  }
  else if(last_channel_3 ==1 && !(PIND & B01000000)){
    last_channel_3 =0;
    receiver_channel_3=micros() - timer_3;
  }
//////////////////////////////////////////////////////////  
if(last_channel_4 == 0 && PIND & B10000000){
    last_channel_4 =1;
    timer_4 = micros();
  }
  else if(last_channel_4 ==1 && !(PIND & B10000000)){
    last_channel_4 =0;
    receiver_channel_4=micros() - timer_4;
  }
}
//////////////////////////////////////////////////////
ISR(PCINT1_vect){
  if(last_channel_5 == 0 && PINC & B00000100){
    last_channel_5 =1;
    timer_5=micros();
  }
  else if(last_channel_5 ==1 && !(PINC & B00000100)){
    last_channel_5 =0;
    receiver_channel_5 = micros()-timer_5;
  }
////////////////////////////////////////////////////
if(last_channel_6 == 0 && PINC & B00001000){
    last_channel_6 =1;
    timer_6=micros();
  }
  else if(last_channel_6 ==1 && !(PINC & B00001000)){
    last_channel_6 =0;
    receiver_channel_6 = micros()-timer_6;
  }
}
////////////////////////////////////////////////////
void setup_mpu(){
  Wire.beginTransmission(0x68);
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission(true);
//////GYRO full scale +/-500 degrees/////
  Wire.beginTransmission(0x68);
  Wire.write(0x1B);
  Wire.write(0x08);
  Wire.endTransmission(true);
  //////Accelerometer full scale +/- 8g /////
  Wire.beginTransmission(0x68);
  Wire.write(0x1C);
  Wire.write(0x10);
  Wire.endTransmission(true);
  
}

void read_mpu(){
  Wire.beginTransmission(0x68);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(0x68,14);
  while(Wire.available()<14);
  acc_x = (Wire.read()<<8)|(Wire.read());
  acc_y = (Wire.read()<<8)|(Wire.read());
  acc_z = (Wire.read()<<8)|(Wire.read());
  temp = (Wire.read()<<8)|(Wire.read());
  gyro_x = (Wire.read()<<8)|(Wire.read()) ;
  gyro_y = ((Wire.read()<<8)|(Wire.read()))* -1;
  gyro_z = ((Wire.read()<<8)|(Wire.read())) * -1;
  
}
