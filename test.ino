
#include <Arduino.h>
#include <AFMotor.h>

//defining Sensor: Từ phải sang trái (Motor 3 -> 4)
#define sensorIR1 A0
#define sensorIR2 A1
#define sensorIR3 A2
#define sensorIR4 A3

//defining motors

AF_DCMotor motor3(3, MOTOR34_1KHZ); // Motor phải 
AF_DCMotor motor4(4, MOTOR34_1KHZ); // Motor trái
int sensor[4] = {0, 0, 0, 0}; // giữ giá trị cảm biến đọc vào

//
float  error, P, I, D, PID_value, previous_error = 0;
int Kp, Ki, Kd, speed_motor, left_speed, right_speed;

int mapline;// 0 là mapline trắng, line đen || 1 là map đen line trắng

void setup() {
  Serial.begin(9600);
  pinMode(sensorIR1, INPUT);
  pinMode(sensorIR2, INPUT);
  pinMode(sensorIR3, INPUT);
  pinMode(sensorIR4, INPUT);
  
  // Set up 2 Motor 3,4 cùng tiến
  motor3.run(FORWARD);
  motor4.run(FORWARD);
  // init value
  // Tham số thử nghiệm
}
 
void loop() { 


   
   calculate_map();
   calculate_error();

   delay(10);
   //calculate_PID();

   control_motors(); 


   
   Serial.print(right_speed); 
   Serial.print("  ");
   Serial.println(left_speed);
}

void calculate_error(){
    sensor[0] = digitalRead(sensorIR1);
    sensor[1] = digitalRead(sensorIR2);
    sensor[2] = digitalRead(sensorIR3);
    sensor[3] = digitalRead(sensorIR4);

    Serial.print(sensor[0]);    
    Serial.print(sensor[1]);    
    Serial.print(sensor[2]);    
    Serial.println(sensor[3]);

    if(mapline == 0){

      if(sensor[0] == 1 && sensor[1] == 0 && sensor[2] == 0 && sensor[3] == 0) error = 3;  // 0 0 0 1
      else if(sensor[0] == 1 && sensor[1] == 1 && sensor[2] == 0 && sensor[3] == 0) error = 2;  // 0 0 1 1
      else if(sensor[0] == 0 && sensor[1] == 1 && sensor[2] == 0 && sensor[3] == 0) error = 1;  // 0 0 1 0
      else if(sensor[0] == 0 && sensor[1] == 1 && sensor[2] == 1 && sensor[3] == 0) error = 0;  // 0 1 1 0
      else if(sensor[0] == 0 && sensor[1] == 0 && sensor[2] == 1 && sensor[3] == 0) error = -1;  // 0 1 0 0
      else if(sensor[0] == 0 && sensor[1] == 0 && sensor[2] == 1 && sensor[3] == 1) error = -2;  // 1 1 0 0
      else if(sensor[0] == 0 && sensor[1] == 0 && sensor[2] == 0 && sensor[3] == 1) error = -3;  // 1 0 0 0
      else if(sensor[0] == 0 && sensor[1] == 0 && sensor[2] == 0 && sensor[3] == 0) error = previous_error;  // 0 0 0 0
        
    }else if(mapline == 1){
          
      if(sensor[0] == 0 && sensor[1] == 1 && sensor[2] == 1 && sensor[3] == 1) error = 3;  // 1 1 1 0
      else if(sensor[0] == 0 && sensor[1] == 0 && sensor[2] == 1 && sensor[3] == 1) error = 2;  // 1 1 0 0 
      else if(sensor[0] == 1 && sensor[1] == 0 && sensor[2] == 1 && sensor[3] == 1) error = 1;  // 1 1 0 1
      else if(sensor[0] == 1 && sensor[1] == 0 && sensor[2] == 0 && sensor[3] == 1) error = 0;  // 1 0 0 1
      else if(sensor[0] == 1 && sensor[1] == 1 && sensor[2] == 0 && sensor[3] == 1) error = -1;  // 1 0 1 1 
      else if(sensor[0] == 1 && sensor[1] == 1 && sensor[2] == 0 && sensor[3] == 0) error = -2;  // 0 0 1 1 
      else if(sensor[0] == 1 && sensor[1] == 1 && sensor[2] == 1 && sensor[3] == 0) error = -3;  // 0 1 1 1 
      else if(sensor[0] == 1 && sensor[1] == 1 && sensor[2] == 1 && sensor[3] == 1) error = previous_error;  // 1 1 1 1 
    }
    


}

void control_motors(){

  motor3.setSpeed(right_speed);  
  motor4.setSpeed(left_speed);

  if(error == 3){
    right_speed = 35;
    left_speed = 75;
  }
  else if(error == 2){
    right_speed = 35;
    left_speed = 65;
    if (previous_error >= 2) left_speed = 80;
  }else if(error == 1){
    right_speed = 50;
    left_speed = 65;
    if (previous_error >= 2) left_speed = 75;
  }else if(error == 0){
    right_speed = 60;
    left_speed = 60;
  }else if(error == -1){
    right_speed = 65;
    left_speed = 50;
  }else if(error == -2){
    right_speed = 65;
    if (previous_error >= 2) right_speed = 75;
    left_speed = 35;
  }else if(error == -3){
    right_speed = 75;
    if (previous_error >= 2) right_speed = 80;
    left_speed = 35;
  }
  previous_error = error;
  
  left_speed = constrain(left_speed, 0, 225);
  right_speed = constrain(right_speed, 0, 225);
 
}

void calculate_map(){
  if((sensor[0] == 0 && sensor[1] == 1 && sensor[2] == 1 && sensor[3] == 0) || (sensor[0] == 0 && sensor[1] == 0 && sensor[2] == 0 && sensor[3] == 0)){
    mapline = 0;//map trang
  }

  if((sensor[0] == 1 && sensor[1] == 0 && sensor[2] == 0 && sensor[3] == 1) || (sensor[0] == 1 && sensor[1] == 1 && sensor[2] == 1 && sensor[3] == 1)){
    mapline = 1;//map den 
  }
  
}
