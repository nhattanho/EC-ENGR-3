#include <ECE3.h>
/* Saving original value of 8 sensors: right -> left, 0 -> 7 */
uint16_t sensorValues[8]; 

/* Initialize the left wheel */
const int left_nslp_pin=31; // ready for PWM
const int left_dir_pin=29; // direction
const int left_pwm_pin=40; // PWM

/* Initialize the right wheel */
const int right_nslp_pin=11; // ready for PWM
const int right_dir_pin=30; // direction
const int right_pwm_pin=39; // PWM

/* Using the led for signal */
const int ledsignal = 41;

/* Saving the 8 latest previous errors */
float preError[8];

/* The average of 8 latest previous errors*/
float averageError = 0;

/* Counting the number of times when the car meets a black line */
int meetBlackline = 0;

/* Counting the number of previous errors we got*/
int countError = 0;

/* Initialize for Kp and Kd */
float Kp = 0.02;
float Kd = 0.15;

/* Delay time for turing around */
float timeTurnAround = 390;  

/* Initialize the beginning speed */
int leftSpeed = 120;  
int rightSpeed = 120;
///////////////////////////////////
void setup() {
  pinMode(left_nslp_pin,OUTPUT);
  pinMode(left_dir_pin,OUTPUT);
  pinMode(left_pwm_pin,OUTPUT);
 
  pinMode(right_nslp_pin,OUTPUT);
  pinMode(right_dir_pin,OUTPUT);
  pinMode(right_pwm_pin,OUTPUT);
 
  digitalWrite(left_dir_pin,LOW);
  digitalWrite(left_nslp_pin,HIGH);
 
  digitalWrite(right_dir_pin,LOW);
  digitalWrite(right_nslp_pin,HIGH);
 
  pinMode(ledsignal, OUTPUT);

  for(int i = 0; i < 8; i++){
    preError[i] = 0;
  }

  ECE3_Init();
  Serial.begin(9600); // set the data rate in bits per second for serial data transmission
  delay(1000);
 }

void loop() {
  ECE3_read_IR(sensorValues);
  /* Scale all the sensors into the same range [0-1000] */
  float s1=(sensorValues[0]-483)*0.495785821;
  float s2=(sensorValues[1]-437)*0.516528926;
  float s3=(sensorValues[2]-528)*0.507099391;
  float s4=(sensorValues[3]-483)*0.689845475;
  float s5=(sensorValues[4]-528)*0.612369871;
  float s6=(sensorValues[5]-621)*0.532197978;
  float s7=(sensorValues[6]-460)*0.573065903;
  float s8=(sensorValues[7]-720)*0.575373993;

  float error = s8*(-8) + s7*(-4) + s6*(-2) + s5*(-1) + s4 + s3*2 + s2*4 + s1*8;
  averageError = getAverageError(error);
  
  float raceSpeedLeft = leftSpeed + Kp*error + Kd*(error-averageError);
  float raceSpeedRight = rightSpeed - Kp*error - Kd*(error-averageError);
  
  if(s1>650 && s2>650 && s3>650 && s4>650 && s5>650 && s6>650 && s7>650&& s8>650)
  {
    meetBlackline++;
    if(meetBlackline == 1) turnAround();
    else ending();
  }
  else race(raceSpeedLeft, raceSpeedRight);
 
  /* Updated the previous errors */
  preError[countError%8] = error;
  countError++;
}

///////////////////Helper Functions//////////////////////////
float getAverageError(float error){
  float average = 0.0;
  for(int i = 0; i < 8; i++){
    if(preError[i] == 0){
      if(countError == 0) preError[i] = error;
      else preError[i]= preError[i-1];
    }
    average += preError[i];
  }
  average = average/8;
  return average;
}

void turnAround(){
  digitalWrite(right_dir_pin,HIGH);
  analogWrite(left_pwm_pin,160);
  analogWrite(right_pwm_pin,160);
  delay(timeTurnAround);
  digitalWrite(right_dir_pin,LOW);
}


void race(float raceSpeedLeft, float raceSpeedRight ){
  analogWrite(left_pwm_pin,raceSpeedLeft);
  analogWrite(right_pwm_pin,raceSpeedRight);
}

void ending(){
  analogWrite(left_pwm_pin,0);
  analogWrite(right_pwm_pin,0);
  digitalWrite(left_nslp_pin,LOW); // turn of the left wheel
  digitalWrite(right_nslp_pin,LOW); // turn of the right wheel
}
