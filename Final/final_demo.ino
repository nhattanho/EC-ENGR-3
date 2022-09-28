
/*First version*/

#include <ECE3.h>

uint16_t sensorValues[8]; // right -> left, 0 -> 7
int c[8]; //calibration value of 8 sensors

const int left_nslp_pin=31; 
const int left_dir_pin=29;
const int left_pwm_pin=40;

const int right_nslp_pin=11; 
const int right_dir_pin=30; 
const int right_pwm_pin=39;

const int LED_RF = 41;

// minimum value of 8 sensors
const int min0 = 483;
const int min1 = 437;
const int min2 = 528;
const int min3 = 483;
const int min4 = 528;
const int min5 = 621;
const int min6 = 460;
const int min7 = 762;

// scale value for normalization 1-1000
const float scale0 = 0.495785821;
const float scale1 = 0.516528926;
const float scale2 = 0.507099391;
const float scale3 = 0.689845475;
const float scale4 = 0.612369871;
const float scale5 = 0.532197978;
const float scale6 = 0.573065903;
const float scale7 = 0.575373993;

// Base speed of 2 wheels
int leftSpd = 85;  
int rightSpd = 85;

// PID values
float Kp = 0.022;       
float Kd= 0.11;     

// other variables
int turningTime = 350; 
int count = 0;
int flag = 0;
int i = 0;
int arr[5];
int sum=0;
float avgPreError=0.0;

///////////////////////////////////
void setup() 
{
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
 
  pinMode(LED_RF, OUTPUT);
 
  ECE3_Init();
  Serial.begin(9600); 
  delay(1000); // delay 1s before running to avoid touching to the car when it starts to run
 }

void loop() 
{
  ECE3_read_IR(sensorValues);

  //calibration 8 sensors
  c[0]=(sensorValues[0]- min0) * scale0; 
  c[1]=(sensorValues[1]- min1) * scale1;  
  c[2]=(sensorValues[2]- min2) * scale2; 
  c[3]=(sensorValues[3]- min3) * scale3; 
  c[4]=(sensorValues[4]- min4) * scale4; 
  c[5]=(sensorValues[5]- min5) * scale5; 
  c[6]=(sensorValues[6]- min6) * scale6; 
  c[7]=(sensorValues[7]- min7) * scale7; 

  // weight factor 8-4-2-1
  int error = c[7]*(-8) + c[6]*(-4) + c[5]*(-2) + c[4]*(-1) + c[3] + c[2]*2 + c[1]*4 + c[0]*8;

  // calculate the average pre-error
  if (i==5) { i=0;}     
  arr[i] = error;  
  sum = 0;
  for (int n=0; n<=i; n++)
  { sum += arr[n];}
  i++;
  avgPreError = sum/i;

  // calculate motor speed
  float currentSpdLeft = leftSpd + Kp*error - Kd*(avgPreError - error);
  float currentSpdRight = rightSpd - Kp*error + Kd*(avgPreError - error);

  int numberOfSensorSeeBlack=0;    
  for(int j=0; j<8; j++)
  {
    if (c[j]>700){numberOfSensorSeeBlack++;}
  }
  
  if (count == 0 && numberOfSensorSeeBlack>=6)    // the car sees the horizontal black line
  {
    count++;
    flag++;

    // stop when seeing the black line the second time
    if(flag == 2) 
    {      
      digitalWrite(left_nslp_pin,LOW); 
      digitalWrite(right_nslp_pin,LOW); 
      analogWrite(left_pwm_pin,0);
      analogWrite(right_pwm_pin,0);
    }
    
    // turn back when seeing black line the first time
    analogWrite(left_pwm_pin,150);
    analogWrite(right_pwm_pin,150);
    digitalWrite(left_dir_pin,HIGH);
    digitalWrite(right_dir_pin,LOW);  
    delay(turningTime);    
  }
  
  else    // run forward, not seeing the horizontal black line
  {
    count = 0;    // reset count to avoid the car stopping when it sees the black line twice at the turning line right after it's done with rotating
    digitalWrite(left_dir_pin,LOW); 
    digitalWrite(right_dir_pin,LOW);
    analogWrite(left_pwm_pin,currentSpdLeft);
    analogWrite(right_pwm_pin,currentSpdRight);
  } 
}

/*Second version - Splited the code into helper functions*/
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
int leftSpeed = 110;  
int rightSpeed = 110;
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
      if(countError == 0) preError[i] = 0;
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
  digitalWrite(left_nslp_pin,LOW); // turn of the left wheel
  digitalWrite(right_nslp_pin,LOW); // turn of the right wheel
  analogWrite(left_pwm_pin,0);
  analogWrite(right_pwm_pin,0);
}
