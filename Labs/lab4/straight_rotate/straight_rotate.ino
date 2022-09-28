// Base code.
// 
// *  NOTE: this code will do only three things:
// *    --rotate one wheel, and 
// *    --blink the right front mainboard LED.
// *    
// *  You will need to add more code to
// *  make the car do anything useful. 
// 

//#include <ECE3_LCD7.h>

//uint16_t sensorValues[8]; // right -> left, 0 -> 7

const int left_nslp_pin=31; // nslp ==> awake & ready for PWM
const int left_dir_pin=29;//back or forth > direction
const int left_pwm_pin=40;//pwm

const int right_nslp_pin=11;
const int right_pwm_pin=39;
const int right_dir_pin=30;


const int LED_RF_LEFT = 51; // LED front left
const int LED_RF_RIGHT = 41;// LED front right

///////////////////////////////////
void setup() {
// put your setup code here, to run once:
  pinMode(left_nslp_pin,OUTPUT);
  pinMode(left_dir_pin,OUTPUT);
  pinMode(left_pwm_pin,OUTPUT);

  pinMode(right_nslp_pin,OUTPUT);
  pinMode(right_dir_pin,OUTPUT);
  pinMode(right_pwm_pin,OUTPUT);
  

  digitalWrite(left_dir_pin,LOW);//forward, high is backward
  digitalWrite(left_nslp_pin,HIGH);

  digitalWrite(right_dir_pin,HIGH);//forward, high is backward
  digitalWrite(right_nslp_pin,HIGH);// ready for PWM

  pinMode(LED_RF_RIGHT, OUTPUT);
  pinMode(LED_RF_LEFT, OUTPUT);
  
//  ECE3_Init();

// set the data rate in bits/second for serial data transmission
  Serial.begin(9600); 
  delay(2000); //Wait 2 seconds before starting 
  
}

void loop() {
  // put your main code here, to run repeatedly: 
  int leftSpd = 70;//speed
  int rightSpd = 70;
//  ECE3_read_IR(sensorValues);

  analogWrite(left_pwm_pin,leftSpd);//0-255
  analogWrite(right_pwm_pin,leftSpd);//0-255

// 
  
//  ECE3_read_IR(sensorValues);

  digitalWrite(LED_RF_RIGHT, HIGH);
  digitalWrite(LED_RF_LEFT, HIGH);
  delay(250);
  digitalWrite(LED_RF_RIGHT, LOW);
  digitalWrite(LED_RF_LEFT, LOW);
  delay(250);
    
  }
