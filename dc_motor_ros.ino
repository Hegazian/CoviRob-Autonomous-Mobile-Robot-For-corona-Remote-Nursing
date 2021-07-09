#include <util/atomic.h> // For the ATOMIC_BLOCK macro

#include "ros.h"
#include "geometry_msgs/Twist.h"
#include <geometry_msgs/Point32.h> //For sending encoder msg
#include <std_msgs/String.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <ros/time.h>
#include <PID_v1.h>

const double speed_to_pwm_ratio = 0.00235;    //Ratio to convert speed (in m/s) to PWM value. It was obtained by plotting the wheel speed in relation to the PWM motor command (the value is the slope of the linear function).
const double min_speed_cmd = 0.0882;          //(min_speed_cmd/speed_to_pwm_ratio) is the minimum command value needed for the motor to start moving. This value was obtained by plotting the wheel speed in relation to the PWM motor command (the value is the constant of the linear function).


#define L 0.36
#define W 0.027
#define R 0.0345
#define pi 3.1415926
#define LOOPTIME 100
#define SMOOTH 10
#define N 13
#define gear_ratio  120
#define MAX_RPM 58
#define Kp 0.5
#define Ki 0.0
#define Kd 0.0

float x=0.0; 
float z=0.0;
float VR,VL;

unsigned long currentmillis ;
unsigned long previousmillis ;


#define PWMR 5
#define IN2 10
#define IN1 11
#define PWML 6
#define IN4 8
#define IN3 9
 
#define encoderR1 19
#define encoderR2 18

#define encoderL1 21
#define encoderL2 20

const double max_speed = 0.4;                 //Max speed in m/s
const byte noCommLoopMax = 10;                //number of main loops the robot will execute without communication before stopping
unsigned int noCommLoops = 0;                 //main loop without communication counter

double speed_reqR = 0;
double speed_reqL = 0;
double speed_actR = 0;
double speed_actL = 0;
double speed_cmd_left = 0;
double speed_cmd_right = 0;

double rpm_reqR = 0;
double rpm_reqL = 0;
double rpm_actR = 0;
double rpm_actL = 0;
double rpm_reqR_smoothed = 0;
double rpm_reqL_smoothed = 0;
double RtickOld=0.0;
double RtickNew=0.0;
double LtickOld=0.0;
double LtickNew=0.0;
int PWM_R = 0;
int PWM_L = 0;

volatile long encoderRpos = 0;
volatile long encoderLpos = 0;

const double PID_left_param[] = { 0, 0, 0.1 }; //Respectively Kp, Ki and Kd for left motor PID
const double PID_right_param[] = { 0, 0, 0.1 }; //Respectively Kp, Ki and Kd for right motor PID


PID PID_leftMotor(&speed_actL, &speed_cmd_left, &speed_reqL, PID_left_param[0], PID_left_param[1], PID_left_param[2], DIRECT);          //Setting up the PID for left motor
PID PID_rightMotor(&speed_actR, &speed_cmd_right, &speed_reqR, PID_right_param[0], PID_right_param[1], PID_right_param[2], DIRECT);   //Setting up the PID for right motor


unsigned long lastMilli = 0;       // loop timing 
unsigned long lastMillipub = 0;



int PWM_val1 = 0;
int PWM_val2 = 0;

float u = 0;
int e = 0;
  
geometry_msgs::Point32 Point_msg;

geometry_msgs::Vector3Stamped speed_msg;

ros::Publisher enc_pub("/encoder", &Point_msg);
ros::Publisher speed_pub("/speed", &speed_msg);

ros::NodeHandle nh;

void velcallback(const geometry_msgs::Twist& value){
  noCommLoops = 0;                                                  //Reset the counter for number of main loops without communication
  
  x = value.linear.x;
  z = value.angular.z;

  speed_reqL = x-z*(L/2);
  speed_reqR = x+z*(L/2);

  
}

ros::Subscriber<geometry_msgs::Twist> sub("cmd_vel" , velcallback);



void setup() {
  
  nh.initNode();
  nh.subscribe(sub);
  nh.advertise(enc_pub);
  nh.advertise(speed_pub);


  pinMode(encoderR1,INPUT_PULLUP);
  pinMode(encoderR2,INPUT_PULLUP);
  pinMode(encoderL1,INPUT_PULLUP);
  pinMode(encoderL2,INPUT_PULLUP);

  pinMode(PWMR,OUTPUT);
  pinMode(IN1,OUTPUT);
  pinMode(IN2,OUTPUT);
  pinMode(PWML,OUTPUT);
  pinMode(IN3,OUTPUT);
  pinMode(IN4,OUTPUT);

  PID_leftMotor.SetSampleTime(95);
  PID_rightMotor.SetSampleTime(95);
  PID_leftMotor.SetOutputLimits(-max_speed, max_speed);
  PID_rightMotor.SetOutputLimits(-max_speed, max_speed);
  PID_leftMotor.SetMode(AUTOMATIC);
  PID_rightMotor.SetMode(AUTOMATIC);

  attachInterrupt(digitalPinToInterrupt(encoderR1), doEncoderR1, CHANGE); 
  attachInterrupt(digitalPinToInterrupt(encoderL1), doEncoderL1, CHANGE); 
  attachInterrupt(digitalPinToInterrupt(encoderR2), doEncoderR2, CHANGE); 
  attachInterrupt(digitalPinToInterrupt(encoderL2), doEncoderL2, CHANGE);

  Serial.begin(57600);

}

void loop() {

  nh.spinOnce();

  if(millis()-lastMilli>= LOOPTIME){
    lastMilli = millis();
    if (abs(encoderLpos) < 5){                                                   //Avoid taking in account small disturbances
      speed_actL = 0;
    }
    else {
      speed_actL=((encoderLpos/N*gear_ratio)*2*PI)*(1000/LOOPTIME)*R;           // calculate speed of left wheel
    }
    if (abs(encoderRpos) < 5){                                                  //Avoid taking in account small disturbances
      speed_actR = 0;
    }
    else {
    speed_actR=((encoderRpos/N*gear_ratio)*2*PI)*(1000/LOOPTIME)*R;          // calculate speed of right wheel
    }

    encoderRpos = 0;
    encoderLpos = 0;

    speed_cmd_left = constrain(speed_cmd_left, -max_speed, max_speed);
    PID_leftMotor.Compute(); 
    // compute PWM value for left motor. Check constant definition comments for more information.
    PWM_L = constrain(((speed_reqL+sgn(speed_reqL)*min_speed_cmd)/speed_to_pwm_ratio) + (speed_cmd_left/speed_to_pwm_ratio), -255, 255); //
    
    
    if (noCommLoops >= noCommLoopMax) {                   //Stopping if too much time without command
      setMotor(0,abs(PWM_L),PWML,IN3,IN4);
    }
    else if (speed_reqL == 0){                        //Stopping
      setMotor(0,abs(PWM_L),PWML,IN3,IN4);
    }
    else if (PWM_L > 0){                          //Going forward
      setMotor(1,abs(PWM_L),PWML,IN3,IN4);
    }
    else {                                               //Going backward
      setMotor(-1,abs(PWM_L),PWML,IN3,IN4);
    } 


    speed_cmd_right = constrain(speed_cmd_right, -max_speed, max_speed);    
    PID_rightMotor.Compute();                                                 
    // compute PWM value for right motor. Check constant definition comments for more information.
    PWM_R = constrain(((speed_reqR+sgn(speed_reqR)*min_speed_cmd)/speed_to_pwm_ratio) + (speed_cmd_right/speed_to_pwm_ratio), -255, 255); // 
    

    if (noCommLoops >= noCommLoopMax) {                   //Stopping if too much time without command
      setMotor(0,abs(PWM_R),PWML,IN2,IN1);
    }
    else if (speed_reqR == 0){                        //Stopping
      setMotor(0,abs(PWM_R),PWMR,IN2,IN1);
    }
    else if (PWM_L > 0){                          //Going forward
      setMotor(1,abs(PWM_R),PWMR,IN2,IN1);
    }
    else {                                               //Going backward
      setMotor(-1,abs(PWM_R),PWMR,IN2,IN1);
    }

    if((millis()-lastMilli) >= LOOPTIME){         //write an error if execution time of the loop in longer than the specified looptime
      Serial.println(" TOO LONG ");
    }

    noCommLoops++;
    if (noCommLoops == 65535){
      noCommLoops = noCommLoopMax;
    }

    publishSpeed(LOOPTIME);
  }
  Serial.print(rpm_reqR);
  Serial.print(" , ");
  Serial.println(rpm_reqL);
  Point_msg.x=encoderRpos;
  Point_msg.y=encoderLpos;
  enc_pub.publish(&Point_msg);

}

void doEncoderR1(){

  if(digitalRead(encoderR2) == LOW){
    if(digitalRead(encoderR1) == HIGH){
      encoderRpos ++ ;
    }
    else{
      encoderRpos -- ;
    }
  }
  else{
    if(digitalRead(encoderR1) == HIGH){
      encoderRpos -- ;
    }
    else{
      encoderRpos ++ ;
    }
  }
}


void doEncoderR2(){

  if(digitalRead(encoderR1) == LOW){
    if(digitalRead(encoderR2) == HIGH){
      encoderRpos -- ;
    }
    else{
      encoderRpos ++ ;
    }
  }
  else{
    if(digitalRead(encoderR2) == HIGH){
      encoderRpos ++ ;
    }
    else{
      encoderRpos -- ;
    }
  }
}


void doEncoderL1(){

  if(digitalRead(encoderL2) == LOW){
    if(digitalRead(encoderL1) == HIGH){
      encoderLpos ++ ;
    }
    else{
      encoderLpos -- ;
    }
  }
  else{
    if(digitalRead(encoderL1) == HIGH){
      encoderLpos -- ;
    }
    else{
      encoderLpos ++ ;
    }
  }
}


void doEncoderL2(){

  if(digitalRead(encoderL1) == LOW){
    if(digitalRead(encoderL2) == HIGH){
      encoderLpos -- ;
    }
    else{
      encoderLpos ++ ;
    }
  }
  else{
    if(digitalRead(encoderL2) == HIGH){
      encoderLpos ++ ;
    }
    else{
      encoderLpos -- ;
    }
  }
}


void setMotor(int dir, int pwmVal, int pwm, int in1, int in2){
  if(pwmVal > 255){
    pwmVal == 255 ;
  }
  analogWrite(pwm,pwmVal);
  if(dir == 1){
    digitalWrite(in1,HIGH);
    digitalWrite(in2,LOW);
  }
  else if(dir == -1){
    digitalWrite(in1,LOW);
    digitalWrite(in2,HIGH);
  }
  else{
    digitalWrite(in1,LOW);
    digitalWrite(in2,LOW);
  }
  
}


void publishSpeed(double time){
    speed_msg.header.stamp = nh.now();      //timestamp for odometry data
    speed_msg.vector.x = rpm_actL;    //left wheel speed 
    speed_msg.vector.y = rpm_actR;   //right wheel speed 
    speed_msg.vector.z = time/1000;         //looptime, should be the same as specified in LOOPTIME (in s)
    speed_pub.publish(&speed_msg);
    nh.spinOnce();
}

void Actual_rpm(unsigned long time){
  RtickNew = encoderRpos;
  LtickNew = encoderLpos;
  rpm_actR = double((RtickNew-RtickOld)*60*1000)/double(time*N*gear_ratio);
  rpm_actL = double((LtickNew-LtickOld)*60*1000)/double(time*N*gear_ratio);
  RtickOld = RtickNew;
  LtickOld = LtickNew;
}

template <typename T> int sgn(T val) {
    return (T(0) < val) - (val < T(0));
}
/*int updatePid(int id, int command, double targetValue, double currentValue) {
  double pidTerm = 0;                            // PID correction
  double error = 0;
  double new_pwm = 0;
  double new_cmd = 0;
  static double last_error1 = 0;
  static double last_error2 = 0;
  static double int_error1 = 0;
  static double int_error2 = 0;
  
  error = targetValue-currentValue;
  if (id == 1) {
    int_error1 += error;
    pidTerm = Kp*error + Kd*(error-last_error1) + Ki*int_error1;
    last_error1 = error;
  }
  else {
    int_error2 += error;
    pidTerm = Kp*error + Kd*(error-last_error2) + Ki*int_error2;
    last_error2 = error;
  }
  new_pwm = constrain(double(command)*MAX_RPM/4095.0 + pidTerm, -MAX_RPM, MAX_RPM);
  new_cmd = 4095.0*new_pwm/MAX_RPM;
  return int(new_cmd);
}*/
