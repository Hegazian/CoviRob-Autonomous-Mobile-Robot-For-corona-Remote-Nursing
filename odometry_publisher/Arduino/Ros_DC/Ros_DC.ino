#include <ros.h>
#include <geometry_msgs/Point32.h> //For sending encoder msg
#include<geometry_msgs/Twist.h> //For cmd_vel subscription
#include <Encoder.h> //Encoder library


Encoder R_enc(18, 19); //right motor encoder goes for pins 19 18
Encoder L_enc(20, 21); //left motor encoder goes for pins 21 20
long RoldPosition  = -999;
long LoldPosition  = -999;
long RnewPosition = 0;
long LnewPosition = 0;
 //-----------------------------------------Robot parameters definition------------ 
#define L 0.36
#define R 0.0325
//--------------------------------Motors VARS-----------------------------------
#define PWMR 5
#define IN2 10
#define IN1 11
#define PWML 6
#define IN4 8
#define IN3 9
// initializing variables
float vel=0.0; //as Twist msgs depend on Vector3 which is float64
float omega=0.0;
float VR,VL;
//-----------------------------------------------------------------------------------------
ros::NodeHandle  nh;
//------------------------------Publish init----------------------------------------------
geometry_msgs::Point32 Point_msg;

ros::Publisher enc_pub("/encoder", &Point_msg); 


//-----------------------------------DC Motors Callback subscribers

void motors_cb(const geometry_msgs::Twist& msg)
{
 
    vel=msg.linear.x;    
    omega=msg.angular.z;  
    
     
    VR=(2*vel+omega*L)/(2*R); 
    VL=(2*vel-omega*L)/(2*R); 

    //-----right motor------
    if(VR<0){
      setMotor(-1,abs(VR),PWMR,IN2,IN1);
    }
    else if(VR>0){
      setMotor(1,VR,PWMR,IN2,IN1);
    }
    else{
      setMotor(0,VR,PWMR,IN2,IN1);
    }

    if(VL<0){
      setMotor(-1,abs(VL),PWML,IN3,IN4);
    }
    else if(VL>0){
      setMotor(1,VL,PWML,IN3,IN4);
    }
    else{
      setMotor(0,VL,PWML,IN3,IN4);
    }
}
//--------------------subscribers---------------------------
ros::Subscriber<geometry_msgs::Twist> sub("/cmd_vel", &motors_cb);

void setup() {
  Serial.begin (57600);    
  // put your setup code here, to run once:
  pinMode(PWMR,OUTPUT);
  pinMode(IN1,OUTPUT);
  pinMode(IN2,OUTPUT);
  pinMode(PWML,OUTPUT);
  pinMode(IN3,OUTPUT);
  pinMode(IN4,OUTPUT);
  
  nh.advertise(enc_pub);  
  nh.subscribe(sub);      
}

void loop() {
  // put your main code here, to run repeatedly:
   //Right Encoder
     //Serial.println("OUT_ENCR");
     RnewPosition = R_enc.read();
     if (RnewPosition != RoldPosition) {
          RoldPosition = RnewPosition; 
          Serial.println(RnewPosition);
        } 
        
  //Serial.println("OUT_ENCL");
  //----left encoder
  LnewPosition = L_enc.read();
  if (LnewPosition != LoldPosition) {
      LoldPosition = LnewPosition; //update positions
      Serial.println(LnewPosition);
      }  
//-------end of encoder

//-----------------------ROS publishing  
        Point_msg.x=RnewPosition;
        Point_msg.y=LnewPosition;
        enc_pub.publish(&Point_msg);
//-------------        
      nh.spinOnce(); 
      delay(10);
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
