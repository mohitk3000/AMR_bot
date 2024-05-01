// importing libraries
//#include <ArduinoHardware.h>
#include <PID_v1.h>
#include <ros.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/Twist.h>

// Handles startup and shutdown of ROS
ros::NodeHandle nh;

//motor_pin
#define DIR1 8  //DIR2
#define PWM1 9  //PWM2

#define DIR2 10   //DIR1
#define PWM2 11   //PWM2

#define sgn(x) ((x) < 0 ? -1 : ((x) > 0 ? 1 : 0))

// Encoder output to Arduino Interrupt pin. Tracks the tick count.
int encoderA1 = 2;
int encoderB1 = 3;
int encoderA2=20;
int encoderB2=19;

// working variavles
long previousMillis = 0;
long currentMillis = 0;
const int interval = 30;
int direction_l=0;
int direction_r=0;

//ROBOT config
const int TICKS_PER_REVOLUTION = 980;
const double WHEEL_RADIUS = 0.05;
const double WHEEL_BASE = 0.41;
const double TICKS_PER_METER = 3120; 
const double max_speed= 1.2;


double velLeftWheel = 0;
double velRightWheel = 0;


// PID Parameters
const double PID_left_param[] = { 1.5, 0.1, 0.5 }; //Respectively Kp, Ki and Kd for left motor PID
const double PID_right_param[] = { 1.5, 0.1, 0.5 }; //Respectively Kp, Ki and Kd for right motor PID

double speed_cmd_left = 0; 
double speed_cmd_right = 0; 
double speed_req_left = 0; 
double speed_req_right = 0; 
double speed_act_left = 0; 
double speed_act_right = 0; 


std_msgs::Float32 vel_left;
std_msgs::Float32 vel_right;

// speed publisher
ros::Publisher leftspeed("left_speed", &vel_left);
ros::Publisher rightspeed("right_speed", &vel_right);

//Setting up the PID for motors (input,output, setpoint, params)
PID PID_leftMotor(&speed_act_left, &speed_cmd_left, &speed_req_left, PID_left_param[0], PID_left_param[1], PID_left_param[2], DIRECT);       
PID PID_rightMotor(&speed_act_right, &speed_cmd_right, &speed_req_right, PID_right_param[0], PID_right_param[1], PID_right_param[2], DIRECT);   

const int PWM_TURN = 80; // Turning PWM output (0 = min, 255 = max for PWM values)

// Set maximum and minimum limits for the PWM values about 0.8 m/s and 0.1m/s
const int PWM_MIN = 50; 
const int PWM_MAX = 230; 

 //PWM command for both motors
int PWM_leftMotor = 0;                    
int PWM_rightMotor = 0;  

//Ratio to convert speed (in m/s) to PWM value. 
//It was obtained by plotting the wheel speed in relation to the PWM motor command (the value is the slope of the linear function).
//const double speed_to_pwm_ratio = 0.00235;    
const double speed_to_pwm_ratio = 0.065; 
const double min_speed_cmd = 0.0882 ;

// double w_r=0, w_l=0;

//wheel_rad is the wheel radius ,wheel_sep is wheel gap
double wheel_rad = 0.05, wheel_sep = 0.390;
double speed_ang=0, speed_lin=0;


// publisher for pulses
std_msgs::Int16 pulse_2;
ros::Publisher rightPub("right_ticks", &pulse_2);
 
std_msgs::Int16 pulse_1;
ros::Publisher leftPub("left_ticks", &pulse_1);


//callback of cmd_vel
void messageCb( const geometry_msgs::Twist& msg){
  speed_ang = msg.angular.z;
  speed_lin = msg.linear.x;
  speed_req_left = (speed_lin/wheel_rad) + ((speed_ang*wheel_sep)/(2.0*wheel_rad));
  speed_req_right = (speed_lin/wheel_rad) - ((speed_ang*wheel_sep)/(2.0*wheel_rad));
}

//subscriber for velocity
ros::Subscriber<geometry_msgs::Twist> sub("cmd_vel", &messageCb );

void A1_CHANGE(){
  if( digitalRead(encoderB1) == 0 ) {
    if ( digitalRead(encoderA1) == 0 ) {
      // A fell, B is low
      pulse_1.data--; // moving reverse
      
    } else {
      
      // A rose, B is lowspeed_ang = msg.angular.z;
      pulse_1.data++; // moving forward
    }
  }else 
  {
    if ( digitalRead(encoderA1) == 0 ) {
      // B fell, A is high
      pulse_1.data++; // moving reverse
    } else {
      // B rose, A is high
      pulse_1.data--; // moving forward
    }
  }

}

void B1_CHANGE(){
  if ( digitalRead(encoderA1) == 0 ) {
    if ( digitalRead(encoderB1) == 0 ) {
      // B fell, A is low
      pulse_1.data++; // moving forward
    } else {
      // B rose, A is low
      pulse_1.data--; // moving reverse
    }
 } else {
    if ( digitalRead(encoderB1) == 0 ) {
      // B fell, A is high
      pulse_1.data--; // moving reverse
    } else 
    {
      // B rose, A is high
      pulse_1.data++; // moving forward
    }
  }

}

void A2_CHANGE(){
  if( digitalRead(encoderB2) == 0 ) {
    if ( digitalRead(encoderA2) == 0 ) {
      // A fell, B is low
      pulse_2.data--; // moving reverse
      
    } else {
      
      // A rose, B is low
      pulse_2.data++; // moving forward
    }
  }else {
    if ( digitalRead(encoderA2) == 0 ) {
      // B fell, A is high
      pulse_2.data++; // moving reverse
    } else {
      // B rose, A is high
      pulse_2.data--; // moving forward
    }
  }

}

void B2_CHANGE(){
  if ( digitalRead(encoderA2) == 0 ) {
    if ( digitalRead(encoderB2) == 0 ) {
      // B fell, A is low
      pulse_2.data++; // moving forward
    } else {
      // B rose, A is low
      pulse_2.data--; // moving reverse
    }
 } else {
    if ( digitalRead(encoderB2) == 0 ) {
      // B fell, A is high
      pulse_2.data--; // moving reverse
    } else {
      // B rose, A is high
      pulse_2.data++; // moving forward
    }
  }

}
void calc_vel_left_wheel(){
   
  // Previous timestamp
  static double prevTime = 0;
   
  // Variable gets created and initialized the first time a function is called.
  static int prevLeftCount = 0;
 
  // Manage rollover and rollunder when we get outside the 16-bit integer range 
  int numOfTicks = (65535 + pulse_2.data - prevLeftCount) % 65535;
 
  // If we have had a big jump, it means the tick count has rolled over.
  if (numOfTicks > 10000) {
        numOfTicks = 0 - (65535 - numOfTicks);
  }
 
  // Calculate wheel velocity in meters per second
  velLeftWheel = (numOfTicks/TICKS_PER_METER/((millis())-prevTime))*1000;
  speed_act_left=velLeftWheel;
  // vel_left.data=velLeftWheel;
  // Keep track of the previous tick count
  prevLeftCount = pulse_2.data;
 
  // Update the timestamp
  prevTime = (millis());
//  Serial.print("left:");
//  Serial.println(velLeftWheel);
}

void calc_vel_right_wheel(){
   
  // Previous timestamp
  static double prevTime = 0;
   
  // Variable gets created and initialized the first time a function is called.
  static int prevRightCount = 0;
 
  // Manage rollover and rollunder when we get outside the 16-bit integer range 
  int numOfTicks = (65535 + pulse_1.data - prevRightCount) % 65535;
 
  if (numOfTicks > 10000) {
        numOfTicks = 0 - (65535 - numOfTicks);
  }
 
  // Calculate wheel velocity in meters per second
  velRightWheel = (numOfTicks/TICKS_PER_METER/((millis())-prevTime))*1000;
  speed_act_right=velRightWheel;
  vel_right.data=velRightWheel;

  prevRightCount = pulse_1.data;
   
  prevTime = (millis());


}

void Motors_init();

void setup() {
  // Serial.begin(115200);
  // Set pin states of the encoder
  Motors_init();
  pinMode(encoderA1, INPUT);
  pinMode(encoderB1, INPUT);
  pinMode(encoderA2, INPUT);
  pinMode(encoderB2, INPUT);
 
  // Every time the pin goes high, this is a tick
  attachInterrupt(0, A1_CHANGE, CHANGE);
  attachInterrupt(1, B1_CHANGE, CHANGE);
  attachInterrupt(3, A2_CHANGE, CHANGE);
  attachInterrupt(4, B2_CHANGE, CHANGE);

  PID_leftMotor.SetSampleTime(95);
  PID_rightMotor.SetSampleTime(95);
  PID_leftMotor.SetOutputLimits(-max_speed, max_speed);
  PID_rightMotor.SetOutputLimits(-max_speed, max_speed);
  PID_leftMotor.SetMode(AUTOMATIC);
  PID_rightMotor.SetMode(AUTOMATIC);

  nh.getHardware()->setBaud(115200);
  nh.initNode();
  nh.advertise(rightPub);
  nh.advertise(leftPub);
  nh.advertise(rightspeed);
  nh.advertise(leftspeed);
  nh.subscribe(sub);
 
}
 
void loop() {
   
  //Record the time  
  currentMillis = millis();
 
  // If 100ms have passed, print the number of ticks
  if (currentMillis - previousMillis > 10) {  
     
    previousMillis = currentMillis;
    // Calculate the velocity of the right and left wheels
    calc_vel_right_wheel();
    calc_vel_left_wheel();
    rightPub.publish( &pulse_2);
    leftPub.publish( &pulse_1 );
    rightspeed.publish( &vel_right);
    leftspeed.publish( &vel_left );
  }

    speed_cmd_left = constrain(speed_cmd_left, -max_speed, max_speed);
    PID_leftMotor.Compute();                                                 
    // compute PWM value for left motor. Check constant definition comments for more information.
    PWM_leftMotor = constrain(((speed_req_left+sgn(speed_req_left)*min_speed_cmd)/speed_to_pwm_ratio) + (speed_cmd_left/speed_to_pwm_ratio), -255, 255); //
    vel_left.data=PWM_leftMotor;
    speed_cmd_right = constrain(speed_cmd_right, -max_speed, max_speed);    
    PID_rightMotor.Compute();                                                 
    // compute PWM value for right motor. Check constant definition comments for more information.
    PWM_rightMotor = constrain(((speed_req_right+sgn(speed_req_right)*min_speed_cmd)/speed_to_pwm_ratio) + (speed_cmd_right/speed_to_pwm_ratio), -255, 255); // 

  MotorR(PWM_rightMotor);
  MotorL(PWM_leftMotor);
  nh.spinOnce();
}

void Motors_init(){

 pinMode(DIR1, OUTPUT);

 pinMode(PWM1, OUTPUT);

 pinMode(PWM2, OUTPUT);

 pinMode(DIR2, OUTPUT);

 digitalWrite(DIR1, LOW);

 digitalWrite(DIR2, LOW);

 digitalWrite(PWM1, LOW);

 digitalWrite(PWM2, LOW);


}

void MotorL(int Pulse_Width1){
 if (Pulse_Width1 > 0){

     analogWrite(PWM1, Pulse_Width1);

     digitalWrite(DIR1, HIGH);


 }

 if (Pulse_Width1 < 0){

     Pulse_Width1=abs(Pulse_Width1);

     analogWrite(PWM1, Pulse_Width1);

     digitalWrite(DIR1, LOW);


 }

 if (Pulse_Width1 == 0){

     analogWrite(PWM1, Pulse_Width1);

     digitalWrite(DIR1, LOW);

 }

}


void MotorR(int Pulse_Width2){


 if (Pulse_Width2 > 0){

     analogWrite(PWM2, Pulse_Width2);

     digitalWrite(DIR2, HIGH);

 }

 if (Pulse_Width2 < 0){

     Pulse_Width2=abs(Pulse_Width2);

     analogWrite(PWM2, Pulse_Width2);

     digitalWrite(DIR2, LOW);

 }

 if (Pulse_Width2 == 0){

     analogWrite(PWM2, Pulse_Width2);

     digitalWrite(DIR2, LOW);

 }

}
