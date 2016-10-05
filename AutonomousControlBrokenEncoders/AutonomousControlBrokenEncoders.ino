#if defined(ARDUINO) && ARDUINO >= 100
  #include "Arduino.h"
#else
  #include "WProgram.h"
#endif

#include <Servo.h>
#include <ros.h>
#include <ros/time.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>



//#include <digitalWriteFast.h>  // library for high performance reads and writes by jrraines
                               // see http://www.arduino.cc/cgi-bin/yabb2/YaBB.pl?num=1267553811/0
                               // and http://code.google.com/p/digitalwritefast/
 
// It turns out that the regular digitalRead() calls are too slow and bring the arduino down when
// I use them in the interrupt routines while the motor runs at full speed creating more than
// 40000 encoder ticks per second per motor.
 
 
// 18, 19, 20, 21
// Quadrature encoders
// Left encoder
#define c_LeftEncoderInterrupt 5
#define c_LeftEncoderPinA 18
#define c_LeftEncoderPinB 19
#define LeftEncoderIsReversed
volatile bool _LeftEncoderBSet;
volatile long _LeftEncoderTicks = 0;
 
// Right encoder
#define c_RightEncoderInterrupt 3
#define c_RightEncoderPinA 20
#define c_RightEncoderPinB 21
volatile bool _RightEncoderBSet;
volatile long _RightEncoderTicks = 0;

ros::NodeHandle  nh;

Servo left1;
Servo left2;
Servo left3;

Servo right1;
Servo right2;
Servo right3;

// for smoothing
int current_left = 90;
int current_right = 90;

double ddl = 0;
double ddr = 0;

const int max_speed = 128/3;

long lastMotorCommand = 0;
char debug_string[200];

long last_odom_time = millis();
long new_odom_time = millis();
long dt = 1;
double V=0,W=0;
#define dist_per_count 0.000740485
// 21 in in meters
#define wheel_distance 0.5334

std_msgs::String str_msg;
ros::Publisher debug_pub("arduino_debug", &str_msg);

//geometry_msgs::Twist controller_cmd_vel_msg;
//ros::Publisher ctrl_pub("controller_cmd_vel", &controller_cmd_vel_msg);

geometry_msgs::Twist odom_msg;
ros::Publisher pub("odom_raw", &odom_msg);

void write_debug(String debugger) {
  debugger.toCharArray(debug_string, 200);
  str_msg.data = debug_string;
  debug_pub.publish(&str_msg);
}

void enca1() {
  write_debug(String("A1\n"));
}

void enca2() {
  write_debug(String("A2\n"));
}

void encb1() {
  write_debug(String("B1\n"));
}

void encb2() {
  write_debug(String("B2\n"));
}

void cmd_vel_cb( const geometry_msgs::Twist& cmd_msg){
  if(digitalRead(25) == HIGH) {
   return; 
  }
  vel_cb(cmd_msg);
}

void key_vel_cb( const geometry_msgs::Twist& cmd_msg){
  if(digitalRead(25) == LOW) {
   return; 
  }
  vel_cb(cmd_msg);
}

void vel_cb( const geometry_msgs::Twist& cmd_msg){
  /* Reset the auto stop timer */
  lastMotorCommand = millis();
  
  //String debugger = "Got message: ";
  //debugger += String(cmd_msg.linear.x) + " ";
  //debugger += String(cmd_msg.angular.z) + "\n";
  
  float rawy = cmd_msg.linear.x; // m/s
  float rawx = cmd_msg.angular.z; // rad/s

  int x, y;
  
  x = (int)(double_map(rawx, -1, 1, -1 * max_speed, max_speed) * 1.800);
  y = (int)(double_map(rawy, 1, -1, -1 * max_speed, max_speed) * 0.375);

  x = max(-128, min(128, x));
  y = max(-128, min(128, y));

  //debugger += String(x) + " ";
  //debugger += String(y) + "\n";

  //write_debug(debugger);
  
  updateArcadeDrive(x,y);
  digitalWrite(13, HIGH-digitalRead(13));  //toggle led  
}

ros::Subscriber<geometry_msgs::Twist> sub("cmd_vel", cmd_vel_cb);
ros::Subscriber<geometry_msgs::Twist> key_sub("key_vel", key_vel_cb);

void setup() {

 

  left1.attach(6, 1000, 2000);
  left2.attach(7, 1000, 2000);
  left3.attach(8, 1000, 2000);
  right1.attach(3, 1000, 2000);
  right2.attach(4, 1000, 2000);
  right3.attach(5, 1000, 2000);


  odom_msg.linear.x = 0.0;
  odom_msg.linear.y = 0.0;
  odom_msg.linear.z = 0.0;
  odom_msg.angular.x = 0.0;
  odom_msg.angular.y = 0.0;
  odom_msg.angular.z = 0.0;
  
  nh.initNode();
  nh.subscribe(sub);
  nh.subscribe(key_sub);
  nh.advertise(pub);
  //nh.advertise(ctrl_pub);
  nh.advertise(debug_pub);
 
  // Quadrature encoders
  // Left encoder
  pinMode(c_LeftEncoderPinA, INPUT);      // sets pin A as input
  digitalWrite(c_LeftEncoderPinA, LOW);  // turn on pullup resistors
  pinMode(c_LeftEncoderPinB, INPUT);      // sets pin B as input
  digitalWrite(c_LeftEncoderPinB, LOW);  // turn on pullup resistors
  attachInterrupt(c_LeftEncoderInterrupt, HandleLeftMotorInterruptA, RISING);
 
  // Right encoder
  pinMode(c_RightEncoderPinA, INPUT);      // sets pin A as input
  digitalWrite(c_RightEncoderPinA, LOW);  // turn on pullup resistors
  pinMode(c_RightEncoderPinB, INPUT);      // sets pin B as input
  digitalWrite(c_RightEncoderPinB, LOW);  // turn on pullup resistors
  attachInterrupt(c_RightEncoderInterrupt, HandleRightMotorInterruptA, RISING);
}



// x and y must be between -128 and 128
void updateArcadeDrive(int x, int y) {
  
  int l = y + x;
  int r = y - x;
  
  l = map(l, 128, -128, 0, 180);
  r = map(r, -128, 128, 0, 180);
  
  l = max(0, min(180, l));
  r = max(0, min(180, r));
  set_left(l);
  set_right(r);
}

void set_left(int l) {
  left1.write(l);
  left2.write(l);
  left3.write(l);
}

void set_right(int r) {
  right1.write(r);
  right2.write(r);
  right3.write(r);
}

double double_map(double x, double in_min, double in_max, double out_min, double out_max)
{
 return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void loop() {
  nh.spinOnce();
  
  //print_controller_speeds();
  if (lastMotorCommand + 1000 < millis()) {
    // stop cart if a third of a secod goes by without a command
    // probably means that the jeston died
    //set_left(90);
    //set_right(90); 
    updateArcadeDrive(0,0);
    String debugger = "Timed out, maybe bad connection";
    write_debug(debugger);
  }
  
  // do odometry
  
  new_odom_time = millis();
  dt = new_odom_time - last_odom_time;
  last_odom_time = new_odom_time;
  
  // dist_per_count = distance traveled per count, delta_left = ticks moved
  double d_left = (_LeftEncoderTicks * dist_per_count); // Left dist
  double d_right = (_RightEncoderTicks * dist_per_count); // Right dist

  // to hot fix broken encoders
  d_left = d_right;

  ddl += d_left;
  ddr += d_right;

  write_debug("L " + String(ddl, 10));
  write_debug("R " + String(ddr, 10));


  V = (d_left + d_right)/(2000 * dt);
  W = (d_right - d_left)/(1000 * dt);

  //write_debug("W " + String(W, 10));
  //write_debug("V " + String(V, 10));
  
  odom_msg.linear.x = V;
  odom_msg.angular.z = W;
  pub.publish(&odom_msg);
  _LeftEncoderTicks = 0;
  _RightEncoderTicks = 0;
  
  
  delay(5);  
}


// Interrupt service routines for the left motor's quadrature encoder
void HandleLeftMotorInterruptA()
{
  // Test transition; since the interrupt will only fire on 'rising' we don't need to read pin A
  _LeftEncoderBSet = digitalRead(c_LeftEncoderPinB);   // read the input pin
 
  // and adjust counter + if A leads B
  #ifdef LeftEncoderIsReversed
    _LeftEncoderTicks -= _LeftEncoderBSet ? -1 : +1;
  #else
    _LeftEncoderTicks += _LeftEncoderBSet ? -1 : +1;
  #endif
}
 
// Interrupt service routines for the right motor's quadrature encoder
void HandleRightMotorInterruptA()
{
  // Test transition; since the interrupt will only fire on 'rising' we don't need to read pin A
  _RightEncoderBSet = digitalRead(c_RightEncoderPinB);   // read the input pin
 
  // and adjust counter + if A leads B
  #ifdef RightEncoderIsReversed
    _RightEncoderTicks -= _RightEncoderBSet ? -1 : +1;
  #else
    _RightEncoderTicks += _RightEncoderBSet ? -1 : +1;
  #endif
}
