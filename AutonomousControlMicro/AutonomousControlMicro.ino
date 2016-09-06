#if (ARDUINO >= 100)
 #include <Arduino.h>
#else
 #include <WProgram.h>
#endif

#include <Servo.h>
#include <ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>

ros::NodeHandle  nh;

Servo left1;
Servo right1;

const int max_speed = 128/3;

long lastMotorCommand = 0;
//char debug_string[50];

std_msgs::String str_msg;
ros::Publisher debug_pub("arduino_debug2", &str_msg);

//void write_debug(String debugger) {
//  debugger.toCharArray(debug_string, 200);
//  str_msg.data = debug_string;
//  debug_pub.publish(&str_msg);
//}

void cmd_vel_cb( const geometry_msgs::Twist& cmd_msg){
  /* Reset the auto stop timer */
  lastMotorCommand = millis();
  
  //String debugger = "Got message: ";
  //debugger += String(cmd_msg.linear.x) + " ";
  //debugger += String(cmd_msg.angular.z) + "\n";
  
  float rawy = cmd_msg.linear.x; // m/s
  float rawx = cmd_msg.angular.z; // rad/s

  int x, y;
  
  x = (int)double_map(rawx, 1, -1, -1 * max_speed, max_speed);
  y = (int)double_map(rawy, -1, 1, -1 * max_speed, max_speed);

  x = max(-128, min(128, x));
  y = max(-128, min(128, y));

  //debugger += String(x) + " ";
  //debugger += String(y) + "\n";

  //write_debug(debugger);
  
  updateArcadeDrive(x,y);
  digitalWrite(13, HIGH-digitalRead(13));  //toggle led  
}

ros::Subscriber<geometry_msgs::Twist> sub("cmd_vel", cmd_vel_cb);

void setup() {

  pinMode(13, OUTPUT);
  
  left1.attach(10, 1000, 2000);
  right1.attach(11, 1000, 2000);
  
  nh.initNode();
  nh.subscribe(sub);
  nh.advertise(debug_pub);
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
}

void set_right(int r) {
  right1.write(r);
}

double double_map(double x, double in_min, double in_max, double out_min, double out_max)
{
 return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

bool light=true;

void loop() {
  nh.spinOnce();

  if (lastMotorCommand + 500 < millis()) {
    // stop cart if a third of a secod goes by without a command
    // probably means that the jeston died
    //set_left(90);
    //set_right(90); 
    updateArcadeDrive(0,0);
    //String debugger = "Timed out, maybe bad connection";
    //write_debug(debugger);
    light = !light;
    digitalWrite(13, light?HIGH:LOW);
  }
  
  delay(10);  
}
