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

int first_controller_pin = 40;
int last_controller_pin = 47;

int gap = 8;

long controller_speeds[8];

Servo left1;
Servo left2;
Servo left3;

Servo right1;
Servo right2;
Servo right3;

const int max_speed = 128/3;

long lastMotorCommand = 0;
char debug_string[200];

std_msgs::String str_msg;
ros::Publisher debug_pub("arduino_debug", &str_msg);

geometry_msgs::Twist controller_cmd_vel_msg;
ros::Publisher pub("controller_cmd_vel", &controller_cmd_vel_msg);

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
  /* Reset the auto stop timer */
  lastMotorCommand = millis();
  
  //String debugger = "Got message: ";
  //debugger += String(cmd_msg.linear.x) + " ";
  //debugger += String(cmd_msg.angular.z) + "\n";
  
  float rawy = cmd_msg.linear.x; // m/s
  float rawx = cmd_msg.angular.z; // rad/s

  int x, y;
  
  x = (int)double_map(rawx, -1, 1, -1 * max_speed, max_speed);
  y = (int)double_map(rawy, 1, -1, -1 * max_speed, max_speed);

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

  for(int i = first_controller_pin; i <= last_controller_pin; i++) {
    pinMode(i, INPUT);
  }
  
  for(int i = 0; i <= 7; i++) {
    controller_speeds[i] = 0;
    //controller_speeds_min[i] = 9999;
    //controller_speeds_max[i] = 0;
  }

  left1.attach(6, 1000, 2000);
  left2.attach(7, 1000, 2000);
  left3.attach(8, 1000, 2000);
  right1.attach(3, 1000, 2000);
  right2.attach(4, 1000, 2000);
  right3.attach(5, 1000, 2000);

  controller_cmd_vel_msg.linear.x = 0.0;
  controller_cmd_vel_msg.linear.y = 0.0;
  controller_cmd_vel_msg.linear.z = 0.0;
  controller_cmd_vel_msg.angular.x = 0.0;
  controller_cmd_vel_msg.angular.y = 0.0;
  controller_cmd_vel_msg.angular.z = 0.0;
  
  nh.initNode();
  nh.subscribe(sub);
  nh.advertise(pub);
  nh.advertise(debug_pub);

  attachInterrupt(digitalPinToInterrupt(18), enca1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(19), enca2, CHANGE);
  attachInterrupt(digitalPinToInterrupt(20), encb1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(21), encb2, CHANGE);
}

void update_controller_speeds() {
  for(int i = 0; i <= 7; i++) {
    controller_speeds[i] = pulseIn(i + first_controller_pin, HIGH, 40000);
  }
}

int get_controller_speed(int i) {
    int value = map(controller_speeds[i], 1000, 1900, 0, 255);
    return max(0, min(255, value));
}

String get_controller_speeds() {
 String s = "";
 for(int i = 0; i <= 7; i++) {
    s += String(i + first_controller_pin) + " ";
    //s += controller_names[i] + " ";
    s += String(get_controller_speed(i)) + "\n";
  }
  s += "\n";
  return s;
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
  
  update_controller_speeds();
  //print_controller_speeds();
  
  if(digitalRead(25) == HIGH) {
    
  }
  
  if(true) {  
    double x = get_controller_speed(7) - 128;
    double y = get_controller_speed(5) - 128;

    int xx = 0;
    if(x > gap) {
      xx = map(x, x + gap,  128, 0,  128);
    } else if(x < -1 * gap) {
      xx = map(x, x - gap, -128, 0, -128); 
    }
  
    int yy = 0;
    if(y > gap) {
      yy = map(y, y + gap,  128, 0,  128);
    } else if(y < -1 * gap) {
      yy = map(y, y - gap, -128, 0, -128); 
    }
  
    double linear_x  = double_map(y, -128.0, 128.0, -1.0, 1.0);
    double angular_z = double_map(x, -128.0, 128.0, 1.0, -1.0);

    double angular_x = get_controller_speed(3);

    controller_cmd_vel_msg.linear.x = linear_x; // m/s
    controller_cmd_vel_msg.angular.z = angular_z; // rad/s
    controller_cmd_vel_msg.angular.x = angular_x; // rad/s

    // push to ros
    pub.publish( &controller_cmd_vel_msg );

    //write_debug(get_controller_speeds());   
  }

  if (lastMotorCommand + 1000 < millis()) {
    // stop cart if a third of a secod goes by without a command
    // probably means that the jeston died
    //set_left(90);
    //set_right(90); 
    updateArcadeDrive(0,0);
    String debugger = "Timed out, maybe bad connection";
    write_debug(debugger);
  }
  
  delay(5);  
}
