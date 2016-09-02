#include <Servo.h>

int first_controller_pin = 40;
int last_controller_pin = 47;

int gap = 8;

long controller_speeds[8];


String controller_names[] = {"?", "?", "Switch?", "Top Left Switch", "left left/right", "left up/down", "right up/down", "right left/right"};

Servo left1;
Servo left2;
Servo left3;

Servo right1;
Servo right2;
Servo right3;

void setup() {

  for(int i = first_controller_pin; i <= last_controller_pin; i++) {
    pinMode(i, INPUT);
  }
  
  for(int i = 0; i <= 7; i++) {
    controller_speeds[i] = 0;
  }

  Serial.begin(9600);
  while (!Serial);             // Leonardo: wait for serial monitor
  Serial.println("\nPeachBot");

  left1.attach(6, 1000, 2000);
  left2.attach(7, 1000, 2000);
  left3.attach(8, 1000, 2000);
  right1.attach(3, 1000, 2000);
  right2.attach(4, 1000, 2000);
  right3.attach(5, 1000, 2000);
}


void update_controller_speeds() {
  for(int i = 0; i <= 7; i++) {
    controller_speeds[i] = pulseIn(i + first_controller_pin, HIGH, 25000);
  }
}

int get_controller_speed(int i) {
    int value = map(controller_speeds[i], 1000, 1900, 0, 255);
    return max(0, min(255, value));
}

void print_controller_speeds() {
 for(int i = 0; i <= 7; i++) {
    Serial.print(i + first_controller_pin);
    Serial.print(" ");
    Serial.print(controller_names[i]);
    Serial.print(" ");
    //Serial.print(controller_speeds[i]);
    
    Serial.print(get_controller_speed(i));
    Serial.println(); 
  }
  Serial.println();
}

void updateArcadeDrive(int x, int y) {
  int xx = x - 128;
  int yy = 128 - y;
  
  int l = yy - xx;
  int r = yy + xx;
  
  l = map(l, -128, 128, 0, 180);
  r = map(r, -128, 128, 0, 180);
  
  l = max(0, min(180, l));
  r = max(0, min(180, r));
  
  set_left(l);
  set_right(r);
}

void set_left(int l) {
  int ll = 90;
  if(l > 90 + gap) {
    ll = map(l, 90 + 1 + gap, 180, 90, 90 - 90);
  } else if(l < 90 - gap) {
    ll = map(l, 0, 90 - 1 - gap, 90 + 90, 90); 
  }
  Serial.print("Left ");
  Serial.println(ll);
  left1.write(ll);
  left2.write(ll);
  left3.write(ll);
}

void set_right(int r) {
  int rr = 90;
  if(r < 90 - gap) {
    rr = map(r, 90 - 1 - gap, 0, 90, 90 - 90);
  } else if(r > 90 + gap) {
    rr = map(r, 180, 90 + 1 + gap, 90 + 90, 90); 
  }
  Serial.print("Right ");
  Serial.println(rr);  
  right1.write(rr);
  right2.write(rr);
  right3.write(rr);
}

void loop() {

  update_controller_speeds();
  //update_controller_speeds_min_max();
  //print_controller_speeds();
  //print_controller_speeds_min_max();
  
  if(digitalRead(25) == HIGH) {  
    int x = get_controller_speed(7);
    int y = get_controller_speed(5);
    //Serial.print("X ");
    //Serial.println(x);
    //Serial.print("Y ");
    //Serial.println(y);
    updateArcadeDrive(x,y);
  } else {
    // controller is off
    set_left(90);
    set_right(90); 
  }
  delay(5);  
}
