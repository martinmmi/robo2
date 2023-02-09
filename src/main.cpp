///////////////////////////////////////////////////////////////
//////////////////////// ROS Robo Car /////////////////////////
///////////////////////////////////////////////////////////////

#include <Arduino.h>
#include <Servo.h>

#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/UInt16.h>

//#include <std_msgs/Bool.h>


ros::NodeHandle  node_handle;

std_msgs::String button_msg;
std_msgs::UInt16 mode_msg;

Servo myservo;

int minAngle = 700;
int maxAngle = 2150;
int rightDistance = 0, leftDistance = 0, middleDistance = 0;
int echoPin = 19;               //Old A4
int trigPin = 26;               //Old A5
int distance = 0;
int average = 0;
int stopDistance = 20;          //Change
int stopDistanceShort = 8;      //Change
int speedDistance = 100;        //Change
int backDistance = 20;          //Change
int i = 0;

#define SEA       34            //Old 3
#define ENA       35            //Old 5
#define ENB       14            //Old 6
#define IN1       12            //Old 7
#define IN2       13            //Old 8
#define IN3        0            //Old 9
#define IN4        2            //Old 11
#define LT_R      !digitalRead(23)
#define LT_M      !digitalRead(22)     //Old 4
#define LT_L      !digitalRead(21)    //Old 2
#define BUTTON    36
#define LED       25

#define carSpeedMin       0     //Change
#define carSpeed        100     //Change
#define carSpeedturn    180     //Change
#define carSpeedMax     255     //Change

bool activeLineSensor = false;
bool activeEyeSensor = false;
bool modeIdenEye = false;
bool modeIdenLine = false;
bool buttonState = false;
bool ledState = false;

long lastSlowup = 0;
long lastForward = 0;
long lastSlowdown = 0;
long lastBack = 0;
long lastLeft = 0;
long lastRight = 0;
long lastStop = 0;
long lastDistance = 0;
long lastSetSensorBack = 0;
long lastPublished = 0;

///////////////////////////////////////////////////////////////

//void myCallback(const std_msgs::Bool& msg);
//ros::Subscriber<std_msgs::Bool> mySubscriber("some_topic", &myCallback);

/*
void myCallback(const std_msgs::Bool& msg) {
    if (msg.data) {
        activeLineSensor = true;
    } else {
        activeLineSensor = false;
    }
}
*/

///////////////////////////////////////////////////////////////

int readRosRobo() {
  // Feed any data from ROS to ROBO.
  if (Serial.available()) {
    Serial.write(Serial.read());
    int data = Serial.read();
    return (int)data;
  }
}

void sendRoboRos() {
  // Feed any data from ROBO to ROS

  if (Serial.available())
    Serial.write(Serial.read());
  }

///////////////////////////////////////////////////////////////

int updateDistanceFast() {

  distance = 0;
  average = 0;

  digitalWrite(trigPin, LOW);     //0v
  delayMicroseconds(2);           //Emit 40 kHz sound
  digitalWrite(trigPin, HIGH);    //5v
  delayMicroseconds(10);          //for 10 microseconds
  digitalWrite(trigPin, LOW);   

  distance = pulseIn(echoPin, HIGH);  //detect a pulse 

  distance = distance / 74 / 2 * 2.54;  //Speed of sound 13511.81 inches/s
                                        //13511.81/10^6 inches per micro
                                        //0.01351181 inches per microsecond
                                        //74.0092341 microseconds per inch
                                        //dividing by 74 and dividing by 2 and multiple by 2.54 for cm
  
  return distance;
  
}  

///////////////////////////////////////////////////////////////

int updateDistance() {

  distance = 0;
  average = 0;

  for (int i = 0; i < 4; i++) {     //Build an Average

    digitalWrite(trigPin, LOW);     //0v
    delayMicroseconds(2);           //Emit 40 kHz sound
    digitalWrite(trigPin, HIGH);    //5v
    delayMicroseconds(10);          //for 10 microseconds
    digitalWrite(trigPin, LOW);   

    distance = pulseIn(echoPin, HIGH);  //detect a pulse 

    distance = distance / 74 / 2 * 2.54;  //Speed of sound 13511.81 inches/s
                                          //13511.81/10^6 inches per micro
    average += distance;                  //0.01351181 inches per microsecond
                                          //74.0092341 microseconds per inch
    delay(10);                             //dividing by 74 and dividing by 2 and multiple by 2.54 for cm
  }
  distance = average / 4;
  return distance;

}  

///////////////////////////////////////////////////////////////

void measureMiddleDistance () {

  if ((millis() - lastDistance > 1000)) {
    Serial.print("D: ");
    Serial.print(middleDistance);
    Serial.println(" cm");
    lastDistance = millis();
  }

}

///////////////////////////////////////////////////////////////

void slowup(){ 

  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);

  if (i < carSpeedMax) {

    for(i = carSpeed; i <= carSpeedMax; i++){ 
      analogWrite(ENB,i);
      analogWrite(ENA,i);
      
      middleDistance = updateDistanceFast();

      if (middleDistance <= stopDistance) {
        break;
      }

      delay(5);
    }
  }

  if ((millis() - lastSlowup > 1000)) {
      Serial.println("Sup");
      lastSlowup = millis();
  }
}

void forward(){ 

  analogWrite(ENA, carSpeed);
  analogWrite(ENB, carSpeed);
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);

  if ((millis() - lastForward > 1000)) {
      Serial.println("F");
      lastForward = millis();
  }
}

void slowdown() {

  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);

  if (i > carSpeed) {

    for(i = carSpeedMax; i >= carSpeed; i--){ 
      analogWrite(ENB,i);
      analogWrite(ENA,i);

      middleDistance = updateDistanceFast();

      if (middleDistance <= stopDistance) {
        break;
      }

      delay(5);
    }
  }

  if ((millis() - lastSlowdown > 1000)) {
      Serial.println("Sdown");
      lastSlowdown = millis();
  }
}

void back() {
 
  analogWrite(ENA, carSpeedturn);
  analogWrite(ENB, carSpeedturn);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);

  if ((millis() - lastBack > 1000)) {
      Serial.println("B");
      lastBack = millis();
  }
}

void left() {
  analogWrite(ENA, carSpeedturn);
  analogWrite(ENB, carSpeedturn);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH); 

  if ((millis() - lastLeft > 1000)) {
      Serial.println("L");
      lastLeft = millis();
  }
}

void right() {
  analogWrite(ENA, carSpeedturn);
  analogWrite(ENB, carSpeedturn);
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);

  if ((millis() - lastRight > 1000)) {
      Serial.println("R");
      lastRight = millis();
  }
}

void stop() {
  digitalWrite(ENA, LOW);
  digitalWrite(ENB, LOW);

  if ((millis() - lastStop > 1000)) {
    Serial.println("S");
    lastStop = millis();
  }
} 

///////////////////////////////////////////////////////////////

void eyeFunction () {

  if (modeIdenEye == false) {
    Serial.println("");
    Serial.println("#######################");
    Serial.println("####### EYE MODE ######");
    Serial.println("#######################");
    Serial.println("");

  modeIdenEye = true;
  modeIdenLine = false;
  }

  middleDistance = updateDistance();
  measureMiddleDistance();

  if(middleDistance <= stopDistance) {      //Stop if something in the way
    stop();

    myservo.write(10);
    delay(500);
    rightDistance = updateDistance();

    myservo.write(180);
    delay(1000);
    leftDistance = updateDistance();

    myservo.write(90);
    delay(500);
    
    if(rightDistance > leftDistance) {
      right();
      delay(300 + random(150));
    }
    else if(rightDistance < leftDistance) {
      left();
      delay(300 + random(150));
    }
    else if((rightDistance <= backDistance) || (leftDistance <= backDistance)) {
      back();
      delay(150 + random(75));
    }
  }
  
  if(middleDistance > stopDistance){
    forward();
    delay(150 + random(75));
    
    while(middleDistance > speedDistance){
      slowup();
      middleDistance = updateDistance();
      measureMiddleDistance();
      if (middleDistance <= speedDistance) {
        slowdown();
        break;
      }
    }
  }
}

///////////////////////////////////////////////////////////////

void lineFunction () {

  if (modeIdenLine == false) {
    Serial.println("");
    Serial.println("#######################");
    Serial.println("###### LINE MODE ######");
    Serial.println("#######################");
    Serial.println("");

  modeIdenEye = false;
  modeIdenLine = true;
  }

  middleDistance = updateDistance();
  measureMiddleDistance();

  if(middleDistance <= stopDistanceShort) {      //Stop if something in the way
    stop();
  }

  if(middleDistance >= stopDistanceShort) {      //Stop if something in the way
  
    if(LT_M){
      forward();
    }

    else if(LT_R) { 
      right();
      while(LT_R);                             
    }

    else if(LT_L) {
      left();
      while(LT_L);  
    }
  }
}

///////////////////////////////////////////////////////////////

void buttonInterrupt() {
  buttonState = true;
  activeEyeSensor = false;
  activeLineSensor = false;
  stop();
  digitalWrite(LED, HIGH);
  delay(1000);
  digitalWrite(LED, LOW);
}

///////////////////////////////////////////////////////////////

void subscriberCallback(const std_msgs::UInt16& mode_msg) {

  if (mode_msg.data == 0) {
    digitalWrite(LED, HIGH);
    delay(100);
    digitalWrite(LED, LOW);
    delay(100);
    digitalWrite(LED, HIGH);
    delay(100);
    digitalWrite(LED, LOW);

    activeEyeSensor = false;
    activeLineSensor = false;
    stop();
  }
  
  if (mode_msg.data == 1) {
    digitalWrite(LED, HIGH); 
    delay(500);
    digitalWrite(LED, LOW); 

    activeEyeSensor = true;
    activeLineSensor = false;
  }

  if (mode_msg.data == 2) {
    digitalWrite(LED, HIGH);
    delay(500);
    digitalWrite(LED, LOW);
    delay(500);
    digitalWrite(LED, HIGH);
    delay(500);
    digitalWrite(LED, LOW);

    activeEyeSensor = false;
    activeLineSensor = true;
  }

}

///////////////////////////////////////////////////////////////

ros::Publisher button_publisher("button_press", &button_msg);
ros::Subscriber<std_msgs::UInt16> mode_subscriber("mode", &subscriberCallback);

///////////////////////////////////////////////////////////////

void setup() {

  Serial.begin(9600);
  //while (!Serial);
  Serial.println("");
  Serial.println("ROBO!");

  pinMode(LED, OUTPUT);
  pinMode(BUTTON, INPUT_PULLDOWN);

  attachInterrupt(digitalPinToInterrupt(BUTTON), buttonInterrupt, CHANGE);

  myservo.attach(SEA,minAngle,maxAngle);
  myservo.write(90);

  node_handle.initNode();
  node_handle.advertise(button_publisher);
  node_handle.subscribe(mode_subscriber);

  pinMode(echoPin, INPUT);    
  pinMode(trigPin, OUTPUT);

  pinMode(IN1,OUTPUT);
  pinMode(IN2,OUTPUT);
  pinMode(IN3,OUTPUT);
  pinMode(IN4,OUTPUT);
  pinMode(ENA,OUTPUT);
  pinMode(ENB,OUTPUT);

  pinMode(LT_R,INPUT);
  pinMode(LT_M,INPUT);
  pinMode(LT_L,INPUT);

  digitalWrite(ENA, HIGH); 
  digitalWrite(ENB, HIGH);

  delay(1500);

}

///////////////////////////////////////////////////////////////

void loop() {

  if (buttonState) {
      buttonState = false;
      Serial.println("Button released");
  }
  
  if (millis() - lastPublished > 1000) {
    button_publisher.publish( &button_msg );
    node_handle.spinOnce();

    lastPublished = millis();
  }

  if (activeEyeSensor == true) {
    eyeFunction();
    //sendRoboRos();
    //NodeMartin.spinOnce();

  }

  if (activeLineSensor == true) {
    lineFunction();
    //NodeMartin.spinOnce();

  }

}

///////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////